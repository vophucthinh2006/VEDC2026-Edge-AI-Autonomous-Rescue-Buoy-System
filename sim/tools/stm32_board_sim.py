#!/usr/bin/env python3
"""The STM32F407 controller of the STM32 boat in the simulator.

Stands behind a pseudo-terminal the Pi's STM32 runtime (Rasp_Pi/main.py) opens as
serial.stm32_port, speaks the firmware's UART protocol (STM32F4/README.md section 11,
STM32F4/Modules/PiLink/uart_protocol.c) and does in Gazebo what the F407 does on the boat
(STM32F4/App/control.c, heading_controller.c, actuators.c):

    $NAV AUTO, fresh   ->  heading PID on the buoy's yaw, Heading_Mix onto rudders and the two rear
                           thrusters, throttle = speed / 1 m/s capped at 0.5, front motor off
    $NAV STOP / stale  ->  motors off, rudders centred (NAV TTL, HBT 500 ms, IMU 100 ms)
    $CAM pan           ->  camera_pan_joint
    $TXD               ->  $ACK,seq,TXD,ACCEPTED, and a person event over LoRa
    every 100 ms       ->  $IMU, $GPS, $SYS (battery SIM_BATTERY_V) from the buoy's Gazebo odometry
    every second       ->  $LNK: the shore link and the way home (App/shore_link.c)
    with --radio-lib   ->  LoRa beacon to tools/shore_station_sim.py, route frames back
                           (App/route_link.c itself, through ctypes), $RTE lines to the Pi until
                           its $HBT reports the route (docs/lora-route-downlink.md); events (a
                           person, overturned, the confirm button) in frames of their own until
                           the station's ACK (App/event_queue.c), whose lights it drives

There is no RC transmitter, as on the boat (STM32F4/App/control.c): the board is armed whenever it is
safe and AUTO is the only mode; the motors turn while the Pi drives. PE4 is the victim's confirm button, not an E-stop: a file named
"confirm" in --dir is one press (counted in $SYS, then the file is removed).
Everything the board decides is appended to <dir>/stm32_events.log, a status line a second goes
to stdout.

    python3 stm32_board_sim.py --pty /tmp/vedc_stm32 --home 10.883438,106.796019 \
        --world vedc_lake --start 25,3,90
"""
from __future__ import annotations

import argparse
import math
import os
import select
import signal
import sys
import threading
import time
import tty
from pathlib import Path

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

# App/app_config.h
NAV_SPEED_MAX = 2.0
NAV_TIMEOUT_S, HBT_TIMEOUT_S, IMU_TIMEOUT_S = 0.3, 0.5, 0.1
CONTROL_PERIOD_S, TELEMETRY_PERIOD_S = 0.01, 0.1
MAX_PITCH_DEG = MAX_ROLL_DEG = 35.0
KP, KI, KD = 0.020, 0.002, 0.008
DEADBAND_DEG, INTEGRAL_LIMIT, RATE_LPF_HZ = 2.0, 100.0, 5.0
RUDDER_GAIN, DIFF_GAIN, DIFF_DIR = 1.0, 0.30, +1
RUDDER_SPAN_US, RUDDER_FRONT_GAIN = 250.0, 1.0
ESC_RAMP_PER_S = 2.0
# App/front_motor.c: the bidirectional front motor pushes with the rear ones and brakes on a throttle cut.
FRONT_THRUST_GAIN, FRONT_BRAKE_LEVEL, FRONT_BRAKE_MIN_DROP = 1.0, 0.40, 0.05
FRONT_BACK_MAX = 0.40         # NAV BACK: the most the front motor pulls astern
FRONT_BRAKE_S_PER_THROTTLE, FRONT_BRAKE_MAX_S, ESC_FRONT_REVERSE_DELAY_S = 2.0, 1.0, 0.2
FRONT_BRAKE_HOLD_S = 1.0
CAM_PAN_LIMIT_DEG = 80.0
AUTO_THROTTLE_MAX = 0.50
TILT_REPEAT_S = 30.0
LORA_EVENT_RETRY_S = 1.5
LORA_DIAG_PERIOD_S = 10.0     # LORA_DIAG_PERIOD_MS: the diagnostics frame (App/diag_frame.h)
DIAG_PI_SILENT_S = 2.0        # DIAG_PI_SILENT_MS: no $HBT this long, the diagnostics say pi_silent
SIM_BATTERY_V, BATTERY_CELLS = 11.9, 3
CELL_VOLTS = (3.27, 3.61, 3.69, 3.71, 3.73, 3.77, 3.79, 3.84, 3.92, 4.02, 4.20)   # App/battery.c
# sim/models/vedc_buoy/model.sdf: a pod turns 45 deg for 500 us, positive cmd_pos (anticlockwise
# seen from above) on the rear pods turns the bow right; the front pod swings the other way.
POD_RAD_PER_US = math.radians(45.0) / 500.0
EARTH_RADIUS_M = 6378137.0
TOPIC = "/model/vedc_buoy/joint/{}/{}"


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def checksum(body: str) -> str:
    value = 0
    for char in body:
        value ^= ord(char)
    return f"{value:02X}"


def heading_error(target_deg: float, current_deg: float) -> float:
    return math.fmod(target_deg - current_deg + 540.0, 360.0) - 180.0


class HeadingController:
    """App/heading_controller.c, line for line: u in [-1, 1], positive turns the bow right."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.integral, self.rate, self.primed = 0.0, 0.0, False

    def update(self, target_deg: float, current_deg: float, yaw_rate_dps: float, dt: float) -> float:
        error = heading_error(target_deg, current_deg)
        if not self.primed:
            self.rate, self.primed = yaw_rate_dps, True
        else:
            tau = 1.0 / (2.0 * math.pi * RATE_LPF_HZ)
            self.rate += dt / (tau + dt) * (yaw_rate_dps - self.rate)
        d = -KD * self.rate
        if abs(error) <= DEADBAND_DEG:
            return clamp(KI * self.integral + d, -1.0, 1.0)
        candidate = clamp(self.integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        raw = KP * error + KI * candidate + d
        output = clamp(raw, -1.0, 1.0)
        if raw == output or (output >= 1.0 and error < 0.0) or (output <= -1.0 and error > 0.0):
            self.integral = candidate
        return output


def cell_percent(volts: float) -> int:
    """Battery_CellPercent of App/battery.c."""
    if volts <= CELL_VOLTS[0]:
        return 0
    if volts >= CELL_VOLTS[-1]:
        return 100
    for i in range(len(CELL_VOLTS) - 1):
        if volts < CELL_VOLTS[i + 1]:
            return int(10.0 * (i + (volts - CELL_VOLTS[i]) / (CELL_VOLTS[i + 1] - CELL_VOLTS[i])) + 0.5)
    return 100


class FrontMotor:
    """App/front_motor.c, line for line: front command -1..1, positive pushes forward."""

    def __init__(self) -> None:
        self.window_start = 0.0
        self.reset()

    def reset(self) -> None:
        self.last, self.until, self.braking = 0.0, 0.0, False
        self.floor = self.window_min = 0.0

    def command(self, throttle: float, now: float) -> float:
        throttle = max(throttle, 0.0)
        # Only the throttle held for a whole FRONT_BRAKE_HOLD_S window is braked off.
        drop = min(self.floor, self.last) - throttle
        if drop >= FRONT_BRAKE_MIN_DROP:
            self.until = now + ESC_FRONT_REVERSE_DELAY_S + min(drop * FRONT_BRAKE_S_PER_THROTTLE, FRONT_BRAKE_MAX_S)
            self.braking = True
        elif throttle > self.last:
            self.braking = False
        self.last = throttle
        self.floor = min(self.floor, throttle)
        self.window_min = min(self.window_min, throttle)
        if now - self.window_start >= FRONT_BRAKE_HOLD_S:
            self.floor, self.window_min, self.window_start = self.window_min, throttle, now
        if self.braking:
            if now < self.until:
                return -FRONT_BRAKE_LEVEL
            self.braking = False
        return throttle * FRONT_THRUST_GAIN


def heading_mix(u: float, throttle: float) -> tuple[float, float, float]:
    """Heading_Mix: rear left, rear right (0..1) and steer (-1..1)."""
    throttle = clamp(throttle, 0.0, 1.0)
    room = min(throttle, 1.0 - throttle)
    diff = clamp(u * DIFF_GAIN * DIFF_DIR, -room, room)
    return throttle + diff, throttle - diff, clamp(u * RUDDER_GAIN, -1.0, 1.0)


class Buoy:
    """The buoy's state from /model/vedc_buoy/odometry: world x east, y north (ENU), sim time."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.wall = 0.0                 # monotonic time of the last pose
        self.stamp: float | None = None
        self.x = self.y = 0.0
        self.heading = self.pitch = self.roll = 0.0
        self.yaw_rate = 0.0             # deg/s, clockwise seen from above (BNO055 gyro Z sign)
        self.ve = self.vn = 0.0

    def on_odometry(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pitch = math.degrees(math.asin(clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0)))
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        heading = (90.0 - math.degrees(yaw)) % 360.0
        with self.lock:
            if self.stamp is not None and stamp > self.stamp:
                dt = stamp - self.stamp
                a = min(1.0, dt / 0.2)   # 0.2 s smoothing: the waves rock the finite differences
                self.ve += a * ((p.x - self.x) / dt - self.ve)
                self.vn += a * ((p.y - self.y) / dt - self.vn)
                self.yaw_rate += min(1.0, dt / 0.05) * (heading_error(heading, self.heading) / dt - self.yaw_rate)
            self.stamp, self.wall = stamp, time.monotonic()
            self.x, self.y, self.heading, self.pitch, self.roll = p.x, p.y, heading, pitch, roll


class Board:
    def __init__(self, args: argparse.Namespace, node: Node) -> None:
        self.args = args
        self.buoy = Buoy()
        node.subscribe(Odometry, "/model/vedc_buoy/odometry", self.buoy.on_odometry)
        self.pub = {name: node.advertise(topic, Double) for name, topic in (
            ("left", TOPIC.format("prop_rear_left_joint", "cmd_thrust")),
            ("right", TOPIC.format("prop_rear_right_joint", "cmd_thrust")),
            ("front", TOPIC.format("prop_front_joint", "cmd_thrust")),
            ("pod_left", TOPIC.format("pod_rear_left_joint", "cmd_pos")),
            ("pod_right", TOPIC.format("pod_rear_right_joint", "cmd_pos")),
            ("pod_front", TOPIC.format("pod_front_joint", "cmd_pos")),
            ("pan", TOPIC.format("camera_pan_joint", "cmd_pos")))}
        self.pid = HeadingController()
        # pi_command_t of uart_protocol.h
        self.nav_mode, self.nav_speed, self.nav_heading, self.nav_ttl = "STOP", 0.0, 0.0, 0.0
        self.nav_seq, self.last_nav = 0, 0.0
        self.last_hbt, self.link_ok = 0.0, False
        self.cam_pan: float | None = None
        self.victims = 0
        self.pi_route_rev = 0
        # $HBT's reason, mission and waypoint, and why the board does not drive: App/diag_frame.h
        self.pi_reason, self.pi_mission, self.pi_wp = "", "", "0/0"
        self.pi_fps: int | None = None
        self.arm_block, self.drive_block = 1, 1
        self.lora_seq = 0
        self.lora: Lora | None = None
        # Actuator outputs after the ESC ramp
        self.esc = [0.0, 0.0]
        self.front_motor, self.front = FrontMotor(), 0.0
        self.steer = 0.0
        self.state = "S"
        self.u = 0.0
        self.armed = False
        self.estop = False          # PE4 is the confirm button now: estop always 0 (App/control.c)
        self.confirm_presses = 0
        self.was_overturned, self.last_tilt = False, 0.0
        self.lights = 0               # App/lights.c: RELAY1 PC3 bit 0, RELAY2 PC2 bit 1
        self.events = (args.dir / "stm32_events.log").open("a", buffering=1)
        self.seq_drops = 0

    def event(self, text: str) -> None:
        self.events.write(f"{time.time():.2f} {text}\n")

    def send(self, fd: int, command: str, *fields: object) -> None:
        body = ",".join((command, *(str(field) for field in fields)))
        try:
            os.write(fd, f"${body}*{checksum(body)}\n".encode("ascii"))
        except (BlockingIOError, OSError):
            pass                        # nobody reads the port: the report is lost, as on a real UART

    # ---- Modules/PiLink/uart_protocol.c -------------------------------------------------------
    def feed_line(self, fd: int, line: str, now: float) -> None:
        line = line.strip("\r")
        if not line.startswith("$") or "*" not in line:
            return
        body, _, received = line[1:].partition("*")
        if len(received) != 2 or checksum(body) != received.upper():
            return
        fields = [field for field in body.split(",") if field]   # strtok skips empty fields
        if len(fields) < 2:
            return
        kind = fields[0]
        try:
            if kind == "NAV" and len(fields) >= 6:
                seq = int(fields[1]) & 0xFFFF
                # Old packets are only dropped while the stream is live: a restarted Pi counts from 1 again.
                stream_live = self.last_nav != 0.0 and now - self.last_nav <= NAV_TIMEOUT_S
                if stream_live and ((seq - self.nav_seq) & 0xFFFF) >= 0x8000:
                    self.seq_drops += 1
                    if self.seq_drops in (1, 10) or self.seq_drops % 100 == 0:
                        self.event(f"NAV seq {seq} not newer than {self.nav_seq}: dropped ({self.seq_drops})")
                    return
                mode = fields[2] if fields[2] in ("AUTO", "MANUAL", "BACK") else "STOP"
                speed, heading, ttl = float(fields[3]), float(fields[4]), int(fields[5])
                if not (0.0 <= speed <= NAV_SPEED_MAX and 0.0 <= heading < 360.0 and 0 < ttl <= 1000):
                    return
                if mode != self.nav_mode:
                    self.event(f"NAV {mode} {speed:.2f} m/s {heading:.1f} deg")
                self.nav_mode, self.nav_speed, self.nav_heading, self.nav_ttl = mode, speed, heading, ttl / 1000.0
                self.nav_seq, self.last_nav, self.seq_drops = seq, now, 0
            elif kind == "HBT" and len(fields) >= 3:
                self.last_hbt, self.link_ok = now, True
                # Optional 4th field: the LoRa route the Pi runs (docs/lora-route-downlink.md).
                self.pi_route_rev = int(fields[3]) & 0xFFFF if len(fields) >= 4 else 0
                # The reason of the NAV command, then the mission and the waypoint (uart_protocol.c)
                self.pi_reason = "".join(c for c in fields[2] if c.islower() or c.isdigit() or c == "_")[:23]
                if len(fields) >= 5 and len(fields[4]) == 1:
                    self.pi_mission = fields[4]
                if len(fields) >= 6:
                    self.pi_wp = fields[5]
                # 7th field: the detector's frames a second x10, "-" while it is off
                self.pi_fps = int(fields[6]) if len(fields) >= 7 and fields[6].isdigit() and int(fields[6]) <= 999 else None
            elif kind == "CAM" and len(fields) >= 3:
                pan = float(fields[2])
                if -90.0 <= pan <= 90.0:
                    self.cam_pan = pan
            elif kind == "TXD" and len(fields) >= 6:
                if fields[2] in ("VICTIM_FOUND", "VICTIM_UPDATE"):
                    if fields[2] == "VICTIM_FOUND":   # a new person; UPDATE: the one followed, not counted again
                        self.victims = min(self.victims + 1, 255)
                    # $TXD,seq,VICTIM_FOUND|VICTIM_UPDATE,lat,lon,confidence[,risk]: 0, 0 is no position; no risk: 2.
                    lat, lon, confidence = float(fields[3]), float(fields[4]), float(fields[5])
                    risk = min(int(fields[6]), 3) if len(fields) >= 7 else 2
                    if self.lora is not None:
                        position = (lat, lon) if (lat or lon) and abs(lat) <= 90 and abs(lon) <= 180 else None
                        self.lora.event_now(self.lora.events.victim(position, risk, round(clamp(confidence, 0.0, 1.0) * 100), self.victims))
                self.event(f"TXD {','.join(fields[2:])} (victims {self.victims})")
                self.send(fd, "ACK", int(fields[1]) & 0xFFFF, "TXD", "ACCEPTED")
        except ValueError:
            return

    # ---- App/control.c ------------------------------------------------------------------------
    def control(self, now: float, dt: float) -> None:
        b = self.buoy
        with b.lock:
            imu_fresh = b.stamp is not None and now - b.wall <= IMU_TIMEOUT_S
            heading, rate, pitch, roll = b.heading, b.yaw_rate, b.pitch, b.roll
        confirm = self.args.dir / "confirm"
        confirm_pressed = confirm.exists()
        if confirm_pressed:
            confirm.unlink()
            self.confirm_presses += 1
            self.event(f"confirm button pressed ({self.confirm_presses})")
        overturned = imu_fresh and (abs(pitch) > MAX_PITCH_DEG or abs(roll) > MAX_ROLL_DEG)
        if self.lora is not None:
            # Core/Src/main.c report_events: overturned (again every 30 s), the confirm button.
            if confirm_pressed:
                self.lora.event_now(self.lora.events.confirm(self.position(), self.confirm_presses))
            if overturned and (not self.was_overturned or now - self.last_tilt >= TILT_REPEAT_S):
                self.lora.event_now(self.lora.events.tilt(self.position(), roll, pitch))
                self.last_tilt = now
                self.event(f"overturned: roll {roll:.1f} pitch {pitch:.1f}, event to the shore")
        self.was_overturned = overturned
        # No RC: armed whenever it is safe, AUTO the only mode (App/control.c).
        armed = imu_fresh and not self.estop and not overturned
        throttle, u, state = 0.0, 0.0, "S"
        held = self.lora is not None and self.lora.shore.hold   # STOP from the dashboard (App/control.c)
        # arm_block_t and drive_block_t of App/control.h, for the diagnostics frame
        self.arm_block = 0 if armed else 2 if overturned else 1
        self.drive_block = 1 if not armed else 4 if held else 2
        backing = False
        if armed and not held:
            nav_fresh = self.nav_mode in ("AUTO", "BACK") and now - self.last_nav <= self.nav_ttl
            hbt_fresh = self.link_ok and now - self.last_hbt <= HBT_TIMEOUT_S
            if imu_fresh and nav_fresh and hbt_fresh:
                state = "A"
                self.drive_block = 0
                # NAV BACK: the front motor alone pulls the boat astern, rudders centred (App/control.c).
                backing = self.nav_mode == "BACK"
                throttle = 0.0 if backing else clamp(self.nav_speed / 1.0, 0.0, AUTO_THROTTLE_MAX)
        if state == "A" and throttle > 0.0:
            if self.state != "A":
                self.pid.reset()
            u = self.pid.update(self.nav_heading, heading, rate, dt)
            left, right, steer = heading_mix(u, throttle)
        else:
            # Stopped, or AUTO at speed 0: no pivoting, motors rest and the rudders centre.
            self.pid.reset()
            left = right = steer = 0.0
        if state != self.state:
            self.event({"A": "AUTO: steering", "S": "stop"}[state]
                       + ("" if state == "A" else f" (estop {int(self.estop)}, imu {int(imu_fresh)}, nav {self.nav_mode}"
                          f" {now - self.last_nav:.2f} s old, hbt {now - self.last_hbt:.2f} s old)"))
        # The front motor: off at once on a fault, else it pushes with the rear ones or brakes.
        if not armed:
            self.front_motor.reset()
            front_target = 0.0
        elif backing:
            self.front_motor.reset()
            front_target = -clamp(self.nav_speed, 0.0, FRONT_BACK_MAX)
        else:
            front_target = self.front_motor.command(throttle if state == "A" else 0.0, now)
        if front_target < 0.0 <= self.front:
            self.event(f"front motor brakes ({FRONT_BRAKE_LEVEL:.2f} backwards)")
        # Actuators_Apply for the front ESC: through neutral before it turns the other way, magnitude
        # rises at ESC_RAMP_PER_S, cuts are immediate.
        if self.front != 0.0 and front_target != 0.0 and (front_target > 0) != (self.front > 0):
            self.front = 0.0
        elif abs(front_target) <= abs(self.front):
            self.front = front_target
        else:
            self.front = math.copysign(min(abs(front_target), abs(self.front) + ESC_RAMP_PER_S * dt), front_target)
        self.state, self.u, self.armed = state, u, armed
        # Actuators_Apply: magnitudes rise at ESC_RAMP_PER_S, cuts are immediate.
        for index, target in enumerate((left, right)):
            self.esc[index] = target if target <= self.esc[index] else min(target, self.esc[index] + ESC_RAMP_PER_S * dt)
        self.steer = steer
        self.publish()

    def publish(self) -> None:
        def put(name: str, value: float) -> None:
            msg = Double()
            msg.data = value
            self.pub[name].publish(msg)
        put("left", self.esc[0] * self.args.rear_thrust_n)
        put("right", self.esc[1] * self.args.rear_thrust_n)
        put("front", self.front * self.args.rear_thrust_n)
        pod = self.steer * RUDDER_SPAN_US * POD_RAD_PER_US
        put("pod_left", pod)
        put("pod_right", pod)
        put("pod_front", -pod * RUDDER_FRONT_GAIN)
        if self.cam_pan is not None:   # Camera_Tick: independent of arming, holds the last angle
            put("pan", math.radians(clamp(self.cam_pan, -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG)))

    def position(self) -> tuple[float, float] | None:
        """The boat's GPS position (None without a fix), as lora_beacon.c boat_position."""
        if self.args.gps_fix < 1:
            return None
        with self.buoy.lock:
            x, y = self.buoy.x, self.buoy.y
        lat0, lon0 = self.args.home
        return (lat0 + math.degrees(y / EARTH_RADIUS_M), lon0 + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(lat0)))))

    # ---- Core/Src/main.c send_telemetry -------------------------------------------------------
    def telemetry(self, fd: int, seq: int, now: float) -> None:
        b = self.buoy
        with b.lock:
            imu_ok = b.stamp is not None and now - b.wall <= IMU_TIMEOUT_S
            x, y, heading, pitch, roll, ve, vn = b.x, b.y, b.heading, b.pitch, b.roll, b.ve, b.vn
        overturned = imu_ok and (abs(pitch) > MAX_PITCH_DEG or abs(roll) > MAX_ROLL_DEG)
        lat0, lon0 = self.args.home
        lat = lat0 + math.degrees(y / EARTH_RADIUS_M)
        lon = lon0 + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
        speed = math.hypot(ve, vn)
        course = math.degrees(math.atan2(ve, vn)) % 360.0
        self.send(fd, "IMU", seq, f"{pitch:.2f}", f"{roll:.2f}", f"{heading:.2f}", int(imu_ok), int(overturned))
        self.send(fd, "GPS", seq, f"{lat:.6f}", f"{lon:.6f}", self.args.gps_fix, "0.9", f"{speed:.2f}", f"{course:.1f}")
        volts = self.battery_v()
        self.send(fd, "SYS", seq, f"{volts:.2f}", 0, int(self.estop), int(self.link_ok), int(self.armed), 1,
                  self.confirm_presses, cell_percent(volts / BATTERY_CELLS))
        if self.lora is not None and seq % 10 == 0:
            # Once a second the shore link and the way home: the Pi takes the boat home when it is lost.
            self.send(fd, "LNK", self.lora.shore.pi_line(seq, int(now * 1000)))
        if self.lora is not None:
            # Route from the dashboard: one $RTE line a period until $HBT reports its revision.
            self.lora.route.set_pi_rev(self.pi_route_rev)
            line = self.lora.route.next_pi_line(seq, int(now * 1000))
            if line is not None:
                self.send(fd, "RTE", line)
                if line.split(",")[2] == "0":
                    self.event(f"RTE rev {line.split(',')[1]} to the Pi ({line.split(',')[3]} points)")

    def battery_v(self) -> float:
        """The pack: SIM_BATTERY_V, or the volts written in a file named battery in --dir (a low pack
        to try the way home: echo 10.6 > ~/vedc_sim/sitl_run/battery)."""
        try:
            return float((self.args.dir / "battery").read_text().strip())
        except (OSError, ValueError):
            return SIM_BATTERY_V

    def diag(self) -> str:
        """App/diag_frame.c Diag_SetPi and DiagFrame_Build (n, s, w, a, d, u, j, k)."""
        if self.link_ok and time.monotonic() - self.last_hbt <= DIAG_PI_SILENT_S:
            reason, mission, wp, fps = self.pi_reason or "-", self.pi_mission, self.pi_wp, self.pi_fps
        else:
            reason, mission, wp, fps = "pi_silent", "", "0/0", None      # the Pi's last word no longer holds
        mission = mission if mission in ("I", "P", "F", "R", "B", "H") else "-"
        return (f"dg={self.args.buoy_id},n={reason},s={mission},w={wp},a={self.arm_block},"
                f"d={self.drive_block},u={round(self.battery_v() * 100)},j=0,k=0:0" + ("" if fps is None else f",f={fps}"))

    def beacon(self, now: float) -> str:
        """App/lora_beacon.c build_frame, with the route field."""
        b = self.buoy
        with b.lock:
            imu_ok = b.stamp is not None and now - b.wall <= IMU_TIMEOUT_S
            x, y, heading, pitch, roll = b.x, b.y, b.heading, b.pitch, b.roll
            ve, vn = b.ve, b.vn
        lat0, lon0 = self.args.home
        lat = lat0 + math.degrees(y / EARTH_RADIUS_M)
        lon = lon0 + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
        target = round(self.nav_heading, 1) % 360.0 if self.state == "A" else -1.0
        mode = "R" if self.state == "A" and not self.lora.shore.ok(int(now * 1000)) else self.state
        self.lora_seq += 1
        frame = (f"id={self.args.buoy_id},{lat:.6f},{lon:.6f},r={roll if imu_ok else 0:.1f},p={pitch if imu_ok else 0:.1f},"
                 f"y={round(heading, 1) % 360.0 if imu_ok else 0:.1f},t={target:.1f},m={mode},i={int(imu_ok)},c={3 if imu_ok else 0},"
                 f"q={self.lora_seq},v={self.victims}")
        # The optional fields, each only while there is room, most needed first.
        for field in (self.lora.route.beacon_field(), f"H={int(self.lora.shore.hold)}", f"L={self.lights}", f"b={cell_percent(self.battery_v() / BATTERY_CELLS)}",
                      f"g={round(math.degrees(math.atan2(ve, vn)), 1) % 360.0:.1f}", f"s={math.hypot(ve, vn):.2f}"):
            if field and len(frame) + 1 + len(field) < 120:
                frame += "," + field
        return frame

    def status(self, now: float) -> str:
        b = self.buoy
        return (f"{self.state} nav={self.nav_mode} {self.nav_speed:.2f}m/s->{self.nav_heading:5.1f} ({now - self.last_nav:4.2f}s)"
                f" yaw={b.heading:5.1f} u={self.u:+.2f} esc={self.esc[0]:.2f}/{self.esc[1]:.2f} steer={self.steer:+.2f}"
                f" v={math.hypot(b.ve, b.vn):.2f} pos=({b.x:.1f},{b.y:.1f}) pan={self.cam_pan if self.cam_pan is not None else 0:.0f}")


class Lora:
    """App/lora_beacon.c over the simulated radio (tools/lora_radio.py): a beacon every 5 s, every
    1.5 s while a route is coming in, events at once and again every 1.5 s until acknowledged; after
    each frame 1.5 s of listening for the station's answer, an RTE frame or an ACK."""

    PERIOD_S, ROUTE_PERIOD_S, RX_WINDOW_S = 5.0, 1.5, 1.5

    def __init__(self, board: Board, lib: Path) -> None:
        import socket
        from lora_radio import BEACON_PORT, DOWNLINK_PORT, BuoyEvents, BuoyRoute, BuoyShore, airtime_s
        self.board, self.route, self.airtime_s = board, BuoyRoute(lib), airtime_s
        self.shore, self.events = BuoyShore(lib), BuoyEvents(lib)
        self.event_due = 0.0
        self.was_ok = True
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", DOWNLINK_PORT))
        self.rx.setblocking(False)
        self.station = ("127.0.0.1", BEACON_PORT)
        self.state, self.next_tx, self.until, self.frame = "idle", time.monotonic() + 1.0, 0.0, ""
        self.next_diag, self.listen = self.next_tx + self.PERIOD_S / 2, True

    def _received(self) -> list[bytes]:
        out = []
        while True:
            try:
                out.append(self.rx.recv(256))
            except BlockingIOError:
                return out

    def event_now(self, eseq: int) -> None:
        self.event_due = time.monotonic()
        self.board.event(f"LoRa event {eseq} queued ({self.events.pending} waiting)")

    def tick(self, now: float) -> None:
        now_ms = int(now * 1000)
        ok = self.shore.ok(now_ms)
        if ok != self.was_ok:
            self.board.event("shore station heard again" if ok else "shore station lost: silent for 60 s, $LNK shore_ok 0")
            self.was_ok = ok
        if self.state == "idle":
            self._received()                     # not listening: the radio hears nothing
            frame = ""
            if self.events.pending and now >= self.event_due:
                frame = self.events.frame(self.board.args.buoy_id)
                self.event_due = now + LORA_EVENT_RETRY_S
            self.listen = True
            if not frame and now >= self.next_tx:
                frame = self.board.beacon(now)
                self.next_tx = now + (self.ROUTE_PERIOD_S if self.route.receiving else self.PERIOD_S)
            elif not frame and now >= self.next_diag and not self.route.receiving:
                # App/lora_beacon.c: the diagnostics between two beacons, unanswered
                frame, self.listen = self.board.diag(), False
                self.next_diag = now + LORA_DIAG_PERIOD_S
            if not frame:
                return
            self.frame = frame
            self.state, self.until = "tx", now + self.airtime_s(len(self.frame))
        elif self.state == "tx":
            self._received()
            if now < self.until:
                return
            self.tx.sendto(self.frame.encode(), self.station)   # the station has it once it is off the air
            self.state, self.until = ("rx", now + self.RX_WINDOW_S) if self.listen else ("idle", now)
        else:
            # One answer per frame: listening stops once ours is in. An answer to another buoy on the
            # channel (the station's ACK to PHAO-03 just before our RTE) is passed over.
            ours = False
            for frame in self._received():
                if ours:
                    break
                ours = True
                was_receiving = self.route.receiving
                eseq = None
                if self.route.on_frame(frame, self.board.args.buoy_id):
                    self.shore.heard(now_ms)
                    self.board.event(f"LoRa RTE taken: rev {self.route.rev}, {self.route.have}/{self.route.total} points")
                    if not self.route.receiving and (was_receiving or self.route.total == 0):
                        self.next_tx = now       # tell the station at once that the route is complete
                elif (eseq := self.shore.on_frame(frame, self.board.args.buoy_id, now_ms)) is not None:
                    if self.shore.hold != getattr(self, "_was_held", False):
                        self.board.event("STOPPED from the dashboard: motors off" if self.shore.hold else "released from the dashboard")
                        self._was_held = self.shore.hold
                    if self.shore.lights != self.board.lights:
                        self.board.event(f"lights {self.shore.lights} (light 1 {'on' if self.shore.lights & 1 else 'off'}, "
                                         f"light 2 {'on' if self.shore.lights & 2 else 'off'})")
                        self.board.lights = self.shore.lights
                    if eseq and self.events.ack(eseq):
                        self.board.event(f"LoRa event {eseq} acknowledged")
                        if self.events.pending:
                            self.event_due = now
                else:
                    ours = False
                    self.board.event(f"LoRa frame refused: {frame[:60]!r}")
            if ours or now >= self.until:
                self.state = "idle"


def set_start(node: Node, world: str, east: float, north: float, heading_deg: float) -> bool:
    """Puts the buoy at a start pose: east, north in metres from the world origin, compass heading."""
    req = Pose()
    req.name = "vedc_buoy"
    req.position.x, req.position.y, req.position.z = east, north, 0.0
    yaw = math.radians(90.0 - heading_deg)
    req.orientation.w, req.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
    ok, rep = node.request(f"/world/{world}/set_pose", req, Pose, Boolean, 3000)
    return bool(ok and rep.data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pty", required=True, type=Path, help="path the Pi opens as serial.stm32_port (a link to the pseudo-terminal)")
    parser.add_argument("--home", required=True, help="lat,lon of the Gazebo world origin (run_boat.sh HOME_LOCATION)")
    parser.add_argument("--dir", type=Path, default=Path("."), help="where stm32_events.log goes and the confirm file is looked for")
    parser.add_argument("--world", default="vedc_lake", help="Gazebo world name, for --start")
    parser.add_argument("--start", help="east,north,heading: move the buoy there first (metres from the origin, compass degrees)")
    parser.add_argument("--gps-fix", type=int, default=1, help="GGA fix quality the board forwards (1 GPS, 2 DGPS)")
    parser.add_argument("--rear-thrust-n", type=float, default=15.0, help="thrust of one rear motor at full throttle (model.sdf guess)")
    parser.add_argument("--radio-lib", type=Path, help="librouteradio.so: with it the board beacons over the simulated LoRa "
                        "(tools/lora_radio.py) and takes routes from tools/shore_station_sim.py")
    parser.add_argument("--buoy-id", default="PHAO-01", help="LORA_BUOY_ID")
    args = parser.parse_args()
    args.home = tuple(float(value) for value in args.home.split(",")[:2])

    node = Node()
    board = Board(args, node)
    if args.radio_lib:
        board.lora = Lora(board, args.radio_lib)
    if args.start:
        east, north, heading = (float(value) for value in args.start.split(","))
        for _ in range(10):
            if set_start(node, args.world, east, north, heading):
                board.event(f"start pose {east:.1f} E {north:.1f} N heading {heading:.0f}")
                break
            time.sleep(1.0)
        else:
            print(f"could not move the buoy: /world/{args.world}/set_pose did not answer", file=sys.stderr, flush=True)

    master, slave = os.openpty()
    tty.setraw(slave)                 # no echo, no line editing: bytes as on a UART
    if args.pty.is_symlink() or args.pty.exists():
        args.pty.unlink()
    args.pty.symlink_to(os.ttyname(slave))
    os.set_blocking(master, False)
    print(f"STM32F407 board on {args.pty} -> {os.ttyname(slave)}, no RC: AUTO", flush=True)
    board.event("board up, no RC: AUTO")

    signal.signal(signal.SIGTERM, signal.default_int_handler)   # the launcher stops it with SIGTERM
    pending, seq = b"", 0
    last_control = next_telemetry = next_status = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            readable, _, _ = select.select([master], [], [], max(0.0, last_control + CONTROL_PERIOD_S - now))
            now = time.monotonic()
            if readable:
                try:
                    pending += os.read(master, 4096)
                except (BlockingIOError, OSError):
                    pass
                *lines, pending = pending.split(b"\n")
                pending = pending[-192:]
                for line in lines:
                    board.feed_line(master, line.decode("ascii", "replace"), now)
            if now - last_control >= CONTROL_PERIOD_S:
                board.control(now, min(now - last_control, 0.05))
                last_control = now
            if board.lora is not None:
                board.lora.tick(now)
            if now >= next_telemetry:
                seq = (seq + 1) & 0xFFFF
                board.telemetry(master, seq, now)
                next_telemetry += TELEMETRY_PERIOD_S
                if next_telemetry < now:
                    next_telemetry = now + TELEMETRY_PERIOD_S
            if now >= next_status:
                print(board.status(now), flush=True)
                next_status = now + 1.0
    except KeyboardInterrupt:
        pass
    finally:
        board.esc, board.steer, board.cam_pan, board.front = [0.0, 0.0], 0.0, None, 0.0
        board.publish()
        if args.pty.is_symlink():
            args.pty.unlink()
    return 0


if __name__ == "__main__":
    sys.exit(main())

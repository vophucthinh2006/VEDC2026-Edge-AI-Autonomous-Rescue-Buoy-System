#!/usr/bin/env python3
"""The STM32F407 controller of the STM32 boat in the simulator (Rasp_Pi/main.py drives it).

Stands where the F407 sits on the boat: behind a pseudo-terminal that Rasp_Pi/modules/uart_comm.py
opens as serial.stm32_port, speaking the same lines as STM32F4/Modules/PiLink/uart_protocol.c and
STM32F4/Core/Src/main.c ($NAV $HBT $CAM $TXD in; $IMU $GPS $SYS $ACK out). What the board would
read from its BNO055 and GPS comes from the Gazebo buoy's odometry; what it would put on its ESC
and servo pins goes to the buoy's thrusters, pods and camera joint.

The heading PID, the motor/rudder mix and the camera glide are the firmware's own C
(STM32F4/App/heading_controller.c, camera_pan.c), compiled for this machine and called through
ctypes; the AUTO branch of App/control.c (Control_Tick) and the ESC ramp of
Modules/Actuators/actuators.c are copied below line for line. The RC transmitter is not simulated:
the board reports armed with the mode switch on AUTO, as on the water once the driver hands over.

    python3 f407_board_sim.py --pty /tmp/vedc_f407 --lib ./libf407.so --log f407.csv
"""
from __future__ import annotations

import argparse
import ctypes
import math
import os
import select
import sys
import threading
import time
import tty
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Rasp_Pi"))

from gz.msgs10.double_pb2 import Double  # noqa: E402
from gz.msgs10.odometry_pb2 import Odometry  # noqa: E402
from gz.transport13 import Node  # noqa: E402

from utils.nmea_packet import decode, encode  # noqa: E402

# STM32F4/App/app_config.h
CONTROL_PERIOD_S = 0.010
TELEMETRY_PERIOD_S = 0.100
HBT_TIMEOUT_S = 0.500
NAV_TIMEOUT_S = 0.300
ESC_RAMP_PER_S = 2.0
CAM_PAN_LIMIT_DEG = 80.0
RUDDER_DIR_FRONT, RUDDER_DIR_REAR = -1.0, 1.0
# Gazebo side (sim/models/vedc_buoy/model.sdf): a rear ESC at full is the 2000 us the ArduPilot link
# turns into 15 N; a pod at +-RUDDER_SPAN_US (250 us of the 1000 us servo range over +-45 deg) is
# +-0.39 rad. Positive steer turns the bow right, as ArduPilot's GroundSteering does on these joints.
REAR_THRUST_N = 15.0
POD_RAD_PER_STEER = 0.25 * math.pi / 2
# Origin of the Gazebo world in GPS terms: run_boat.sh's HOME_LOCATION, the one
# Rasp_Pi/config/sim_central.yaml and the other scenes give their people in.
HOME_LAT, HOME_LON = 10.883438, 106.796019
EARTH_M_PER_DEG = 111320.0
TOPIC = "/model/vedc_buoy/joint/{}/{}"


class HeadingControllerT(ctypes.Structure):
    _fields_ = [("integral", ctypes.c_float), ("error_deg", ctypes.c_float), ("rate_dps", ctypes.c_float),
                ("rate_primed", ctypes.c_bool), ("p", ctypes.c_float), ("i", ctypes.c_float), ("d", ctypes.c_float)]


class CameraPanT(ctypes.Structure):
    _fields_ = [("angle_deg", ctypes.c_float)]


def load_firmware(path: Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(path))
    f, pf = ctypes.c_float, ctypes.POINTER(ctypes.c_float)
    lib.HeadingController_Update.restype = f
    lib.HeadingController_Update.argtypes = [ctypes.POINTER(HeadingControllerT), f, f, f, f]
    lib.HeadingController_Reset.argtypes = [ctypes.POINTER(HeadingControllerT)]
    lib.Heading_Mix.argtypes = [f, f, pf, pf, pf]
    lib.Heading_Wrap360.restype = f
    lib.Heading_Wrap360.argtypes = [f]
    lib.CameraPan_Step.restype = f
    lib.CameraPan_Step.argtypes = [ctypes.POINTER(CameraPanT), f, f]
    return lib


class Pose:
    """The buoy as the board's sensors would see it, from Gazebo odometry."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.x = self.y = 0.0
        self.heading_deg = 90.0      # compass: 0 north, clockwise; Gazebo x is east
        self.yaw_rate_dps = 0.0      # positive clockwise, like the heading
        self.pitch_deg = self.roll_deg = 0.0
        self.speed_mps = 0.0
        self.course_deg = 0.0
        self.stamp = 0.0

    def on_odometry(self, msg: Odometry) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        vx, vy = msg.twist.linear.x, msg.twist.linear.y      # body frame: forward, left
        with self.lock:
            self.x, self.y = msg.pose.position.x, msg.pose.position.y
            self.heading_deg = (90.0 - math.degrees(yaw)) % 360.0
            self.yaw_rate_dps = -math.degrees(msg.twist.angular.z)
            self.pitch_deg, self.roll_deg = math.degrees(pitch), math.degrees(roll)
            self.speed_mps = math.hypot(vx, vy)
            self.course_deg = (self.heading_deg - math.degrees(math.atan2(vy, vx))) % 360.0
            self.stamp = time.monotonic()

    def latlon(self) -> tuple[float, float]:
        return (HOME_LAT + self.y / EARTH_M_PER_DEG,
                HOME_LON + self.x / (EARTH_M_PER_DEG * math.cos(math.radians(HOME_LAT))))


class PiCommand:
    """pi_command_t of uart_protocol.h, filled the way handle_packet() fills it."""

    def __init__(self) -> None:
        self.mode = "STOP"
        self.speed_mps = self.heading_deg = 0.0
        self.ttl_s = 0.0
        self.sequence = 0
        self.last_nav = self.last_hbt = self.last_cam = 0.0
        self.cam_pan_deg = 0.0
        self.victims = 0

    def handle(self, line: bytes, now: float, reply) -> str | None:
        packet = decode(line)
        if packet is None or len(packet.fields) < 1:
            return None
        f = packet.fields
        if packet.command == "NAV" and len(f) >= 5:
            seq = int(f[0])
            live = self.last_nav and now - self.last_nav <= NAV_TIMEOUT_S
            if live and ((seq - self.sequence) & 0xFFFF) >= 0x8000:
                return None
            speed, heading, ttl = float(f[2]), float(f[3]), int(f[4])
            if not (0.0 <= speed <= 2.0 and 0.0 <= heading < 360.0 and 0 < ttl <= 1000):
                return None
            self.mode = f[1] if f[1] in ("AUTO", "MANUAL") else "STOP"
            self.speed_mps, self.heading_deg, self.ttl_s, self.sequence, self.last_nav = speed, heading, ttl / 1000.0, seq, now
        elif packet.command == "HBT" and len(f) >= 2:
            self.last_hbt = now
        elif packet.command == "CAM" and len(f) >= 2:
            pan = float(f[1])
            if -90.0 <= pan <= 90.0:
                self.cam_pan_deg, self.last_cam = pan, now
        elif packet.command == "TXD" and len(f) >= 5:
            if f[1] == "VICTIM_FOUND":
                self.victims += 1
            reply("ACK", f[0], "TXD", "ACCEPTED")
            return "TXD " + ",".join(f[1:])
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pty", required=True, type=Path, help="path the Pi opens as serial.stm32_port (a link to the pseudo-terminal)")
    parser.add_argument("--lib", required=True, type=Path, help="heading_controller.c + camera_pan.c built as a shared library")
    parser.add_argument("--log", type=Path, help="CSV of the boat's state, ten lines a second (t: Unix time)")
    parser.add_argument("--odometry", default="/model/vedc_buoy/odometry")
    args = parser.parse_args()

    fw = load_firmware(args.lib)
    master, slave = os.openpty()
    tty.setraw(slave)
    if args.pty.is_symlink() or args.pty.exists():
        args.pty.unlink()
    args.pty.symlink_to(os.ttyname(slave))
    os.set_blocking(master, False)

    pose = Pose()
    node = Node()
    if not node.subscribe(Odometry, args.odometry, pose.on_odometry):
        print(f"cannot subscribe to {args.odometry}", file=sys.stderr)
        return 1
    pubs = {name: node.advertise(TOPIC.format(joint, kind), Double) for name, joint, kind in (
        ("left", "prop_rear_left_joint", "cmd_thrust"), ("right", "prop_rear_right_joint", "cmd_thrust"),
        ("front", "prop_front_joint", "cmd_thrust"), ("pod_left", "pod_rear_left_joint", "cmd_pos"),
        ("pod_right", "pod_rear_right_joint", "cmd_pos"), ("pod_front", "pod_front_joint", "cmd_pos"),
        ("camera", "camera_pan_joint", "cmd_pos"))}

    def publish(name: str, value: float) -> None:
        msg = Double()
        msg.data = value
        pubs[name].publish(msg)

    def reply(command: str, *fields: object) -> None:
        try:
            os.write(master, encode(command, *fields))
        except (BlockingIOError, OSError):
            pass                     # nobody reads the port: lost, as on a real UART

    log = args.log.open("w", buffering=1) if args.log else None
    if log:
        log.write("t,x,y,heading,speed,mode,nav_speed,nav_heading,rear_left,rear_right,steer,cam_deg,victims\n")
    pi = PiCommand()
    heading = HeadingControllerT()
    fw.HeadingController_Reset(ctypes.byref(heading))
    pan = CameraPanT(0.0)
    heading_auto = False
    rear = [0.0, 0.0]
    steer = 0.0
    pending = b""
    started = last_tick = last_telemetry = last_log = time.monotonic()
    telemetry_seq = 0
    print(f"STM32F407 on {args.pty} -> {os.ttyname(slave)}", flush=True)
    try:
        while True:
            readable, _, _ = select.select([master], [], [], 0.002)
            now = time.monotonic()
            if readable:
                try:
                    pending += os.read(master, 4096)
                except (BlockingIOError, OSError):
                    pass
                *lines, pending = pending.split(b"\n")
                for line in lines:
                    event = pi.handle(line, now, reply)
                    if event:
                        print(f"{now - started:7.1f} {event}", flush=True)
            if now - last_tick < CONTROL_PERIOD_S:
                continue
            dt, last_tick = min(now - last_tick, 0.1), now
            with pose.lock:
                yaw, rate, x, y = pose.heading_deg, pose.yaw_rate_dps, pose.x, pose.y
                imu_fresh = now - pose.stamp < 0.2

            # control.c, Control_Tick, AUTO branch (armed, mode switch on AUTO).
            nav_fresh = pi.mode == "AUTO" and pi.ttl_s > 0 and now - pi.last_nav <= pi.ttl_s
            hbt_fresh = pi.last_hbt and now - pi.last_hbt <= HBT_TIMEOUT_S
            target = (0.0, 0.0, 0.0)            # rear_left, rear_right, steer
            if not imu_fresh or not nav_fresh or not hbt_fresh:
                fw.HeadingController_Reset(ctypes.byref(heading))
                heading_auto = False
                rear = [0.0, 0.0]               # Actuators_Stop: no ramp down
            else:
                if not heading_auto:
                    fw.HeadingController_Reset(ctypes.byref(heading))
                    heading_auto = True
                throttle = max(0.0, min(pi.speed_mps / 1.0, 0.50))
                if throttle <= 0.0:
                    fw.HeadingController_Reset(ctypes.byref(heading))
                else:
                    u = fw.HeadingController_Update(ctypes.byref(heading), fw.Heading_Wrap360(pi.heading_deg), yaw, rate, dt)
                    left, right, mixed = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
                    fw.Heading_Mix(u, throttle, ctypes.byref(left), ctypes.byref(right), ctypes.byref(mixed))
                    target = (left.value, right.value, mixed.value)
                # actuators.c: speed-ups ramp at ESC_RAMP_PER_S, cuts are immediate.
                step = ESC_RAMP_PER_S * dt
                rear = [min(out + step, goal) if goal > out else goal for out, goal in zip(rear, target[:2])]
            steer = target[2]
            publish("left", rear[0] * REAR_THRUST_N)
            publish("right", rear[1] * REAR_THRUST_N)
            publish("front", 0.0)
            publish("pod_left", RUDDER_DIR_REAR * steer * POD_RAD_PER_STEER)
            publish("pod_right", RUDDER_DIR_REAR * steer * POD_RAD_PER_STEER)
            publish("pod_front", RUDDER_DIR_FRONT * steer * POD_RAD_PER_STEER)
            # control.c, Camera_Tick: glide to the Pi's CAM angle; independent of arming.
            if pi.last_cam:
                angle = fw.CameraPan_Step(ctypes.byref(pan), max(-CAM_PAN_LIMIT_DEG, min(CAM_PAN_LIMIT_DEG, pi.cam_pan_deg)), dt)
                publish("camera", math.radians(angle))

            if now - last_telemetry >= TELEMETRY_PERIOD_S:
                last_telemetry = now
                telemetry_seq = (telemetry_seq + 1) & 0xFFFF
                with pose.lock:
                    lat, lon = pose.latlon()
                    pitch, roll, speed, course = pose.pitch_deg, pose.roll_deg, pose.speed_mps, pose.course_deg
                overturned = abs(pitch) > 35.0 or abs(roll) > 35.0
                reply("IMU", telemetry_seq, f"{pitch:.2f}", f"{roll:.2f}", f"{yaw:.2f}", int(imu_fresh), int(overturned))
                reply("GPS", telemetry_seq, f"{lat:.7f}", f"{lon:.7f}", 3, "0.8", f"{speed:.2f}", f"{course:.1f}")
                reply("SYS", telemetry_seq, "0.0", 0, 0, int(bool(hbt_fresh)), 1, 1, 0)
            if log and now - last_log >= 0.1:
                last_log = now
                log.write(f"{time.time():.2f},{x:.3f},{y:.3f},{yaw:.1f},{pose.speed_mps:.2f},{pi.mode if nav_fresh else 'STOP'},"
                          f"{pi.speed_mps:.2f},{pi.heading_deg:.1f},{rear[0]:.3f},{rear[1]:.3f},{steer:.3f},{pan.angle_deg:.1f},{pi.victims}\n")
    except KeyboardInterrupt:
        pass
    finally:
        for name in ("left", "right", "front"):
            publish(name, 0.0)
        if args.pty.is_symlink():
            args.pty.unlink()
    return 0


if __name__ == "__main__":
    sys.exit(main())

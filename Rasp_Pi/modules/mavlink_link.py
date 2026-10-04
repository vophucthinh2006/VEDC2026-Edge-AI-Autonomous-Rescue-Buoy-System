"""The sole owner of the Pi<->Pixhawk MAVLink link (TELEM2 on the boat, UDP in the simulator).

Feeds the StateStore from the autopilot's telemetry and offers the few commands the rescue
logic needs. ArduPilot Rover keeps steering and all failsafes; the Pi only switches modes and,
in GUIDED, says where to go. A GUIDED heading/speed target times out after 3 s in the
autopilot, so a hung Pi stops the boat by itself.
"""
from __future__ import annotations

import logging
import math
import threading
from time import monotonic

from pymavlink import mavutil

from modules.state_store import StateStore
from utils.geometry import bearing_deg, haversine_m

M = mavutil.mavlink
# The Pi shares the vehicle's system id and announces itself as its onboard computer.
ONBOARD_COMPONENT = M.MAV_COMP_ID_ONBOARD_COMPUTER
# (message id, Hz) asked from the autopilot once it is heard. TELEM ports send nothing by default.
STREAMS = ((M.MAVLINK_MSG_ID_ATTITUDE, 10), (M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5), (M.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
           (M.MAVLINK_MSG_ID_SYS_STATUS, 1), (M.MAVLINK_MSG_ID_MISSION_CURRENT, 1))
WAYPOINT_REFRESH_S = 5.0   # the current mission item is read again this often: the operator may have changed the mission


class MavlinkLink(threading.Thread):
    def __init__(self, url: str, baud: int, state: StateStore, stop_event: threading.Event, vehicle: dict) -> None:
        super().__init__(name="mavlink", daemon=True)
        self._url, self._baud, self._state, self._stop_event, self._v = url, baud, state, stop_event, vehicle
        self._master = None
        self._tx = threading.Lock()          # pymavlink is not thread-safe on the send side
        self._mode_numbers: dict[str, int] = {}
        self._gps = {"fix": 0, "hdop": 99.0}
        self._mission_seq = 0
        self._mission_total = 0
        self._waypoint: tuple[int, float, float] | None = None   # mission item, lat, lon; None: not a waypoint
        self._waypoint_asked = 0.0
        self._wp_dist_m = -1.0
        self._wp_bearing_deg = 0.0
        self.log = logging.getLogger(__name__)
        self.ready = threading.Event()       # set once the autopilot's heartbeat is heard

    # ---- telemetry in -------------------------------------------------------------------
    def _on_message(self, msg) -> None:
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            if msg.autopilot == M.MAV_AUTOPILOT_INVALID or msg.get_srcComponent() != M.MAV_COMP_ID_AUTOPILOT1:
                return   # MAVProxy, a ground station or ourselves
            first = not self.ready.is_set()
            if first:
                self._master.target_system, self._master.target_component = msg.get_srcSystem(), msg.get_srcComponent()
                self._master.mav.srcSystem = msg.get_srcSystem()
                self._mode_numbers = {name.upper(): number for name, number in (self._master.mode_mapping() or {}).items()}
            armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            self._state.update_vehicle(mavutil.mode_string_v10(msg).upper(), armed, self._mission_seq, self._mission_total,
                                       self._wp_dist_m, self._wp_bearing_deg)
            if first:
                self.ready.set()
                self._request_streams()
                self.log.info("autopilot heard: system %d, mode %s", msg.get_srcSystem(), mavutil.mode_string_v10(msg))
        elif kind == "ATTITUDE":
            pitch, roll = math.degrees(msg.pitch), math.degrees(msg.roll)
            overturned = abs(pitch) > self._v["max_pitch_deg"] or abs(roll) > self._v["max_roll_deg"]
            self._state.update_imu(pitch, roll, math.degrees(msg.yaw), True, overturned)
        elif kind == "GPS_RAW_INT":
            self._gps = {"fix": int(msg.fix_type), "hdop": msg.eph / 100.0 if msg.eph != 0xFFFF else 99.0}
        elif kind == "GLOBAL_POSITION_INT":
            speed = math.hypot(msg.vx, msg.vy) / 100.0
            course = msg.hdg / 100.0 if msg.hdg != 0xFFFF else 0.0
            self._state.update_gps(msg.lat / 1e7, msg.lon / 1e7, self._gps["fix"], self._gps["hdop"], speed, course)
            # The autopilot's own wp_dist (NAV_CONTROLLER_OUTPUT) is to the avoidance target while it
            # goes around something, so the distance to the mission waypoint is worked out here.
            waypoint = self._waypoint
            if waypoint and waypoint[0] == self._mission_seq:
                self._wp_dist_m = haversine_m(msg.lat / 1e7, msg.lon / 1e7, waypoint[1], waypoint[2])
                self._wp_bearing_deg = bearing_deg(msg.lat / 1e7, msg.lon / 1e7, waypoint[1], waypoint[2])
            else:
                self._wp_dist_m = -1.0
        elif kind == "SYS_STATUS":
            volts = msg.voltage_battery / 1000.0 if msg.voltage_battery != 0xFFFF else 0.0
            self._state.update_system(volts, False, False, True)
        elif kind == "MISSION_CURRENT":
            self._mission_seq = int(msg.seq)
            total = int(getattr(msg, "total", 0))
            self._mission_total = 0 if total == 0xFFFF else total
            now = monotonic()
            if (self._waypoint is None or self._waypoint[0] != self._mission_seq) or now - self._waypoint_asked >= WAYPOINT_REFRESH_S:
                self._waypoint_asked = now
                with self._tx:
                    self._master.mav.mission_request_int_send(self._master.target_system, self._master.target_component, self._mission_seq)
        elif kind == "MISSION_ITEM_INT":
            is_waypoint = msg.command == M.MAV_CMD_NAV_WAYPOINT and (msg.x or msg.y)
            if msg.seq == self._mission_seq:
                self._waypoint = (int(msg.seq), msg.x / 1e7, msg.y / 1e7) if is_waypoint else None

    def _request_streams(self) -> None:
        for message_id, hz in STREAMS:
            self._command(M.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, 1e6 / hz)

    def run(self) -> None:
        try:
            self._master = mavutil.mavlink_connection(self._url, baud=self._baud, source_system=1,
                                                      source_component=ONBOARD_COMPONENT)
        except Exception as exc:  # noqa: BLE001 - serial and socket errors differ per transport
            self.log.error("MAVLink link unavailable (%s): %s", self._url, exc)
            self._stop_event.set()
            return
        self.log.info("MAVLink open: %s", self._url)
        last_heartbeat = 0.0
        while not self._stop_event.is_set():
            msg = self._master.recv_match(blocking=True, timeout=0.2)
            if msg is not None and msg.get_type() != "BAD_DATA":
                self._on_message(msg)
            now = monotonic()
            if now - last_heartbeat >= 1.0:
                last_heartbeat = now
                with self._tx:
                    self._master.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    # ---- commands out -------------------------------------------------------------------
    def _command(self, command: int, *params: float) -> None:
        padded = list(params) + [0.0] * (7 - len(params))
        with self._tx:
            self._master.mav.command_long_send(self._master.target_system, self._master.target_component, command, 0, *padded)

    def set_mode(self, name: str) -> bool:
        number = self._mode_numbers.get(name.upper())
        if number is None:
            self.log.error("unknown mode %s", name)
            return False
        self._command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number)
        return True

    def set_mission_current(self, seq: int) -> None:
        """Jump the mission to an item; the autopilot carries on from there."""
        self._command(M.MAV_CMD_DO_SET_MISSION_CURRENT, seq)

    def goto(self, lat: float, lon: float) -> None:
        """GUIDED position target. The autopilot navigates there with its own obstacle avoidance."""
        with self._tx:
            self._master.mav.set_position_target_global_int_send(
                0, self._master.target_system, self._master.target_component, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b110111111000, int(lat * 1e7), int(lon * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def heading_speed(self, heading_deg: float, speed_mps: float) -> None:
        """GUIDED heading and speed. Must be repeated: the autopilot drops it after 3 s."""
        self._command(M.MAV_CMD_NAV_SET_YAW_SPEED, heading_deg % 360.0, speed_mps, 0)

    def statustext(self, text: str, severity: int = M.MAV_SEVERITY_NOTICE) -> None:
        """Shown on every ground station that hears this link."""
        with self._tx:
            self._master.mav.statustext_send(severity, text[:50].encode("ascii", "replace"))

    def obstacle_distance(self, distances_cm: list[int], min_cm: int, max_cm: int) -> None:
        """72 sectors of 5 deg, clockwise from the bow, for the autopilot's avoidance (PRX1_TYPE = 2)."""
        with self._tx:
            self._master.mav.obstacle_distance_send(0, M.MAV_DISTANCE_SENSOR_LASER, distances_cm, 5, min_cm, max_cm,
                                                    0.0, 0.0, M.MAV_FRAME_BODY_FRD)

    def set_servo(self, output: int, pwm_us: int) -> None:
        self._command(M.MAV_CMD_DO_SET_SERVO, output, pwm_us)

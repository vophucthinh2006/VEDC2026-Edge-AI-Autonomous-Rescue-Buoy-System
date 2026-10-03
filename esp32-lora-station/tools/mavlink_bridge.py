#!/usr/bin/env python3
"""Forward an ArduPilot vehicle's MAVLink telemetry to the shore-station cloud.

Acts like the ESP32 station, but reads MAVLink instead of LoRa: the simulator
(sim/run_boat.sh, MAVProxy output 14551) today, a Pixhawk on a telemetry radio
later. Posts the same JSON as the ESP32 to /api/ingest, with "link":"mavlink",
no RSSI/SNR, and a "nav" group (armed, mode, speed, throttle, waypoint, battery,
satellites) for the dashboard HUD.

    export VEDC_INGEST_TOKEN=...        # same secret as the ESP32's CLOUD_INGEST_TOKEN
    python3 mavlink_bridge.py --url http://127.0.0.1:8787/api/ingest
    python3 mavlink_bridge.py --url https://<worker>.workers.dev/api/ingest --id PHAO-01

Needs pymavlink (in ~/venv-ardupilot after sim/install_deps.sh).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

from pymavlink import mavutil

AUTO_MODES = {"AUTO", "GUIDED", "RTL", "SMART_RTL", "LOITER", "FOLLOW", "DOCK", "CIRCLE"}
STOP_MODES = {"HOLD", "INITIALISING"}
AHRS_BIT = mavutil.mavlink.MAV_SYS_STATUS_AHRS
EKF_ATTITUDE = mavutil.mavlink.EKF_ATTITUDE
EKF_POS_ABS = mavutil.mavlink.EKF_POS_HORIZ_ABS
STALE_S = 3.0


class Vehicle:
    """Latest value of each MAVLink message we use, with its arrival time."""

    def __init__(self) -> None:
        self.msgs: dict[str, tuple[float, object]] = {}
        self.system: int | None = None

    def update(self, msg) -> None:
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            # MAVProxy and ground stations heartbeat too; keep the autopilot's.
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                return
            if self.system is None:
                self.system = msg.get_srcSystem()
            elif msg.get_srcSystem() != self.system:
                return
        self.msgs[kind] = (time.monotonic(), msg)

    def get(self, kind: str):
        entry = self.msgs.get(kind)
        if entry is None or time.monotonic() - entry[0] > STALE_S:
            return None
        return entry[1]


def build_frame(v: Vehicle, buoy_id: str, seq: int) -> dict | None:
    hb, att = v.get("HEARTBEAT"), v.get("ATTITUDE")
    if hb is None or att is None:
        return None
    pos, gps, hud = v.get("GLOBAL_POSITION_INT"), v.get("GPS_RAW_INT"), v.get("VFR_HUD")
    nav_out, mission, sys_status, ekf = (v.get("NAV_CONTROLLER_OUTPUT"), v.get("MISSION_CURRENT"),
                                         v.get("SYS_STATUS"), v.get("EKF_STATUS_REPORT"))

    mode_name = mavutil.mode_string_v10(hb).upper()
    if not mode_name.replace("_", "").isalpha() or len(mode_name) > 16:
        mode_name = "UNKNOWN"
    armed = 1 if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else 0
    steering_by_heading = armed and mode_name in AUTO_MODES
    mode = "A" if steering_by_heading else ("S" if not armed or mode_name in STOP_MODES else "M")

    fix = 1 if gps is not None and gps.fix_type >= 3 and pos is not None else 0
    frame: dict = {"id": buoy_id, "link": "mavlink", "fix": fix}
    if fix:
        frame["lat"] = pos.lat / 1e7
        frame["lon"] = pos.lon / 1e7

    imu_ok = 0
    if sys_status is not None:
        healthy = sys_status.onboard_control_sensors_health & AHRS_BIT
        present = sys_status.onboard_control_sensors_present & AHRS_BIT
        imu_ok = 1 if present and healthy else 0
    # ArduPilot has no BNO055-style 0..3 calibration; report the EKF instead:
    # 3 when it has both attitude and an absolute position, else 0.
    calib = 3 if ekf is not None and ekf.flags & EKF_ATTITUDE and ekf.flags & EKF_POS_ABS else 0

    target_yaw = -1.0
    wp_dist = -1.0
    if steering_by_heading and nav_out is not None:
        target_yaw = round(nav_out.target_bearing % 360, 1) % 360
        wp_dist = float(nav_out.wp_dist)
    frame.update({
        "roll": round(max(-180.0, min(180.0, math.degrees(att.roll))), 1),
        "pitch": round(max(-90.0, min(90.0, math.degrees(att.pitch))), 1),
        "yaw": round(math.degrees(att.yaw) % 360, 1) % 360,
        "target_yaw": target_yaw,
        "mode": mode,
        "imu_ok": imu_ok,
        "calib": calib,
        "seq": seq,
    })

    batt_v, batt_pct = 0.0, -1
    if sys_status is not None:
        if sys_status.voltage_battery != 0xFFFF:
            batt_v = sys_status.voltage_battery / 1000.0
        batt_pct = sys_status.battery_remaining if sys_status.battery_remaining >= 0 else -1
    sats, hdop = 0, 99.0
    if gps is not None:
        sats = min(gps.satellites_visible, 99)
        hdop = min(gps.eph / 100.0, 99.0) if gps.eph != 0xFFFF else 99.0
    frame["nav"] = {
        "armed": armed,
        "mode_name": mode_name,
        "gs": round(min(max(hud.groundspeed, 0.0), 50.0), 2) if hud else 0.0,
        "thr": max(-100, min(100, int(hud.throttle))) if hud else 0,
        "wp_dist": wp_dist,
        "wp_seq": mission.seq if mission is not None else 0,
        "batt_v": round(min(batt_v, 100.0), 2),
        "batt_pct": batt_pct,
        "sats": sats,
        "hdop": round(hdop, 2),
    }
    frame["raw"] = (f"MAV {mode_name} {'ARMED' if armed else 'DISARMED'} "
                    f"gs={frame['nav']['gs']:.1f} hdg={frame['yaw']:.0f}")[:100]
    return frame


def post(url: str, token: str, frame: dict) -> int:
    body = json.dumps(frame).encode()
    req = urllib.request.Request(url, data=body, method="POST", headers={
        "Content-Type": "application/json", "Authorization": "Bearer " + token})
    try:
        with urllib.request.urlopen(req, timeout=4) as resp:
            return resp.status
    except urllib.error.HTTPError as err:
        return err.code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--link", default="udpin:0.0.0.0:14551", help="MAVLink connection (pymavlink syntax)")
    parser.add_argument("--url", required=True, help="cloud ingest URL, ending in /api/ingest")
    parser.add_argument("--id", default="SIM-01", help="name shown on the dashboard")
    parser.add_argument("--rate", type=float, default=2.0, help="posts per second")
    args = parser.parse_args()
    token = os.environ.get("VEDC_INGEST_TOKEN", "")
    if not token:
        print("Set VEDC_INGEST_TOKEN to the cloud's INGEST_TOKEN.", file=sys.stderr)
        return 2

    master = mavutil.mavlink_connection(args.link, source_system=253)
    vehicle = Vehicle()
    period = 1.0 / max(args.rate, 0.1)
    next_post = time.monotonic()
    seq = 0
    sent = failed = 0
    last_report = time.monotonic()
    print(f"listening on {args.link}, posting to {args.url} as {args.id} at {args.rate:g} Hz", flush=True)
    while True:
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is not None and msg.get_type() != "BAD_DATA":
            vehicle.update(msg)
        now = time.monotonic()
        if now < next_post:
            continue
        next_post = now + period
        frame = build_frame(vehicle, args.id, seq)
        if frame is None:
            continue
        seq = (seq + 1) & 0xFFFFFFFF
        try:
            status = post(args.url, token, frame)
        except (urllib.error.URLError, OSError) as err:
            status = 0
            failed += 1
            print(f"post failed: {err}", file=sys.stderr)
        else:
            if status == 200:
                sent += 1
            else:
                failed += 1
                print(f"cloud answered HTTP {status}", file=sys.stderr)
                if status == 401:
                    print("Wrong VEDC_INGEST_TOKEN; stopping.", file=sys.stderr)
                    return 1
        if now - last_report >= 10:
            last_report = now
            nav = frame["nav"]
            print(f"sent {sent}, failed {failed} | {nav['mode_name']} armed={nav['armed']} "
                  f"gs={nav['gs']} fix={frame['fix']}", flush=True)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)

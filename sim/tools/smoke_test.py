#!/usr/bin/env python3
"""Drive the simulated buoy 20 m east in GUIDED and check that it really moves.

Run while sim/run_boat.sh is up. 14550 belongs to Mission Planner, and
run_boat.sh also sends to 14551 (the dashboard bridge's port). With both of
those in use, give the test its own MAVProxy output at the MAV> prompt:
    output add 127.0.0.1:14552
    python3 smoke_test.py --link udpin:0.0.0.0:14552
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from pymavlink import mavutil


def wait_position(master, timeout: float = 5.0):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    if msg is None:
        raise RuntimeError("no GLOBAL_POSITION_INT")
    return msg.lat / 1e7, msg.lon / 1e7


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dn = (lat2 - lat1) * 111_320.0
    de = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--link", default="udpin:0.0.0.0:14550")
    parser.add_argument("--distance", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.link, source_system=250)
    # MAVProxy forwards GCS heartbeats too; lock onto the autopilot's own.
    deadline = time.time() + 30
    while True:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            master.target_system, master.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            break
        if time.time() > deadline:
            print("FAIL: no ArduPilot heartbeat")
            return 1
    print(f"heartbeat from system {master.target_system}, mode {master.flightmode}")

    # The EKF needs a GPS fix and a few seconds before it accepts arming.
    deadline = time.time() + 60
    while time.time() < deadline:
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
            break
        text = master.recv_match(type="STATUSTEXT", blocking=False)
        if text:
            print("  ", text.text)
        time.sleep(2)
    else:
        print("FAIL: could not arm within 60 s")
        return 1
    print("armed")

    master.set_mode("GUIDED")
    start_lat, start_lon = wait_position(master)
    goal_lat = start_lat
    goal_lon = start_lon + args.distance / (111_320.0 * math.cos(math.radians(start_lat)))
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,                 # use position only
        int(goal_lat * 1e7), int(goal_lon * 1e7), 0,
        0, 0, 0, 0, 0, 0, 0, 0)
    print(f"guided to {args.distance:.0f} m east")

    deadline = time.time() + args.timeout
    best = 0.0
    while time.time() < deadline:
        lat, lon = wait_position(master)
        moved = distance_m(start_lat, start_lon, lat, lon)
        to_go = distance_m(lat, lon, goal_lat, goal_lon)
        best = max(best, moved)
        print(f"  moved {moved:5.1f} m, to go {to_go:5.1f} m", end="\r")
        if to_go < 3.0:
            print(f"\nPASS: reached the goal, moved {moved:.1f} m")
            break
    else:
        print(f"\nFAIL: moved only {best:.1f} m in {args.timeout:.0f} s")

    master.set_mode("HOLD")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    return 0 if best >= args.distance - 3.0 else 1


if __name__ == "__main__":
    sys.exit(main())

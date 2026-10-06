#!/usr/bin/env python3
"""Experiment: the Pi steers the Pixhawk boat itself, to compare avoidance algorithms.

Normally the autopilot avoids obstacles (BendyRuler) and the Pi only tells it where they are
(rescue_main.py). Here the autopilot is put in GUIDED and follows the heading and speed that one
of the navigators of modules/navigation.py computes from the LiDAR, five times a second:

    bapf    the starboard-biased potential field of the STM32 runtime, with its settings.yaml values
    paper   the biased potential field of Jo et al. 2022 (papers/), with the CPA gate
    vo      the velocity obstacle of the same paper, for obstacles at rest

No person detection, no rescue logic. Run from this directory, under sim/env.sh, once the boat is
armed (sim/tools/avoid_compare.sh does all of it):

    python3 pi_steer.py --navigator vo --route "0,38.5;0,2" --overlay config/sim.yaml
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import signal
import threading
from pathlib import Path
from time import monotonic, sleep

from modules.mavlink_link import MavlinkLink
from modules.navigation import BapfNavigator, PaperBapfNavigator, VoNavigator
from modules.obstacle_feed import SECTOR_DEG, sectors
from modules.perception import Obstacle
from modules.state_store import Snapshot, StateStore
from rescue_main import load_config

HOME = (10.883438, 106.796019)            # sim/run_boat.sh HOME_LOCATION
LIDAR_MIN_M, LIDAR_MAX_M = 0.15, 8.0
CRUISE_MPS, ARRIVE_M = 1.0, 2.0           # WP_SPEED and WP_RADIUS of sim/params/vedc_buoy.parm
HULL_M = 0.55                             # bow to centre: the boat as a circle

# Jo et al. size their fields for a 4.88 m boat on a 25 m grid (a = 4, b = 2/64: safety boundary
# 32 m, 6.5 boat lengths; bias a = 3, b = 1/48, 2/3 of a length away; TCPA 20 s, DCPA 24 m).
# Scaled to a 1.1 m boat in a 12 m wide street the boundary would be 7 m, wider than the room on
# either side, so the fields are drawn in: boundary 4 m, bias 5 m, gate 8 s and 1.5 m.
#
# Tuning, each change after a run in the flood scene (sim/tools/avoid_compare.sh):
#   1. bias a 1.5 -> 0.6. With a bias nearly as strong as the main field the sideways push won
#      over everything and the boat span on the spot (28 000 deg of turning in 240 s).
#   2. VO radius 1.0 -> 1.3 m and switch 20 -> 30 deg: at 1.0 m the boat, which lags its commands,
#      came within 0.57 m of the car; BendyRuler, asked for 1 m, keeps 1.0 m.
#   3. Field drawn in further: boundary 4 -> 2.5 m (b 0.25 -> 0.4), a 2 -> 1.5, bias boundary
#      5 -> 3.3 m, DCPA gate 1.5 -> 1.2 m. With the 4 m boundary the car pushed the boat into the
#      alley between two houses across the street, where it stayed.
PAPER = {"a": 1.5, "b": 0.4, "bias_a": 0.6, "bias_b": 0.3, "bias_offset_m": 2.0 / 3.0 * 1.1, "att_max": 4.0,
         "control_max": 8.0, "tcpa_max_s": 8.0, "dcpa_min_m": 1.2}
# 6 s at 1 m/s is about BendyRuler's 5 m look-ahead.
VO = {"radius_m": 1.3, "tcpa_max_s": 6.0, "heading_step_deg": 5.0, "switch_deg": 30.0}
CLUSTER_JUMP_M = 0.8                      # neighbouring sectors further apart in range are two objects


def sector_obstacles(snap: Snapshot) -> tuple[Obstacle, ...]:
    """One obstacle per 5 deg sector with a LiDAR return, at its nearest range: what the autopilot
    is given too. Per ray, a wall would count as dozens of obstacles and its push would grow with
    the number of rays that happen to hit it."""
    rays = [(o.bearing_body_deg, o.distance_m, o.width_m / 2.0) for o in snap.obstacles]
    clear = int(LIDAR_MAX_M * 100) + 1
    return tuple(Obstacle(index * SECTOR_DEG if index * SECTOR_DEG < 180.0 else index * SECTOR_DEG - 360.0, cm / 100.0, 0.1)
                 for index, cm in enumerate(sectors(rays, LIDAR_MIN_M, LIDAR_MAX_M)) if cm < clear)


def clustered(obstacles: tuple[Obstacle, ...]) -> tuple[Obstacle, ...]:
    """Neighbouring sectors at a similar range are one object, kept as its nearest point. The
    potential fields add one push per obstacle, as the paper adds one per boat: left as sectors,
    a tree 4 m away would push five times and a wall a dozen."""
    by_bearing = sorted(obstacles, key=lambda o: o.bearing_body_deg)
    groups: list[list[Obstacle]] = []
    for obstacle in by_bearing:
        previous = groups[-1][-1] if groups else None
        if previous and obstacle.bearing_body_deg - previous.bearing_body_deg <= SECTOR_DEG + 0.1 \
                and abs(obstacle.distance_m - previous.distance_m) <= CLUSTER_JUMP_M:
            groups[-1].append(obstacle)
        else:
            groups.append([obstacle])
    if len(groups) > 1 and by_bearing[0].bearing_body_deg <= -180.0 + SECTOR_DEG / 2 and by_bearing[-1].bearing_body_deg >= 175.0 - 0.1 \
            and abs(by_bearing[0].distance_m - by_bearing[-1].distance_m) <= CLUSTER_JUMP_M:
        groups[0] = groups.pop() + groups[0]          # the same object across the stern
    return tuple(min(group, key=lambda o: o.distance_m) for group in groups)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--navigator", choices=("bapf", "paper", "vo"), required=True)
    parser.add_argument("--route", required=True, help='waypoints as "north,east;north,east" in metres from home')
    parser.add_argument("--config", type=Path, default=Path("config/settings.yaml"))
    parser.add_argument("--overlay", type=Path, action="append", default=[])
    parser.add_argument("--mavlink", help="overrides mavlink.url")
    parser.add_argument("--viewer", type=int, nargs="?", const=8090, metavar="PORT")
    args = parser.parse_args()
    config = load_config(args.config, args.overlay)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger("pi_steer")

    vehicle = {"boat_radius_m": HULL_M, "max_speed_mps": CRUISE_MPS, "arrive_radius_m": ARRIVE_M}
    if args.navigator == "bapf":
        # As the STM32 runtime runs it, but at this experiment's cruise speed and arrival radius.
        navigator = BapfNavigator({**config["vehicle"], "max_speed_mps": CRUISE_MPS, "arrive_radius_m": ARRIVE_M}, config["planner"])
    elif args.navigator == "paper":
        navigator = PaperBapfNavigator(vehicle, PAPER)
    else:
        navigator = VoNavigator(vehicle, VO)
    k = 111_320.0 * math.cos(math.radians(HOME[0]))
    route = [(HOME[0] + float(n) / 111_320.0, HOME[1] + float(e) / k) for n, e in (leg.split(",") for leg in args.route.split(";"))]

    stop_event = threading.Event()
    state = StateStore()
    link = MavlinkLink(args.mavlink or config["mavlink"]["url"], int(config["mavlink"]["baud"]), state, stop_event, config["vehicle"])
    from modules.gazebo_io import GazeboLidar   # this experiment only exists in the simulator
    lidar = GazeboLidar(config["lidar"]["gazebo_topic"], state, LIDAR_MIN_M, LIDAR_MAX_M)
    link.start()
    status = {"text": "waiting"}
    if args.viewer:
        from modules.viewer import Viewer
        Viewer(args.viewer, state, stop_event, config["avoidance"], config["camera"], lambda: None, lambda: [], lambda: None,
               lambda: f"{args.navigator}: {status['text']}").start()
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

    while not link.ready.wait(timeout=0.5):
        if stop_event.is_set():
            return 1
    log.info("navigator %s, %d waypoints, lidar %s; waiting for the boat to be armed", args.navigator, len(route), "ok" if lidar.opened else "NOT subscribed")
    while not stop_event.is_set() and not (state.snapshot().vehicle.armed and state.snapshot().gps.fix >= 3):
        sleep(0.2)
    link.set_mode("GUIDED")
    leg, last_log, last_reason = 0, 0.0, ""
    try:
        while not stop_event.is_set() and leg < len(route):
            snap = state.snapshot()
            if snap.vehicle.mode != "GUIDED":
                link.set_mode("GUIDED")           # not there yet, or somebody took the boat: ask again
                sleep(0.2)
                continue
            obstacles = sector_obstacles(snap)
            if args.navigator != "vo":                 # the velocity obstacle wants every point, the fields one push per object
                obstacles = clustered(obstacles)
            snap = dataclasses.replace(snap, obstacles=obstacles, waypoint=route[leg], human_target=None)
            command = navigator.plan(snap)
            if command.reason == "arrived":
                leg += 1
                link.statustext(f"PI WP {leg} REACHED")
                log.info("waypoint %d reached", leg)
                continue
            link.heading_speed(command.heading_deg, command.speed_mps if command.mode != "STOP" else 0.0)
            status["text"] = f"wp {leg + 1} {command.reason}"
            now = monotonic()
            if command.reason != last_reason or now - last_log >= 5.0:
                last_reason, last_log = command.reason, now
                log.info("wp %d | %s | heading %.0f (bow %.0f) | speed %.2f | %d obstacles", leg + 1, command.reason,
                         command.heading_deg, snap.imu.yaw_deg, command.speed_mps, len(snap.obstacles))
            sleep(0.2)
        if leg >= len(route):
            link.set_mode("HOLD")
            link.statustext("ROUTE DONE")
            log.info("route done")
            sleep(1.0)
    finally:
        stop_event.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

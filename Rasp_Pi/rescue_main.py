#!/usr/bin/env python3
"""RedgeSCUE Pi runtime for the Pixhawk boat. Run from this directory: python3 rescue_main.py.

The autopilot (ArduPilot Rover) steers, navigates and keeps the failsafes. This program watches
the camera and, when a person is confirmed, takes the boat to them, holds, reports and hands the
boat back (modules/rescue.py). main.py is the older runtime for the STM32 controller.

Simulator: python3 rescue_main.py --overlay config/sim.yaml   (under sim/env.sh)
"""
from __future__ import annotations

import argparse
import logging
import signal
import threading
from pathlib import Path
from time import monotonic, sleep

import yaml

from modules.mavlink_link import MavlinkLink
from modules.obstacle_feed import ObstacleFeed
from modules.rescue import RescueMission
from modules.state_store import StateStore


def merge(base: dict, overlay: dict) -> dict:
    """Overlay values win; nested sections are merged key by key."""
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path: Path, overlays: list[Path]) -> dict:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    for overlay in overlays:
        merge(config, yaml.safe_load(overlay.read_text(encoding="utf-8")))
    return config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=Path("config/settings.yaml"))
    parser.add_argument("--overlay", type=Path, action="append", default=[],
                        help="YAML file whose values override --config (config/sim.yaml); may be repeated, later ones win")
    parser.add_argument("--detector", choices=("camera", "simulated"), help="overrides rescue.detector")
    parser.add_argument("--mavlink", help="overrides mavlink.url, e.g. udpin:0.0.0.0:14552")
    parser.add_argument("--no-scan", action="store_true", help="camera fixed ahead while searching (camera.scan.enabled off)")
    parser.add_argument("--scan-mode", choices=("step", "sweep"), help="overrides camera.scan.mode")
    parser.add_argument("--scan-rate", type=float, metavar="DEG_S", help="overrides camera.scan.sweep_rate_deg_s")
    parser.add_argument("--viewer", type=int, nargs="?", const=8090, metavar="PORT",
                        help="serve the live view of camera and LiDAR on http://<this machine>:PORT/ (default 8090)")
    args = parser.parse_args()
    config = load_config(args.config, args.overlay)
    if args.detector:
        config["rescue"]["detector"] = args.detector
    if args.mavlink:
        config["mavlink"]["url"] = args.mavlink
    if args.no_scan:
        config["camera"]["scan"]["enabled"] = False
    if args.scan_mode:
        config["camera"]["scan"]["mode"] = args.scan_mode
    if args.scan_rate:
        config["camera"]["scan"]["sweep_rate_deg_s"] = args.scan_rate
    logging.basicConfig(level=getattr(logging, config["logging"]["level"].upper()),
                        format="%(asctime)s %(levelname)s %(threadName)s %(name)s: %(message)s")
    log = logging.getLogger("rescue")

    stop_event = threading.Event()
    state = StateStore()
    link = MavlinkLink(config["mavlink"]["url"], int(config["mavlink"]["baud"]), state, stop_event, config["vehicle"])
    mission = RescueMission(config["vehicle"], config["rescue"])
    workers: list[threading.Thread] = [link]

    if config["rescue"]["detector"] == "simulated":
        from modules.detector_sim import SimulatedDetector
        detector = SimulatedDetector(config["camera"], config["sim"], state, stop_event)
        workers.append(detector)
    else:
        from modules.ai_vision import VisionWorker
        camera = config["camera"]
        if camera.get("source") == "gazebo":
            from modules.gazebo_io import GazeboPan
            send_pan = GazeboPan(camera["pan_topic"])
        else:
            # The pan servo hangs off the STM32F103 helper board, which is not wired yet.
            def send_pan(_pan_deg: float) -> None:
                return
        # The rescue logic reports the person itself, with a position; the worker's own callback is unused.
        detector = VisionWorker(camera, state, stop_event, lambda _target: None, send_pan,
                                float(config["vehicle"]["target_standoff_m"]))
        workers.append(detector)
    avoidance = config["avoidance"]
    # The serial LiDAR (LDS-008) is not wired into this runtime yet: on the boat only the keep-outs are sent.
    lidar = None
    if config.get("lidar", {}).get("source") == "gazebo":
        from modules.gazebo_io import GazeboLidar
        lidar = GazeboLidar(config["lidar"]["gazebo_topic"], state, float(avoidance["min_range_m"]), float(avoidance["max_range_m"]))
    # LiDAR returns and keep-out circles around people already attended go to the autopilot, which
    # bends the path around them and returns to it. Started once the link is up.
    obstacle_feed = ObstacleFeed(avoidance, state, stop_event, mission.keep_outs, mission.attending, link.obstacle_distance)
    if args.viewer:
        from modules.viewer import Viewer   # needs OpenCV; only loaded when asked for
        workers.append(Viewer(args.viewer, state, stop_event, avoidance, config["camera"], detector.view,
                              mission.keep_outs, mission.attending, lambda: mission.phase.value))
    for worker in workers:
        worker.start()

    def request_stop(*_unused: object) -> None:
        stop_event.set()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    log.info("waiting for the autopilot on %s", config["mavlink"]["url"])
    while not link.ready.wait(timeout=0.5):
        if stop_event.is_set():
            return 1
    obstacle_feed.start()
    workers.append(obstacle_feed)
    log.info("rescue logic running, detector: %s, lidar: %s", config["rescue"]["detector"],
             "none" if lidar is None else "gazebo" if lidar.opened else "gazebo (cannot subscribe)")
    last_phase = mission.phase
    last_status = 0.0
    try:
        while not stop_event.is_set():
            snapshot, now = state.snapshot(), monotonic()
            if now - last_status >= 5.0:
                last_status = now
                target = snapshot.human_target
                nearest = min((o.distance_m for o in snapshot.obstacles), default=None)
                log.info("%s %s | fix %d | yaw %.0f | person %s | nearest obstacle %s | phase %s", snapshot.vehicle.mode or "?",
                         "armed" if snapshot.vehicle.armed else "disarmed", snapshot.gps.fix, snapshot.imu.yaw_deg,
                         f"{target.bearing_body_deg:+.0f} deg {target.distance_m:.1f} m" if target else "none",
                         f"{nearest:.1f} m" if nearest is not None else "none", mission.phase.value)
            for action in mission.step(snapshot, now):
                if action.kind == "mode":
                    link.set_mode(action.mode)
                elif action.kind == "goto":
                    link.goto(action.lat, action.lon)
                elif action.kind == "skip":
                    link.set_mission_current(action.seq)
                elif action.kind == "report":
                    text = action.text if not action.lat else f"{action.text} {action.lat:.6f} {action.lon:.6f} c={action.confidence:.2f}"
                    link.statustext(text)
                    log.warning(text)
            if mission.phase is not last_phase:
                log.info("phase %s -> %s", last_phase.value, mission.phase.value)
                last_phase = mission.phase
            sleep(0.2)
    finally:
        stop_event.set()
        for worker in workers:
            worker.join(timeout=1.0)
        log.info("stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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

from modules.mavlink_link import MavlinkLink
from modules.obstacle_feed import ObstacleFeed
from modules.rescue import RescueMission
from modules.state_store import StateStore
from utils.config import load_config


VICTIMS_NAME = "VICTIMS"   # NAMED_VALUE_INT: people found so far, for the shore bridge


def pan_through_autopilot(link: MavlinkLink, camera: dict):
    """Camera servo on one of the autopilot's outputs (camera.pan_output), driven with DO_SET_SERVO.
    At most ten commands a second: the search sweep asks for a new angle on every camera frame."""
    output, centre = int(camera["pan_output"]), float(camera.get("pan_center_us", 1500))
    us_per_deg, limit = float(camera.get("pan_us_per_deg", 5.5)) * float(camera.get("pan_dir", 1)), float(camera["pan_limit_deg"])
    last = [0.0]

    def send_pan(pan_deg: float) -> None:
        now = monotonic()
        if not link.ready.is_set() or now - last[0] < 0.1:
            return
        last[0] = now
        link.set_servo(output, round(centre + us_per_deg * max(-limit, min(limit, pan_deg))))
    return send_pan


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
    parser.add_argument("--log", type=Path, nargs="?", const=Path("logs"), metavar="DIR",
                        help="write one CSV line a second of the run to DIR/run_<date>_<time>.csv (default logs/)")
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
    # The STM32F103 helper board (camera servo, alarm relays, E-stop, LoRa beacon), if one is fitted.
    helper = None
    helper_port = str(config.get("helper", {}).get("port", "none"))
    if helper_port.lower() != "none":
        from modules.helper_board import HelperBoard
        helper = HelperBoard(helper_port, int(config["helper"]["baud"]), stop_event)
        workers.append(helper)

    if config["rescue"]["detector"] == "simulated":
        from modules.detector_sim import SimulatedDetector
        detector = SimulatedDetector(config["camera"], config["sim"], state, stop_event)
        workers.append(detector)
    else:
        from modules.ai_vision import VisionWorker
        camera = config["camera"]
        # The servo is on the helper board. In the simulator the Pi turns the Gazebo camera itself,
        # unless a simulated board is there to do it (camera.pan_via: helper, config/sim_hw.yaml).
        if helper is not None and (camera.get("source") != "gazebo" or camera.get("pan_via") == "helper"):
            send_pan = helper.send_pan
        elif camera.get("source") == "gazebo":
            from modules.gazebo_io import GazeboPan
            send_pan = GazeboPan(camera["pan_topic"])
        elif int(camera.get("pan_output", 0)) > 0:
            send_pan = pan_through_autopilot(link, camera)
        else:
            def send_pan(_pan_deg: float) -> None:       # no helper board, camera.pan_output 0: camera fixed ahead
                return
        # The rescue logic reports the person itself, with a position; the worker's own callback is unused.
        detector = VisionWorker(camera, state, stop_event, lambda _target: None, send_pan,
                                float(config["vehicle"]["target_standoff_m"]))
        workers.append(detector)
    avoidance = config["avoidance"]
    # lidar.source none: only the keep-outs are sent, the autopilot avoids nothing else.
    lidar_source = str(config.get("lidar", {}).get("source", "none")).lower()
    min_m, max_m = float(avoidance["min_range_m"]), float(avoidance["max_range_m"])
    if lidar_source == "gazebo":
        from modules.gazebo_io import GazeboLidar
        lidar = GazeboLidar(config["lidar"]["gazebo_topic"], state, min_m, max_m)
        lidar_source = "gazebo" if lidar.opened else "gazebo (cannot subscribe)"
    elif lidar_source == "serial":
        from modules.lidar_serial import MODELS, SerialLidar
        from modules.perception import ray_obstacles
        port, model = config["serial"], str(config["serial"].get("lidar_model", "lds008")).lower()
        if model not in MODELS:
            parser.error(f"serial.lidar_model: {model} is not one of {', '.join(MODELS)}")
        # One obstacle per ray, as from the simulator's LiDAR: a wall keeps its shape for the autopilot.
        workers.append(SerialLidar(model, port["lidar_port"], int(port["lidar_baud"]), config["vehicle"], state, stop_event,
                                   lambda scan: ray_obstacles([(p.angle_deg, p.distance_m) for p in scan if p.intensity > 0], min_m, max_m),
                                   required=False))
        lidar_source = f"serial {model} on {port['lidar_port']}"
    elif lidar_source != "none":
        parser.error(f"lidar.source: unknown source {lidar_source}")
    # LiDAR returns and keep-out circles around people already attended go to the autopilot, which
    # bends the path around them and returns to it. Started once the link is up.
    # With a LiDAR, a scan older than lidar_stale_s stops the feed: the autopilot then reports its
    # proximity sensor as failed instead of taking silence for open water.
    obstacle_feed = ObstacleFeed(avoidance, state, stop_event, mission.keep_outs, mission.attending, link.obstacle_distance,
                                 None if lidar_source == "none" else float(avoidance.get("lidar_stale_s", 1.0)))
    decision: dict = {}       # what the viewer's map shows of the rescue logic, replaced whole every step
    if args.viewer:
        from modules.viewer import Viewer   # needs OpenCV; only loaded when asked for
        workers.append(Viewer(args.viewer, state, stop_event, avoidance, config["camera"], detector.view,
                              mission.keep_outs, mission.attending, lambda: mission.phase.value, decision=lambda: decision.get("now")))
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
    log.info("rescue logic running, detector: %s, lidar: %s, helper board: %s", config["rescue"]["detector"], lidar_source, helper_port)
    last_phase = mission.phase
    last_status = 0.0
    last_count = 0.0
    estop_held = False
    aimed_at = None           # the last point the boat was sent to
    run_log = None
    if args.log:
        from utils.run_log import RunLog
        run_log = RunLog(args.log)
        log.info("run log: %s", run_log.path)
    try:
        while not stop_event.is_set():
            snapshot, now = state.snapshot(), monotonic()
            if now - last_status >= 5.0:
                last_status = now
                target = snapshot.human_target
                nearest = min((o.distance_m for o in snapshot.obstacles), default=None)
                board = "" if helper is None else " | board off" if not helper.online(now) else                     f" | board {helper.state.battery_v:.1f} V {'lora ok' if helper.state.lora_ok else 'NO LORA'}"
                log.info("%s %s | fix %d | yaw %.0f | person %s | nearest obstacle %s | phase %s%s", snapshot.vehicle.mode or "?",
                         "armed" if snapshot.vehicle.armed else "disarmed", snapshot.gps.fix, snapshot.imu.yaw_deg,
                         f"{target.bearing_body_deg:+.0f} deg {target.distance_m:.1f} m" if target else "none",
                         f"{nearest:.1f} m" if nearest is not None else "none", mission.phase.value, board)
            actions = mission.step(snapshot, now)
            if now - last_count >= 1.0:
                # Repeated, so a lost message loses nothing: the shore bridge shows the count on the
                # dashboard, the helper board switches the alarm on when it rises.
                last_count = now
                link.named_int(VICTIMS_NAME, mission.found)
                if helper is not None:
                    helper.report(snapshot, now, mission.found, mission.phase.value)
                if run_log is not None:
                    run_log.write(snapshot, now, mission.phase.value, mission.found,
                                  helper.state if helper is not None else None, helper is not None and helper.online(now))
            # The E-stop button is on the helper board, which cannot stop the motors: the Pi asks the
            # autopilot to. Once per press; whoever then changes the mode has the boat.
            pressed = helper is not None and helper.online(now) and helper.state.estop
            if pressed and not estop_held:
                link.set_mode(config["rescue"].get("estop_mode", "HOLD"))
                link.statustext("ESTOP PRESSED")
                log.warning("E-stop pressed: %s requested", config["rescue"].get("estop_mode", "HOLD"))
            estop_held = pressed
            state.set_focus(mission.attending())      # the detector keeps the camera on the person being attended
            for action in actions:
                if action.kind == "mode":
                    link.set_mode(action.mode)
                elif action.kind == "goto":
                    link.goto(action.lat, action.lon)
                    aimed_at = (action.lat, action.lon)
                elif action.kind == "face":
                    link.heading_speed(action.heading, action.speed)
                elif action.kind == "skip":
                    link.set_mission_current(action.seq)
                elif action.kind == "report":
                    text = action.text if not action.lat else f"{action.text} {action.lat:.6f} {action.lon:.6f} c={action.confidence:.2f}"
                    link.statustext(text)
                    log.warning(text)
            if args.viewer:
                decision["now"] = {**mission.describe(snapshot, now), "goto": aimed_at if mission.phase.value == "approach" else None}
            if mission.phase is not last_phase:
                log.info("phase %s -> %s", last_phase.value, mission.phase.value)
                last_phase = mission.phase
            sleep(0.2)
    finally:
        stop_event.set()
        for worker in workers:
            worker.join(timeout=1.0)
        if run_log is not None:
            run_log.close()
        log.info("stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

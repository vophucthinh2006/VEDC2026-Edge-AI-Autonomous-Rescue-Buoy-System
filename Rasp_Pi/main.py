#!/usr/bin/env python3
"""RedgeSCUE Pi runtime for the STM32 controller. Run from this directory: python3 main.py
(or ./run.sh stm32). rescue_main.py is the runtime for the Pixhawk boat.

The Pi keeps the search route (modules/waypoints.py, the file of mission.file or --mission) and
follows a person it has seen in a few frames (modules/follow.py); the navigator turns either into
a heading and a speed, which the STM32F407 holds ($NAV)."""
from __future__ import annotations

import argparse
import logging
import signal
import threading
from dataclasses import replace
from pathlib import Path
from time import monotonic, sleep

from modules.ai_vision import VisionWorker
from modules.announcer import Announcer
from modules.follow import Follow
from modules.lidar_serial import SerialLidar
from modules.navigation import BapfNavigator
from modules.perception import cluster_scan
from modules.safety import SafetyArbiter
from modules.state_store import HumanTarget, StateStore
from modules.uart_comm import Stm32Uart
from modules.waypoints import Route, load_route
from utils.config import load_config
from utils.geometry import bearing_deg


def configure_logging(config: dict) -> None:
    Path(config["logging"]["directory"]).mkdir(exist_ok=True)
    logging.basicConfig(level=getattr(logging, config["logging"]["level"].upper()), format="%(asctime)s %(levelname)s %(threadName)s %(name)s: %(message)s")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("config/settings.yaml"))
    parser.add_argument("--overlay", type=Path, action="append", default=[],
                        help="YAML file whose values override --config (config/boat_stm32.yaml); may be repeated, later ones win")
    parser.add_argument("--no-camera", action="store_true", help="run navigation without the TFLite camera worker")
    parser.add_argument("--no-lidar", action="store_true", help="do not open the LiDAR port (serial.lidar_model: none)")
    parser.add_argument("--mission", type=Path, help="route file (YAML or Mission Planner .waypoints); overrides mission.file")
    parser.add_argument("--viewer", type=int, nargs="?", const=8090, metavar="PORT",
                        help="serve the live view (camera, LiDAR, route) on this port, 8090 if no port is given")
    args = parser.parse_args()
    config = load_config(args.config, args.overlay)
    configure_logging(config)
    log = logging.getLogger("main")
    stop_event = threading.Event()
    state = StateStore()
    uart = Stm32Uart(config["serial"]["stm32_port"], int(config["serial"]["stm32_baud"]), state, stop_event)
    lidar_model = "none" if args.no_lidar else str(config["serial"].get("lidar_model", "ld14")).lower()
    if lidar_model not in ("ld14", "lds008", "none"):
        parser.error(f"serial.lidar_model: unknown model {lidar_model}")

    mission_file = args.mission or (Path(config["mission"]["file"]) if config.get("mission", {}).get("file") else None)
    arrive_m = float(config["vehicle"]["arrive_radius_m"])
    route = load_route(mission_file, arrive_m) if mission_file and mission_file.exists() else Route([], arrive_m)
    if mission_file and not mission_file.exists():
        log.warning("mission file %s not found: no route, the boat only follows people it sees", mission_file)
    log.info("route: %d waypoints%s, geofence %s", len(route.waypoints), " (loop)" if route.loop and route.waypoints else "",
             f"{len(route.geofence)} corners" if len(route.geofence) >= 3 else "off")
    state.update_route(tuple((i + 1, lat, lon) for i, (lat, lon) in enumerate(route.waypoints)))
    follow = Follow(config.get("follow", {}))
    navigator = BapfNavigator(config["vehicle"], config["planner"])
    safety = SafetyArbiter(config["vehicle"], route.inside_fence)
    sequence = 0
    lora_rev = 0          # route sent from the dashboard over LoRa now running, 0 = the file's

    def person_confirmed(target: HumanTarget) -> None:
        # The report to the shore is Follow's "found" event, with the person's reckoned position.
        log.info("person seen: bearing=%.1f distance=%.2f confidence=%.2f", target.bearing_body_deg, target.distance_m, target.confidence)

    announcer = Announcer(config.get("audio", {}), stop_event)
    workers: list[threading.Thread] = [uart, announcer]
    if lidar_model != "none":
        # Clusters, for the potential field: one push per object, not one per ray.
        max_m, gap_m = float(config["vehicle"]["influence_distance_m"]) + 2.0, float(config["planner"]["cluster_gap_m"])
        # Without lidar_required the boat runs on with no obstacle avoidance: the safety arbiter
        # only asks for a fresh scan when the LiDAR is required.
        workers.append(SerialLidar(lidar_model, config["serial"]["lidar_port"], int(config["serial"]["lidar_baud"]), config["vehicle"], state, stop_event,
                                   lambda scan: cluster_scan(scan, max_distance_m=max_m, gap_m=gap_m), bool(config["vehicle"].get("lidar_required", True))))
    elif config["vehicle"].get("lidar_required", True):
        log.warning("no LiDAR but vehicle.lidar_required is set: AUTO will stay stopped")
    vision: VisionWorker | None = None
    if config["camera"]["enabled"] and not args.no_camera:
        cam_sequence = 0

        def send_pan(pan_deg: float) -> None:
            nonlocal cam_sequence
            cam_sequence = (cam_sequence + 1) & 0xFFFF
            uart.send("CAM", cam_sequence, f"{pan_deg:.1f}")

        vision = VisionWorker(config["camera"], state, stop_event, person_confirmed, send_pan, float(config["vehicle"]["target_standoff_m"]))
        workers.append(vision)
    status = {"text": "", "lines": []}
    if args.viewer:
        from modules.viewer import Viewer   # needs OpenCV; only loaded when asked for
        workers.append(Viewer(args.viewer, state, stop_event, config["avoidance"], config["camera"],
                              vision.view if vision else (lambda: None), lambda: [], lambda: None, lambda: follow.phase,
                              decision=lambda: {"lines": status["lines"]}))
    for worker in workers:
        worker.start()

    def request_stop(*_unused: object) -> None:
        stop_event.set()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    log.info("RedgeSCUE started")
    last_text = ""
    confirm_seen = -1        # presses of the confirm button (PE4) already announced; -1: none reported yet
    try:
        while not stop_event.is_set():
            sequence = (sequence + 1) & 0xFFFF
            now = monotonic()
            snapshot = state.snapshot()
            # Expire a target even if vision worker stalls; Follow keeps its own last sighting.
            if snapshot.human_target and now - snapshot.human_target.timestamp > config["vehicle"]["target_timeout_s"]:
                state.update_human_target(None)
                snapshot = state.snapshot()
            presses = snapshot.system.confirm_presses
            if presses >= 0:
                # The first report (or one lower than before: the STM32 restarted) is only the starting count.
                if presses > confirm_seen >= 0:
                    announcer.say("supplies_confirmed")
                    log.warning("confirm button pressed: the person reached the boat and took the supplies")
                confirm_seen = presses
            lora = uart.lora_route
            if lora is not None and lora.rev != lora_rev:
                # The dashboard's route replaces the file's, from its first point; the pool's geofence stays.
                route = Route(list(lora.points), arrive_m, lora.loop, route.geofence)
                lora_rev = lora.rev
                state.update_route(tuple((i + 1, lat, lon) for i, (lat, lon) in enumerate(route.waypoints)))
                log.warning("route from the dashboard (rev %d): %s", lora_rev, route.describe())
            if snapshot.gps.fix >= 2 and route.update(snapshot.gps.lat_deg, snapshot.gps.lon_deg):
                log.info("waypoint reached, now %s", route.describe())
            planned, events = follow.step(replace(snapshot, waypoint=route.current), now)
            for event in events:
                if event.kind == "found":
                    # Reckoned from the boat's GPS, the camera's bearing and the box's range; 0, 0 without a fix.
                    lat, lon = (event.lat, event.lon) if event.lat is not None else (0.0, 0.0)
                    uart.send("TXD", sequence, "VICTIM_FOUND", f"{lat:.6f}", f"{lon:.6f}", f"{event.confidence:.2f}")
                    log.warning("following a person: %s", f"reckoned at {lat:.6f} {lon:.6f}" if event.lat is not None else "no GPS fix, no position")
                else:
                    log.warning("person lost: back to the route (%s)", route.describe())
            proposed = navigator.plan(planned)
            command = safety.check(planned, proposed)
            uart.send("NAV", sequence, command.mode, f"{command.speed_mps:.2f}", f"{command.heading_deg:.1f}", config["vehicle"]["nav_ttl_ms"])
            # 4th field: the LoRa route revision running, so the STM32 stops resending it (docs/lora-route-downlink.md)
            uart.send("HBT", sequence, command.reason, lora_rev)
            goal = route.current
            target_bearing = bearing_deg(snapshot.gps.lat_deg, snapshot.gps.lon_deg, *goal) if goal else command.heading_deg
            state.update_steering(command.heading_deg, target_bearing)
            status["text"] = f"{follow.describe()}  {route.describe()}  NAV {command.mode} {command.speed_mps:.2f} m/s {command.heading_deg:.0f} ({command.reason})"
            # The viewer's words: what the boat is doing, the command the STM32 gets, and why.
            status["lines"] = [f"FOLLOW: person {planned.human_target.distance_m:.1f} m away" if planned.human_target
                               else f"SEARCH: {route.describe()}",
                               "going to them along the camera's bearing" if planned.human_target else "the camera sweeps for people",
                               f"NAV {command.mode} {command.speed_mps:.2f} m/s, heading {command.heading_deg:03.0f}",
                               f"reason: {command.reason}",
                               "the STM32F407 holds that heading (PID)"]
            text = f"{follow.phase} {command.reason}"
            if text != last_text:
                log.info("%s", status["text"])
                last_text = text
            sleep(0.10)
    finally:
        # Send best-effort STOP. STM32 TTL remains the independent guaranteed stop.
        uart.send("NAV", sequence + 1, "STOP", "0.00", "0.0", config["vehicle"]["nav_ttl_ms"])
        sleep(0.05)
        stop_event.set()
        for worker in workers:
            worker.join(timeout=1.0)
        log.info("RedgeSCUE stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

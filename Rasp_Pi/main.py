#!/usr/bin/env python3
"""RedgeSCUE Pi runtime for the STM32 controller. Run from this directory: python3 main.py
(or ./run.sh stm32). rescue_main.py is the runtime for the Pixhawk boat.

The Pi keeps the search route (modules/waypoints.py, the file of mission.file or --mission, or the
dashboard's over LoRa) and decides what the boat does (modules/mission.py: IDLE, PATROL, FOLLOW,
RETURN, HOME_HOLD); the navigator turns that into a heading and a speed, which the STM32F407
holds ($NAV), and the safety arbiter can always overrule it."""
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
from modules.bt_manual import BluetoothManual
from modules.drive_log import DriveLog
from modules.heading_bias import HeadingBias
from modules.lidar_serial import SerialLidar
from modules.mission import Mission
from modules.navigation import BapfNavigator, NavCommand
from modules.perception import RasterSweep, SweepPattern, cluster_scan
from modules.safety import SafetyArbiter
from modules.state_store import RISK_NAMES, HumanTarget, StateStore
from modules.uart_comm import Stm32Uart
from modules.waypoints import Route, start_route
from utils.config import load_config
from utils.geometry import bearing_deg, heading_tenths
from utils.log_files import run_log_path
from utils.run_log import RunLog


def configure_logging(config: dict) -> Path:
    """The terminal and a file of this run's own (utils/log_files.py), returned."""
    settings = config["logging"]
    path = run_log_path(Path(settings["directory"]), "main", int(settings.get("keep_runs", 30)))
    logging.basicConfig(level=getattr(logging, settings["level"].upper()), format="%(asctime)s %(levelname)s %(threadName)s %(name)s: %(message)s",
                        handlers=[logging.StreamHandler(), logging.FileHandler(path, encoding="utf-8")])

    def thread_failed(args: threading.ExceptHookArgs) -> None:
        # A worker that dies (vision, LiDAR, viewer) said so on stderr only.
        logging.getLogger("main").critical("thread %s failed", args.thread.name if args.thread else "?",
                                           exc_info=(args.exc_type, args.exc_value, args.exc_traceback))
    threading.excepthook = thread_failed
    return path


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
    log_path = configure_logging(config)
    log = logging.getLogger("main")
    log.info("log file: %s", log_path)
    # Every AUTO loop as CSV beside it, for the lake test's report (tools/lake_report.py, docs/lake-test.md)
    log_dir = Path(config["logging"]["directory"])
    run_log = RunLog(log_dir, run_log_path(log_dir, "nav", int(config["logging"].get("keep_runs", 30)), suffix=".csv"))
    log.info("nav log: %s", run_log.path)
    stop_event = threading.Event()
    state = StateStore()
    uart = Stm32Uart(config["serial"]["stm32_port"], int(config["serial"]["stm32_baud"]), state, stop_event)
    lidar_model = "none" if args.no_lidar else str(config["serial"].get("lidar_model", "ld14")).lower()
    if lidar_model not in ("ld14", "lds008", "none"):
        parser.error(f"serial.lidar_model: unknown model {lidar_model}")

    mission_file = args.mission or (Path(config["mission"]["file"]) if config.get("mission", {}).get("file") else None)
    arrive_m = float(config["vehicle"]["arrive_radius_m"])
    # mission.waypoints_from_file false (the STM32 boat, no RC): the file gives the geofence only and
    # the boat waits for a route from the dashboard; follow.only_on_route keeps it still meanwhile.
    from_file = bool(config.get("mission", {}).get("waypoints_from_file", True))
    only_on_route = bool(config.get("follow", {}).get("only_on_route", False))
    route = start_route(mission_file, arrive_m, from_file)
    if mission_file and not mission_file.exists():
        log.warning("mission file %s not found: no route, no geofence", mission_file)
    if not from_file:
        log.info("waiting for a route from the dashboard%s", "; the boat stays still until then" if only_on_route else "")
    log.info("route: %d waypoints%s, geofence %s", len(route.waypoints), " (loop)" if route.loop and route.waypoints else "",
             f"{len(route.geofence)} corners" if len(route.geofence) >= 3 else "off")
    state.update_route(tuple((i + 1, lat, lon) for i, (lat, lon) in enumerate(route.waypoints)))
    mission = Mission(config, route, only_on_route)
    navigator = BapfNavigator(config["vehicle"], config["planner"])
    # The fence is the pool's: a route from the dashboard keeps it (Route(..., route.geofence) below).
    safety = SafetyArbiter(config["vehicle"], route.inside_fence)
    heading_bias = HeadingBias(config["vehicle"].get("heading_bias"))
    last_mode = "STOP"
    sequence = 0
    txd_sequence = 0
    lora_rev = 0          # route sent from the dashboard over LoRa now running, 0 = the file's

    def person_confirmed(target: HumanTarget) -> None:
        # The reports to the shore are the mission's (modules/mission.py), with reckoned positions.
        log.info("person seen: %s bearing=%.1f distance=%.2f confidence=%.2f", RISK_NAMES[target.risk], target.bearing_body_deg,
                 target.distance_m, target.confidence)

    announcer = Announcer(config.get("audio", {}), stop_event)
    workers: list[threading.Thread] = [uart, announcer]
    # Manual control from the phone over Bluetooth (modules/bt_manual.py): V into MANUAL, v back.
    manual: BluetoothManual | None = None
    if config.get("manual", {}).get("enabled", False):
        manual = BluetoothManual(config["manual"], stop_event)
        workers.append(manual)
    scan = config["camera"].get("scan", {})
    # camera.tilt_enabled: the pan-tilt gimbal. The phone's W sweep is then the AUTO one, over the rows too.
    tilt_on = bool(config["camera"].get("tilt_enabled", False))
    accel = scan.get("sweep_accel_deg_s2", 0.0)   # eased turns at the ends, as the AUTO sweep
    manual_sweep = (RasterSweep(scan.get("sweep_limit_deg", 45), scan.get("sweep_rate_deg_s", 15), scan.get("tilt_rows_deg", [0, 20]),
                                scan.get("tilt_settle_s", 0.6), accel)
                    if tilt_on and str(scan.get("mode", "")).lower() == "raster"
                    else SweepPattern(scan.get("sweep_limit_deg", 45), scan.get("sweep_rate_deg_s", 15), accel))
    manual_pan: dict[str, float | None] = {"deg": 0.0, "tilt": float(config["camera"].get("tilt_home_deg", 10.0)) if tilt_on else None}
    cam_sequence = 0

    def send_cam(pan_deg: float, tilt_deg: float | None = None) -> None:
        # $CAM,seq,pan[,tilt]: the F407 glides the gimbal there and clamps the tilt to 0..60 deg; without
        # a tilt it keeps its own (10 deg).
        nonlocal cam_sequence
        cam_sequence = (cam_sequence + 1) & 0xFFFF
        tilt = () if tilt_deg is None else (f"{tilt_deg:.1f}",)
        uart.send("CAM", cam_sequence, f"{pan_deg:.1f}", *tilt)

    def manual_pan_loop() -> None:
        # The phone's W sweep at camera.pan_rate_hz from a thread of its own, as the AUTO sweep
        # (ai_vision.py _pan_loop): from the 10 Hz main loop the servo moved in visible steps.
        period = 1.0 / float(config["camera"].get("pan_rate_hz", 20.0) or 20.0)
        sweeping = False
        while not stop_event.wait(period):
            phone = manual.state()
            on = phone.active and phone.sweep
            if on:
                now = monotonic()
                if not sweeping:
                    manual_sweep.start(now, manual_pan["deg"])
                manual_pan["deg"] = manual_sweep.frame_done(now)
                manual_pan["tilt"] = getattr(manual_sweep, "tilt_deg", manual_pan["tilt"])
                send_cam(manual_pan["deg"], manual_pan["tilt"])
            sweeping = on
    if manual is not None:
        workers.append(threading.Thread(target=manual_pan_loop, name="manual-pan", daemon=True))
    if lidar_model != "none":
        # Clusters, for the potential field: one push per object, not one per ray.
        # As far as the LiDAR sees (vehicle.lidar_max_range_m), so the detour ahead (navigation.py) sees obstacles in
        # time; unset: the field's reach plus 2 m, as before.
        max_m = float(config["vehicle"].get("lidar_max_range_m", float(config["vehicle"]["influence_distance_m"]) + 2.0))
        gap_m = float(config["planner"]["cluster_gap_m"])
        # Without lidar_required the boat runs on with no obstacle avoidance: the safety arbiter
        # only asks for a fresh scan when the LiDAR is required.
        workers.append(SerialLidar(lidar_model, config["serial"]["lidar_port"], int(config["serial"]["lidar_baud"]), config["vehicle"], state, stop_event,
                                   lambda scan: cluster_scan(scan, max_distance_m=max_m, gap_m=gap_m), bool(config["vehicle"].get("lidar_required", True))))
    elif config["vehicle"].get("lidar_required", True):
        log.warning("no LiDAR but vehicle.lidar_required is set: AUTO will stay stopped")
    vision: VisionWorker | None = None
    if config["camera"]["enabled"] and not args.no_camera:
        def send_pan(pan_deg: float, tilt_deg: float | None = None) -> None:
            # In MANUAL the phone owns the camera servo (W/w): the detector keeps looking, its pan is dropped.
            if manual is None or not manual.state().active:
                manual_pan["deg"], manual_pan["tilt"] = pan_deg, tilt_deg
                send_cam(pan_deg, tilt_deg)

        vision = VisionWorker(config["camera"], state, stop_event, person_confirmed, send_pan, navigator.standoff_m,
                              lambda: mission.sweeping, lambda: mission.detecting, tilt=tilt_on)
        workers.append(vision)
    status = {"text": "", "lines": [], "phase": ""}
    if args.viewer:
        from modules.viewer import Viewer   # needs OpenCV; only loaded when asked for
        workers.append(Viewer(args.viewer, state, stop_event, config["avoidance"], config["camera"],
                              vision.view if vision else (lambda: None), lambda: [], lambda: None, lambda: status["phase"],
                              decision=lambda: {"lines": status["lines"]}))
    for worker in workers:
        worker.start()

    stop_why = {"text": ""}

    def request_stop(signum: int, _frame: object = None) -> None:
        stop_why["text"] = f"signal {signal.Signals(signum).name}"
        stop_event.set()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    # The ssh window of run_all.bat closed: stop as on Ctrl+C, and say so. Left ignored under nohup
    # (tools/bench_demo.sh).
    if hasattr(signal, "SIGHUP") and signal.getsignal(signal.SIGHUP) is not signal.SIG_IGN:
        signal.signal(signal.SIGHUP, request_stop)
    log.info("RedgeSCUE started")
    last_text = ""
    drive_log = DriveLog()
    try:
        while not stop_event.is_set():
            sequence = (sequence + 1) & 0xFFFF
            now = monotonic()
            snapshot = state.snapshot()
            # Expire a target even if vision worker stalls; Follow keeps its own last sighting.
            if snapshot.human_target and now - snapshot.human_target.timestamp > config["vehicle"]["target_timeout_s"]:
                state.update_human_target(None)
                snapshot = state.snapshot()
            # The compass's bias from the GPS course on straight runs (modules/heading_bias.py): the mission and
            # the navigator work in true headings; $NAV gets the IMU's frame back (+ bias) for the F407's PID.
            bias = heading_bias.update(now, snapshot.imu.yaw_deg, snapshot.gps.course_deg, snapshot.gps.speed_mps,
                                       snapshot.gps.fix, last_mode == "BACK")
            if bias:
                snapshot = replace(snapshot, imu=replace(snapshot.imu, yaw_deg=(snapshot.imu.yaw_deg - bias) % 360.0))
            lora = uart.lora_route
            if lora is not None and lora.rev != lora_rev:
                # The dashboard's route replaces the file's, from its first point; the pool's geofence stays.
                route = Route(list(lora.points), arrive_m, lora.loop, route.geofence)
                mission.set_route(route)
                lora_rev = lora.rev
                state.update_route(tuple((i + 1, lat, lon) for i, (lat, lon) in enumerate(route.waypoints)))
                log.warning("route from the dashboard (rev %d): %s", lora_rev, route.describe())
            step = mission.step(snapshot, now)
            status["phase"] = mission.state
            if step.supplies_taken:
                announcer.say("supplies_confirmed")
                log.warning("confirm button pressed: the person reached the boat and took the supplies")
            for report in step.reports:
                # Reckoned from the boat's GPS, the camera's bearing and the box's range; 0, 0 without a fix.
                # 7th field: the person's risk, 0 SAFE .. 3 CRITICAL. A new person is VICTIM_FOUND (the
                # F407 counts it); a new risk or position of the one followed is VICTIM_UPDATE (not counted).
                txd_sequence = (txd_sequence + 1) & 0xFFFF
                kind = "VICTIM_FOUND" if report.why == "found" else "VICTIM_UPDATE"
                uart.send("TXD", txd_sequence, kind, f"{report.lat:.6f}", f"{report.lon:.6f}", f"{report.confidence:.2f}", report.risk)
                log.warning("report (%s): %s person %s", report.why, RISK_NAMES[report.risk],
                            f"at {report.lat:.6f} {report.lon:.6f}" if (report.lat, report.lon) != (0.0, 0.0) else "with no position (no GPS fix)")
            phone = manual.state(now) if manual else None
            if phone is not None and phone.active:
                # MANUAL: the phone's motors, rudders and lights go straight to the F407 ($MAN), in place of
                # the navigator's $NAV; the mission and the detector carry on, so v resumes where it was.
                hold = snapshot.link.hold
                rear, front = (0.0, 0.0) if hold else (phone.rear, phone.front)
                uart.send("MAN", sequence, f"{rear:.2f}", f"{front:.2f}", f"{phone.steer:.2f}", phone.lights,
                          config["vehicle"]["nav_ttl_ms"])
                fps = vision.fps if vision else None
                uart.send("HBT", sequence, "dashboard_hold" if hold else "manual", lora_rev, mission.code,
                          mission.waypoint_field, "-" if fps is None else round(fps * 10))
                status["phase"] = "MANUAL"
                status["lines"] = [f"MANUAL rear {rear:.2f} front {front:+.2f} steer {phone.steer:+.1f}",
                                   f"level {phone.level}/9, lights {phone.lights}, camera {'sweep' if phone.sweep else 'held'}"]
                text = f"MANUAL {phone.motion} {phone.level} {phone.steer:+.1f}{' hold' if hold else ''}"
                if text != last_text:
                    log.info("%s", text)
                    last_text = text
                sleep(0.10)
                continue
            planned = step.planned
            proposed = navigator.plan(planned)
            command = safety.check(planned, proposed)
            if snapshot.link.hold:
                # The admin pressed DỪNG PHAO: the F407 keeps the motors off whatever comes; say so too, so
                # the viewer and the log show why the boat stands. The route and the mission carry on.
                command = NavCommand("STOP", 0.0, command.heading_deg, "dashboard_hold")
            last_mode = command.mode
            uart.send("NAV", sequence, command.mode, f"{command.speed_mps:.2f}", heading_tenths((command.heading_deg + bias) % 360.0),
                      config["vehicle"]["nav_ttl_ms"])
            # 4th field: the LoRa route revision running, so the STM32 stops resending it (docs/lora-route-downlink.md);
            # then the mission and the waypoint gone to, which the F407 sends on to the dashboard with the reason,
            # and the detector's frames a second x10 ("-" while it is off: no route, or no camera)
            fps = vision.fps if vision else None
            uart.send("HBT", sequence, command.reason, lora_rev, mission.code, mission.waypoint_field,
                      "-" if fps is None else round(fps * 10))
            wp = mission.route.current   # the waypoint itself; planned.waypoint is the aim point along the leg
            run_log.write(snapshot, now, mission.state, 0, cmd_mode=command.mode, cmd_speed_mps=f"{command.speed_mps:.2f}",
                          cmd_heading_deg=f"{command.heading_deg:.1f}", reason=command.reason, wp_index=mission.waypoint_field,
                          wp_lat=f"{wp[0]:.7f}" if wp else "", wp_lon=f"{wp[1]:.7f}" if wp else "", heading_bias_deg=f"{bias:.1f}")
            goal = planned.waypoint
            target_bearing = bearing_deg(snapshot.gps.lat_deg, snapshot.gps.lon_deg, *goal) if goal else command.heading_deg
            state.update_steering(command.heading_deg, target_bearing)
            status["text"] = f"{mission.describe()}  NAV {command.mode} {command.speed_mps:.2f} m/s {command.heading_deg:.0f} ({command.reason})"
            # The viewer's words: what the boat is doing, the command the STM32 gets, and why.
            status["lines"] = mission.lines(snapshot) + [f"NAV {command.mode} {command.speed_mps:.2f} m/s, heading {command.heading_deg:03.0f}",
                                                         f"reason: {command.reason}",
                                                         "the STM32F407 holds that heading (PID)"]
            text = f"{mission.state} {command.reason}"
            if text != last_text:
                log.info("%s", status["text"])
                last_text = text
            for line in drive_log.update(command, snapshot, now):
                log.warning("%s", line)
            sleep(0.10)
    except Exception:
        stop_why["text"] = "the main loop failed"
        log.exception("main loop failed: the boat stops (the F407 stops the motors without $NAV)")
        raise
    finally:
        # Send best-effort STOP. STM32 TTL remains the independent guaranteed stop.
        uart.send("NAV", sequence + 1, "STOP", "0.00", "0.0", config["vehicle"]["nav_ttl_ms"])
        sleep(0.05)
        # Signal, the main loop's failure, or a worker that cannot go on (a required LiDAR).
        log.warning("RedgeSCUE stopping: %s", stop_why["text"] or "a worker stopped the program")
        stop_event.set()
        for worker in workers:
            worker.join(timeout=1.0)
        run_log.close()
        log.info("RedgeSCUE stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

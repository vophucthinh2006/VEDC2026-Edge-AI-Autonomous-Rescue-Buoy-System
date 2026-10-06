#!/usr/bin/env python3
"""RedgeSCUE Pi runtime for the STM32 controller. Run from this directory: python3 main.py
(or ./run.sh stm32). rescue_main.py is the runtime for the Pixhawk boat."""
from __future__ import annotations

import argparse
import logging
import queue
import signal
import threading
from pathlib import Path
from time import monotonic, sleep

import serial
import yaml

from modules.ai_vision import VisionWorker
from modules.lidar_ld14 import LD14Decoder, ScanAssembler
from modules.lidar_lds008 import LDS008Decoder, START_COMMAND
from modules.navigation import BapfNavigator
from modules.perception import cluster_scan
from modules.safety import SafetyArbiter
from modules.state_store import HumanTarget, StateStore
from modules.uart_comm import Stm32Uart
from utils.geometry import signed_angle_deg


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class LidarWorker(threading.Thread):
    def __init__(self, model: str, port: str, baud: int, vehicle: dict, planner: dict, state: StateStore, stop_event: threading.Event) -> None:
        super().__init__(name="lidar", daemon=True)
        self.model, self.port, self.baud, self.vehicle, self.planner, self.state, self.stop = model, port, baud, vehicle, planner, state, stop_event
        self.log = logging.getLogger(__name__)

    def run(self) -> None:
        decoder, assembler = (LDS008Decoder() if self.model == "lds008" else LD14Decoder()), ScanAssembler()
        offset = float(self.vehicle["lidar_yaw_offset_deg"])
        # The sensor's own angle may run anticlockwise seen from above; the boat's bearings run clockwise.
        sign = -1.0 if self.vehicle.get("lidar_anticlockwise", False) else 1.0
        try:
            with serial.Serial(self.port, self.baud, timeout=0.1) as ser:
                self.log.info("%s UART open: %s @ %d", self.model.upper(), self.port, self.baud)
                if self.model == "lds008":
                    ser.write(START_COMMAND)   # the motor only spins once told to
                while not self.stop.is_set():
                    points = decoder.feed(ser.read(256))
                    scan = assembler.add(points)
                    if scan is None:
                        continue
                    calibrated = tuple(type(point)((sign * point.angle_deg + offset) % 360.0, point.distance_m, point.intensity) for point in scan)
                    obstacles = cluster_scan(calibrated, max_distance_m=float(self.vehicle["influence_distance_m"]) + 2.0, gap_m=float(self.planner["cluster_gap_m"]))
                    self.state.update_obstacles(obstacles)
        except serial.SerialException as exc:
            # Without lidar_required the boat runs on with no obstacle avoidance: the safety arbiter
            # only asks for a fresh scan when the LiDAR is required.
            if self.vehicle.get("lidar_required", True):
                self.log.error("%s UART unavailable: %s", self.model.upper(), exc)
                self.stop.set()
            else:
                self.log.warning("%s UART unavailable, running without obstacle avoidance: %s", self.model.upper(), exc)


def configure_logging(config: dict) -> None:
    Path(config["logging"]["directory"]).mkdir(exist_ok=True)
    logging.basicConfig(level=getattr(logging, config["logging"]["level"].upper()), format="%(asctime)s %(levelname)s %(threadName)s %(name)s: %(message)s")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("config/settings.yaml"))
    parser.add_argument("--no-camera", action="store_true", help="run navigation without the TFLite camera worker")
    parser.add_argument("--no-lidar", action="store_true", help="do not open the LiDAR port (serial.lidar_model: none)")
    args = parser.parse_args()
    config = load_config(args.config)
    configure_logging(config)
    log = logging.getLogger("main")
    stop_event = threading.Event()
    state = StateStore()
    uart = Stm32Uart(config["serial"]["stm32_port"], int(config["serial"]["stm32_baud"]), state, stop_event)
    lidar_model = "none" if args.no_lidar else str(config["serial"].get("lidar_model", "ld14")).lower()
    if lidar_model not in ("ld14", "lds008", "none"):
        parser.error(f"serial.lidar_model: unknown model {lidar_model}")
    navigator, safety = BapfNavigator(config["vehicle"], config["planner"]), SafetyArbiter(config["vehicle"])
    sequence = 0

    def person_confirmed(target: HumanTarget) -> None:
        snap = state.snapshot()
        # GPS is the notification point; confidence is explicitly transmitted, no image is sent.
        uart.send("TXD", sequence, "VICTIM_FOUND", f"{snap.gps.lat_deg:.6f}", f"{snap.gps.lon_deg:.6f}", f"{target.confidence:.2f}")
        log.warning("person locked: bearing=%.1f distance=%.2f confidence=%.2f", target.bearing_body_deg, target.distance_m, target.confidence)

    workers: list[threading.Thread] = [uart]
    if lidar_model != "none":
        workers.append(LidarWorker(lidar_model, config["serial"]["lidar_port"], int(config["serial"]["lidar_baud"]), config["vehicle"], config["planner"], state, stop_event))
    elif config["vehicle"].get("lidar_required", True):
        log.warning("no LiDAR but vehicle.lidar_required is set: AUTO will stay stopped")
    if config["camera"]["enabled"] and not args.no_camera:
        cam_sequence = 0

        def send_pan(pan_deg: float) -> None:
            nonlocal cam_sequence
            cam_sequence = (cam_sequence + 1) & 0xFFFF
            uart.send("CAM", cam_sequence, f"{pan_deg:.1f}")

        workers.append(VisionWorker(config["camera"], state, stop_event, person_confirmed, send_pan, float(config["vehicle"]["target_standoff_m"])))
    for worker in workers:
        worker.start()

    def request_stop(*_unused: object) -> None:
        stop_event.set()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    log.info("RedgeSCUE started")
    try:
        while not stop_event.is_set():
            sequence = (sequence + 1) & 0xFFFF
            snapshot = state.snapshot()
            # Expire a target even if vision worker stalls; then resume waypoint mission.
            if snapshot.human_target and monotonic() - snapshot.human_target.timestamp > config["vehicle"]["target_timeout_s"]:
                state.update_human_target(None)
                snapshot = state.snapshot()
            proposed = navigator.plan(snapshot)
            command = safety.check(snapshot, proposed)
            uart.send("NAV", sequence, command.mode, f"{command.speed_mps:.2f}", f"{command.heading_deg:.1f}", config["vehicle"]["nav_ttl_ms"])
            uart.send("HBT", sequence, command.reason)
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

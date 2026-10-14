"""The LiDAR on one of the Pi's UARTs (LDS-008 or LD14), for both runtimes.

Every revolution is turned to the boat's frame (bearing from the bow, clockwise) and handed to
to_obstacles, which decides what an obstacle is: clusters for the Pi's own potential field
(main.py), one per ray for the autopilot's avoidance (rescue_main.py). A revolution with nothing
in range still refreshes the state store, so "no obstacle" and "no LiDAR" stay apart: the decoders
pass a reading that saw nothing on as distance 0, intensity 0, which to_obstacles leaves out.
"""
from __future__ import annotations

import logging
import threading
from typing import Callable

import serial

from modules.lidar_ld14 import LD14Decoder, LidarPoint, ScanAssembler
from modules.lidar_lds008 import LDS008Decoder, START_COMMAND
from modules.state_store import StateStore

MODELS = ("ld14", "lds008")
REOPEN_S = 3.0


def to_boat_frame(scan: tuple[LidarPoint, ...], yaw_offset_deg: float, anticlockwise: bool) -> tuple[LidarPoint, ...]:
    """The sensor's own angle may run anticlockwise seen from above, and its zero need not point
    at the bow; the boat's bearings run clockwise from the bow."""
    sign = -1.0 if anticlockwise else 1.0
    return tuple(LidarPoint((sign * point.angle_deg + yaw_offset_deg) % 360.0, point.distance_m, point.intensity) for point in scan)


class SerialLidar(threading.Thread):
    def __init__(self, model: str, port: str, baud: int, vehicle: dict, state: StateStore, stop_event: threading.Event,
                 to_obstacles: Callable[[tuple[LidarPoint, ...]], tuple[object, ...]], required: bool) -> None:
        super().__init__(name="lidar", daemon=True)
        self.model, self.port, self.baud, self.vehicle, self.state, self.stop = model, port, baud, vehicle, state, stop_event
        self.to_obstacles, self.required = to_obstacles, required
        self.log = logging.getLogger(__name__)

    def run(self) -> None:
        decoder, assembler = (LDS008Decoder() if self.model == "lds008" else LD14Decoder()), ScanAssembler()
        offset = float(self.vehicle["lidar_yaw_offset_deg"])
        anticlockwise = bool(self.vehicle.get("lidar_anticlockwise", False))
        told = False
        while not self.stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.1) as ser:
                    self.log.info("%s UART open: %s @ %d", self.model.upper(), self.port, self.baud)
                    told = False
                    if self.model == "lds008":
                        ser.write(START_COMMAND)   # the motor only spins once told to
                    while not self.stop.is_set():
                        scan = assembler.add(decoder.feed(ser.read(256)))
                        if scan is not None:
                            self.state.update_obstacles(self.to_obstacles(to_boat_frame(scan, offset, anticlockwise)))
            except (serial.SerialException, OSError) as exc:
                # required: the program cannot go on without it. Otherwise the scans simply stop,
                # whoever reads them sees them go stale, and the port is tried again (a USB adapter
                # that came loose and was pushed back in).
                if self.required:
                    self.log.error("%s UART unavailable: %s", self.model.upper(), exc)
                    self.stop.set()
                    return
                if not told:
                    told = True
                    self.log.warning("%s UART unavailable, no LiDAR data: %s", self.model.upper(), exc)
                self.stop.wait(REOPEN_S)

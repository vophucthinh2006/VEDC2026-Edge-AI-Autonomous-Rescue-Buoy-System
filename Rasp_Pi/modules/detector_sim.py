"""Geometric stand-in for the camera detector: 'sees' known victim positions.

For exercising the rescue logic without a camera or a model. It reports a person when a listed
position lies inside the camera's field of view and detection range, with the bearing and
distance the real detector would give. The field of view turns like the real camera's: the
search sweep while the autopilot searches, on the person once one is in view, ahead otherwise.
It says nothing about how well the real model detects.
"""
from __future__ import annotations

import logging
import random
import threading
from time import monotonic, sleep

from modules.perception import make_scan
from modules.state_store import HumanTarget, StateStore
from utils.geometry import bearing_deg, haversine_m, signed_angle_deg


class SimulatedDetector(threading.Thread):
    def __init__(self, camera: dict, sim: dict, state: StateStore, stop_event: threading.Event) -> None:
        super().__init__(name="detector-sim", daemon=True)
        self.state, self.stop = state, stop_event
        self.half_fov = float(camera["horizontal_fov_deg"]) / 2.0
        self.range_m = float(sim["detect_range_m"])
        # [lat, lon] or [lat, lon, range_m]: someone standing in full view is seen from further than a swimmer.
        self.victims = [(float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 else self.range_m) for v in sim["victims"]]
        self.pan_limit = float(camera["pan_limit_deg"])
        scan = camera.get("scan") or {}
        self._scan = make_scan(scan)
        self._scan_modes = {str(mode).upper() for mode in scan.get("modes", [])}
        self.log = logging.getLogger(__name__)
        self._view = {"frame": None, "note": "simulated detector: no camera frame", "pan_deg": 0.0, "scanning": False}

    def view(self) -> dict:
        """For the live viewer: where the simulated field of view points."""
        return dict(self._view)

    def run(self) -> None:
        self.log.info("simulated detector: %d victim(s), range %.1f m, FOV +-%.0f deg", len(self.victims), self.range_m, self.half_fov)
        pan, scanning = 0.0, False
        while not self.stop.is_set():
            snap = self.state.snapshot()
            now = monotonic()
            best: HumanTarget | None = None
            seen: list[tuple[HumanTarget, tuple[float, float]]] = []
            if snap.gps.fix >= 3 and not (scanning and not self._scan.usable(now)):
                for lat, lon, range_m in self.victims:
                    distance = haversine_m(snap.gps.lat_deg, snap.gps.lon_deg, lat, lon)
                    bearing = signed_angle_deg(bearing_deg(snap.gps.lat_deg, snap.gps.lon_deg, lat, lon) - snap.imu.yaw_deg)
                    if distance > range_m or abs(signed_angle_deg(bearing - pan)) > self.half_fov:
                        continue
                    # A bounding box gives the range to about +-10 %.
                    seen.append((HumanTarget(bearing + random.gauss(0.0, 1.0), distance * random.gauss(1.0, 0.05),
                                             max(0.6, 0.9 - 0.05 * distance), now), (lat, lon)))
            if seen:
                # Followed: the person the rescue logic named, if in view, else the nearest.
                if snap.focus is not None:
                    best = min(seen, key=lambda s: haversine_m(s[1][0], s[1][1], *snap.focus))[0]
                else:
                    best = min(seen, key=lambda s: s[0].distance_m)[0]
            self.state.update_human_targets(tuple(target for target, _ in seen), best)
            if best is not None:
                pan, scanning = max(-self.pan_limit, min(self.pan_limit, best.bearing_body_deg)), False
            elif self._scan is not None and snap.vehicle.armed and snap.vehicle.mode in self._scan_modes:
                if not scanning:
                    pan, scanning = self._scan.start(now, pan), True
                elif self._scan.usable(now):
                    pan = self._scan.frame_done(now)
            else:
                pan, scanning = 0.0, False
            self._view["pan_deg"], self._view["scanning"] = pan, scanning
            sleep(0.2)

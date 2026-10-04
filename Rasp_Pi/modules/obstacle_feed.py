"""Obstacles for the autopilot's own avoidance: MAVLink OBSTACLE_DISTANCE, 72 sectors of 5 deg.

Two sources: LiDAR returns, and keep-out circles around people already attended, so the search
pattern, once resumed, does not run through the person it just found. ArduPilot (OA_TYPE 1,
PRX1_TYPE 2) bends the path around what is reported here and returns to the mission.

The person the boat is going to, or holding next to, is taken out of the LiDAR data: reported as
an obstacle they would make the autopilot stop short of the stand-off or back away from them.
"""
from __future__ import annotations

import math
import threading
from time import sleep
from typing import Callable

from modules.state_store import Snapshot, StateStore
from utils.geometry import bearing_deg, haversine_m, signed_angle_deg

SECTORS = 72
SECTOR_DEG = 360.0 / SECTORS


def sectors(objects: list[tuple[float, float, float]], min_m: float, max_m: float) -> list[int]:
    """objects: (bearing from the bow in degrees, clockwise; distance to the centre in m; radius in m).
    Returns centimetres per sector, sector 0 centred on the bow, clockwise; max + 1 cm means clear."""
    clear = int(max_m * 100) + 1
    out = [clear] * SECTORS
    for bearing, distance, radius in objects:
        edge = distance - radius
        if edge > max_m:
            continue
        edge = max(edge, min_m)
        # Half-angle the circle subtends; an object the boat is inside of covers everything.
        half = 180.0 if distance <= radius else math.degrees(math.asin(radius / distance))
        first = math.floor((bearing - half) / SECTOR_DEG + 0.5)
        last = math.floor((bearing + half) / SECTOR_DEG + 0.5)
        for index in range(first, min(last, first + SECTORS - 1) + 1):
            out[index % SECTORS] = min(out[index % SECTORS], int(edge * 100))
    return out


def _polar(snap: Snapshot, lat: float, lon: float) -> tuple[float, float]:
    """Bearing from the bow (degrees, clockwise) and distance of a map position."""
    bearing = signed_angle_deg(bearing_deg(snap.gps.lat_deg, snap.gps.lon_deg, lat, lon) - snap.imu.yaw_deg)
    return bearing, haversine_m(snap.gps.lat_deg, snap.gps.lon_deg, lat, lon)


def objects_from(snap: Snapshot, keep_outs: list[tuple[float, float]], keep_out_radius_m: float,
                 attending: tuple[float, float] | None = None) -> list[tuple[float, float, float]]:
    found = [(o.bearing_body_deg, o.distance_m, o.width_m / 2.0) for o in snap.obstacles]
    if snap.gps.fix < 3:
        return found
    if attending is not None:
        bearing, distance = _polar(snap, *attending)

        def apart(o: tuple[float, float, float]) -> float:
            return math.sqrt(max(0.0, o[1] ** 2 + distance ** 2 - 2.0 * o[1] * distance * math.cos(math.radians(o[0] - bearing))))
        found = [o for o in found if apart(o) > keep_out_radius_m]
    for lat, lon in keep_outs:
        found.append((*_polar(snap, lat, lon), keep_out_radius_m))
    return found


class ObstacleFeed(threading.Thread):
    def __init__(self, avoidance: dict, state: StateStore, stop_event: threading.Event,
                 keep_outs: Callable[[], list[tuple[float, float]]], attending: Callable[[], tuple[float, float] | None],
                 send: Callable[[list[int], int, int], None]) -> None:
        super().__init__(name="obstacles", daemon=True)
        self.a, self.state, self.stop, self.keep_outs, self.send = avoidance, state, stop_event, keep_outs, send
        self.attending = attending

    def run(self) -> None:
        min_m, max_m = float(self.a["min_range_m"]), float(self.a["max_range_m"])
        period = 1.0 / float(self.a["rate_hz"])
        while not self.stop.is_set():
            objects = objects_from(self.state.snapshot(), list(self.keep_outs()), float(self.a["keep_out_radius_m"]), self.attending())
            self.send(sectors(objects, min_m, max_m), int(min_m * 100), int(max_m * 100))
            sleep(period)

"""LiDAR-sector geometry shared by the STM32 runtime and its live viewer."""
from __future__ import annotations

import math

from modules.state_store import Snapshot
from utils.geometry import bearing_deg, haversine_m, signed_angle_deg


SECTORS = 72
SECTOR_DEG = 360.0 / SECTORS


def sectors(objects: list[tuple[float, float, float]], min_m: float, max_m: float) -> list[int]:
    """Return the nearest edge distance in centimetres for each 5-degree sector."""
    clear = int(max_m * 100) + 1
    out = [clear] * SECTORS
    for bearing, distance, radius in objects:
        edge = distance - radius
        if edge > max_m:
            continue
        edge = max(edge, min_m)
        half = 180.0 if distance <= radius else math.degrees(math.asin(radius / distance))
        first = math.floor((bearing - half) / SECTOR_DEG + 0.5)
        last = math.floor((bearing + half) / SECTOR_DEG + 0.5)
        for index in range(first, min(last, first + SECTORS - 1) + 1):
            out[index % SECTORS] = min(out[index % SECTORS], int(edge * 100))
    return out


def _polar(snap: Snapshot, lat: float, lon: float) -> tuple[float, float]:
    bearing = signed_angle_deg(bearing_deg(snap.gps.lat_deg, snap.gps.lon_deg, lat, lon) - snap.imu.yaw_deg)
    return bearing, haversine_m(snap.gps.lat_deg, snap.gps.lon_deg, lat, lon)


def objects_from(snap: Snapshot, keep_outs: list[tuple[float, float]], keep_out_radius_m: float,
                 attending: tuple[float, float] | None = None) -> list[tuple[float, float, float]]:
    """Combine raw LiDAR returns and map keep-outs for the STM32 viewer."""
    found = [(o.bearing_body_deg, o.distance_m, o.width_m / 2.0) for o in snap.obstacles]
    if snap.gps.fix < 3:
        return found
    if attending is not None:
        bearing, distance = _polar(snap, *attending)

        def apart(item: tuple[float, float, float]) -> float:
            return math.sqrt(max(0.0, item[1] ** 2 + distance ** 2 - 2.0 * item[1] * distance * math.cos(math.radians(item[0] - bearing))))

        found = [item for item in found if apart(item) > keep_out_radius_m]
    for lat, lon in keep_outs:
        found.append((*_polar(snap, lat, lon), keep_out_radius_m))
    return found

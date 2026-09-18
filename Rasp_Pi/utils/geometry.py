"""Small, dependency-free geometry helpers. Heading is degrees clockwise from North."""
from __future__ import annotations

import math
from typing import Iterable


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_deg(angle: float) -> float:
    return angle % 360.0


def signed_angle_deg(angle: float) -> float:
    """Normalize to [-180, 180). Positive means clockwise/right turn."""
    return (angle + 180.0) % 360.0 - 180.0


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial great-circle bearing, clockwise from North."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return wrap_deg(math.degrees(math.atan2(y, x)))


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_371_000.0
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dl / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(a))


def body_to_world(forward_m: float, right_m: float, yaw_deg: float) -> tuple[float, float]:
    """Body forward/right vector to East/North vector using compass yaw."""
    yaw = math.radians(yaw_deg)
    east = forward_m * math.sin(yaw) + right_m * math.cos(yaw)
    north = forward_m * math.cos(yaw) - right_m * math.sin(yaw)
    return east, north


def world_to_heading(east: float, north: float) -> float:
    return wrap_deg(math.degrees(math.atan2(east, north)))


def median(values: Iterable[float]) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    mid = len(ordered) // 2
    return ordered[mid] if len(ordered) % 2 else (ordered[mid - 1] + ordered[mid]) / 2

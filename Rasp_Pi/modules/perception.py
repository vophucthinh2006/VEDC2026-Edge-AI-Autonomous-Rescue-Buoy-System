"""LiDAR filtering, simple clustering, and camera-to-LiDAR target association."""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin

from modules.lidar_ld14 import LidarPoint
from utils.geometry import median, signed_angle_deg


@dataclass(frozen=True)
class Obstacle:
    bearing_body_deg: float  # positive right/clockwise from bow
    distance_m: float
    width_m: float


def cluster_scan(points: tuple[LidarPoint, ...], min_distance_m: float = 0.15, max_distance_m: float = 8.0, gap_m: float = 0.25) -> tuple[Obstacle, ...]:
    """Cluster adjacent polar points. Angle 0 is assumed aligned to the bow after installation calibration."""
    valid = sorted((p for p in points if min_distance_m <= p.distance_m <= max_distance_m and p.intensity > 0), key=lambda p: p.angle_deg)
    groups: list[list[LidarPoint]] = []
    for point in valid:
        if not groups:
            groups.append([point])
            continue
        previous = groups[-1][-1]
        angular_gap = abs(signed_angle_deg(point.angle_deg - previous.angle_deg))
        chord = (point.distance_m ** 2 + previous.distance_m ** 2 - 2 * point.distance_m * previous.distance_m * cos(radians(angular_gap))) ** 0.5
        if angular_gap < 4.0 and chord <= gap_m:
            groups[-1].append(point)
        else:
            groups.append([point])
    obstacles = []
    for group in groups:
        if len(group) < 2:
            continue
        angle = median(p.angle_deg for p in group)
        distance = median(p.distance_m for p in group)
        if angle is None or distance is None:
            continue
        first, last = group[0], group[-1]
        angular_width = abs(signed_angle_deg(last.angle_deg - first.angle_deg))
        width = max(0.05, 2 * distance * sin(radians(angular_width / 2)))
        obstacles.append(Obstacle(signed_angle_deg(angle), distance, width))
    return tuple(obstacles)


def associate_person(bearing_body_deg: float, obstacles: tuple[Obstacle, ...], half_angle_deg: float, min_distance_m: float, max_distance_m: float) -> Obstacle | None:
    candidates = [o for o in obstacles if abs(signed_angle_deg(o.bearing_body_deg - bearing_body_deg)) <= half_angle_deg and min_distance_m <= o.distance_m <= max_distance_m]
    return min(candidates, key=lambda obstacle: obstacle.distance_m) if candidates else None

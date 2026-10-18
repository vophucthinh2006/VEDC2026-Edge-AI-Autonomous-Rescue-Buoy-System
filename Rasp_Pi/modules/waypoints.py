"""The search route of the STM32 boat, kept on the Pi: the STM32F407 only holds a heading and a speed.

A mission file lists the waypoints (lat, lon) in order and, optionally, a geofence polygon:

    loop: true               # back to the first waypoint after the last; false: stop at the last
    waypoints:
      - [10.762622, 106.660172]
      - [10.762700, 106.660250]
    geofence:                # corners of the pool, in order; fewer than 3: no fence
      - [10.762600, 106.660150]
      ...

A Mission Planner file (.waypoints, "QGC WPL 110") is read too: its NAV_WAYPOINT items (command 16)
after the home row become the route. tools/set_waypoints.py writes the YAML form.
"""
from __future__ import annotations

from pathlib import Path

import yaml

from utils.geometry import haversine_m

NAV_WAYPOINT = 16


class Route:
    def __init__(self, waypoints: list[tuple[float, float]], arrive_radius_m: float, loop: bool = True,
                 geofence: list[tuple[float, float]] | None = None) -> None:
        self.waypoints = [(float(lat), float(lon)) for lat, lon in waypoints]
        self.arrive_radius_m = float(arrive_radius_m)
        self.loop = bool(loop)
        self.geofence = [(float(lat), float(lon)) for lat, lon in (geofence or [])]
        self.index = 0
        self.laps = 0

    @property
    def current(self) -> tuple[float, float] | None:
        """The waypoint being gone to; None with no route, or once a route without loop is done."""
        return self.waypoints[self.index] if self.index < len(self.waypoints) else None

    def update(self, lat: float, lon: float) -> bool:
        """Move on when the boat is inside arrive_radius_m of the current waypoint. True if it moved on."""
        goal = self.current
        if goal is None or haversine_m(lat, lon, goal[0], goal[1]) > self.arrive_radius_m:
            return False
        self.index += 1
        if self.index >= len(self.waypoints) and self.loop:
            self.index, self.laps = 0, self.laps + 1
        return True

    def inside_fence(self, lat: float, lon: float) -> bool:
        """Ray casting on the fence polygon (small enough to treat lat/lon as flat). No fence: always inside."""
        if len(self.geofence) < 3:
            return True
        inside = False
        for (lat1, lon1), (lat2, lon2) in zip(self.geofence, self.geofence[1:] + self.geofence[:1]):
            if (lat1 > lat) != (lat2 > lat) and lon < lon1 + (lat - lat1) * (lon2 - lon1) / (lat2 - lat1):
                inside = not inside
        return inside

    def describe(self) -> str:
        if not self.waypoints:
            return "no route"
        if self.current is None:
            return f"route done ({len(self.waypoints)} wp)"
        return f"wp {self.index + 1}/{len(self.waypoints)}" + (f" lap {self.laps + 1}" if self.loop else "")


def read_qgc_wpl(text: str) -> list[tuple[float, float]]:
    """NAV_WAYPOINT items of a Mission Planner .waypoints file, home (item 0) left out."""
    lines = [line.split() for line in text.splitlines() if line.strip()]
    if not lines or not lines[0][0].upper().startswith("QGC"):
        raise ValueError("not a QGC WPL file")
    points = []
    for fields in lines[1:]:
        if len(fields) >= 10 and int(fields[0]) > 0 and int(float(fields[3])) == NAV_WAYPOINT:
            points.append((float(fields[8]), float(fields[9])))
    return points


def load_route(path: Path, arrive_radius_m: float) -> Route:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".waypoints":
        return Route(read_qgc_wpl(text), arrive_radius_m)
    data = yaml.safe_load(text) or {}
    return Route([tuple(p) for p in data.get("waypoints") or []], arrive_radius_m, data.get("loop", True),
                 [tuple(p) for p in data.get("geofence") or []])

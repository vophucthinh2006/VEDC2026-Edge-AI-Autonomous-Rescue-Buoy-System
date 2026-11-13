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

from math import cos, hypot, radians
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

    def clamp_inside_fence(self, lat: float, lon: float, margin_m: float) -> tuple[float, float]:
        """The point itself when it is inside the fence (or there is no fence); else the nearest
        point on the fence, moved margin_m inside it: square off the edge, or toward the fence's
        middle where that does not land inside (a corner, a narrow end). The boat's way home when
        the shore station stands outside the pool."""
        if self.inside_fence(lat, lon):
            return lat, lon
        # Flat metres north and east of the point, which is small enough a fence for it.
        k_north, k_east = 111_320.0, 111_320.0 * cos(radians(lat))

        def to_latlon(north: float, east: float) -> tuple[float, float]:
            return lat + north / k_north, lon + east / k_east

        corners = [((c_lat - lat) * k_north, (c_lon - lon) * k_east) for c_lat, c_lon in self.geofence]
        best: tuple[float, tuple[float, float], tuple[float, float]] | None = None   # distance, point, edge direction
        for (n1, e1), (n2, e2) in zip(corners, corners[1:] + corners[:1]):
            dn, de = n2 - n1, e2 - e1
            length_sq = dn * dn + de * de
            t = 0.0 if length_sq == 0 else max(0.0, min(1.0, -(n1 * dn + e1 * de) / length_sq))
            point = (n1 + t * dn, e1 + t * de)
            if best is None or hypot(*point) < best[0]:
                best = (hypot(*point), point, (dn, de))
        assert best is not None
        _, (pn, pe), (dn, de) = best
        length = hypot(dn, de) or 1.0
        normal = (-de / length, dn / length)
        if not self.inside_fence(*to_latlon(pn + 0.01 * normal[0], pe + 0.01 * normal[1])):
            normal = (-normal[0], -normal[1])
        middle = (sum(n for n, _ in corners) / len(corners), sum(e for _, e in corners) / len(corners))
        to_middle = hypot(middle[0] - pn, middle[1] - pe) or 1.0
        for un, ue in (normal, ((middle[0] - pn) / to_middle, (middle[1] - pe) / to_middle)):
            candidate = to_latlon(pn + margin_m * un, pe + margin_m * ue)
            if self.inside_fence(*candidate):
                return candidate
        return to_latlon(*middle) if self.inside_fence(*to_latlon(*middle)) else to_latlon(pn, pe)

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


def start_route(path: Path | None, arrive_radius_m: float, waypoints_from_file: bool = True) -> Route:
    """The route at boot: the mission file's (no file or a missing one: none). With
    waypoints_from_file false (mission.waypoints_from_file, the STM32 boat) only the file's geofence
    is kept: the waypoints come from the dashboard alone."""
    route = load_route(path, arrive_radius_m) if path and path.exists() else Route([], arrive_radius_m)
    return route if waypoints_from_file else Route([], arrive_radius_m, route.loop, route.geofence)


def load_route(path: Path, arrive_radius_m: float) -> Route:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".waypoints":
        return Route(read_qgc_wpl(text), arrive_radius_m)
    data = yaml.safe_load(text) or {}
    return Route([tuple(p) for p in data.get("waypoints") or []], arrive_radius_m, data.get("loop", True),
                 [tuple(p) for p in data.get("geofence") or []])

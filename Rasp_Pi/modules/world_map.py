"""The decision map of the live viewer: the boat on a north-up map, with what it is deciding on.

    route and current waypoint   where the mission wants the boat (blue)
    thin blue arrow              the straight bearing to that waypoint
    thick yellow arrow           where the boat is steered right now (by the autopilot, or by the
                                 Pi's navigator on the STM32 boat). When the two differ, avoidance
                                 is bending the path round what the LiDAR sees: that difference is
                                 the decision this map is there to show.
    white dots                   LiDAR returns, remembered where they were seen
    red arcs                     the sectors the Pi reports as blocked
    people                       grey: seen, not confirmed. yellow: confirmed, waiting. green: the
                                 one being gone to (with the stand-off ring and the point aimed at).
                                 blue: attended, with the ring nobody inside is gone to again.
    text                         what the Pi is doing and why, in words

Drawn from the same state the rescue logic reads; it changes nothing.
"""
from __future__ import annotations

import math
from collections import deque
from time import monotonic

import cv2
import numpy as np

from modules.obstacle_feed import SECTOR_DEG
from modules.state_store import Snapshot
from modules.ui_draw import BLUE, CARD, CARD_EDGE, CYAN, GREEN, GRID, MUTED, ORANGE, RED, ROUTE, STRIP, TEXT, WHITE, YELLOW, put
from utils.geometry import signed_angle_deg

WORLD_W, WORLD_H = 1096, 480   # as wide as the camera and LiDAR cards above it, with their gap
PANEL_W = 380                  # the words, on the right
MAP_W = WORLD_W - PANEL_W
GREY, DARK = MUTED, GRID
HIT_KEEP_S = 180.0             # a LiDAR return stays on the map this long after it was last seen
HIT_CELL_M = 0.25
MAX_HITS = 8000
MIN_SPAN_M = 30.0
MARGIN_M = 6.0


class WorldMemory:
    """What the map has to remember between frames: where the boat has been and where the LiDAR
    has seen something, both in metres east and north of the first position fixed."""

    def __init__(self) -> None:
        self.origin: tuple[float, float] | None = None
        self.trail: deque[tuple[float, float]] = deque(maxlen=4000)
        self.hits: dict[tuple[int, int], float] = {}

    def to_local(self, lat: float, lon: float) -> tuple[float, float]:
        lat0, lon0 = self.origin
        return (lon - lon0) * 111_320.0 * math.cos(math.radians(lat0)), (lat - lat0) * 111_320.0

    def update(self, snap: Snapshot, now: float) -> tuple[float, float] | None:
        """Take in a snapshot. Returns the boat's place, or None while there is no position."""
        if snap.gps.fix < 3:
            return None
        if self.origin is None:
            self.origin = (snap.gps.lat_deg, snap.gps.lon_deg)
        boat = self.to_local(snap.gps.lat_deg, snap.gps.lon_deg)
        if not self.trail or math.hypot(boat[0] - self.trail[-1][0], boat[1] - self.trail[-1][1]) >= 0.3:
            self.trail.append(boat)
        for obstacle in snap.obstacles:
            bearing = math.radians(snap.imu.yaw_deg + obstacle.bearing_body_deg)
            cell = (round((boat[0] + obstacle.distance_m * math.sin(bearing)) / HIT_CELL_M),
                    round((boat[1] + obstacle.distance_m * math.cos(bearing)) / HIT_CELL_M))
            self.hits[cell] = now
        if len(self.hits) > MAX_HITS:
            for cell, seen in sorted(self.hits.items(), key=lambda item: item[1])[:len(self.hits) - MAX_HITS]:
                del self.hits[cell]
        for cell in [cell for cell, seen in self.hits.items() if now - seen > HIT_KEEP_S]:
            del self.hits[cell]
        return boat


def steering_words(snap: Snapshot, now: float) -> list[tuple[str, tuple[int, int, int]]]:
    """In words: is the boat going straight for its waypoint, or round something?"""
    nav, vehicle = snap.nav, snap.vehicle
    if nav.steer_bearing_deg is None or now - nav.timestamp > 3.0:
        return [("no steering report yet", GREY)]
    lines = [(f"steering {nav.steer_bearing_deg:03.0f} deg", YELLOW)]
    if vehicle.wp_dist_m >= 0.0 and vehicle.mode in ("AUTO", "GUIDED", "RTL"):
        off = signed_angle_deg(nav.steer_bearing_deg - vehicle.wp_bearing_deg)
        if abs(off) < 8.0:
            lines.append((f"= straight for the waypoint ({vehicle.wp_bearing_deg:03.0f})", WHITE))
        else:
            lines.append((f"= {abs(off):.0f} deg {'RIGHT' if off > 0 else 'LEFT'} of the waypoint ({vehicle.wp_bearing_deg:03.0f})", ORANGE))
            ahead = [o for o in snap.obstacles if abs(signed_angle_deg(snap.imu.yaw_deg + o.bearing_body_deg - vehicle.wp_bearing_deg)) <= 30.0]
            if ahead:
                nearest = min(ahead, key=lambda o: o.distance_m)
                lines.append((f"  going round: {nearest.distance_m:.1f} m on the way there", ORANGE))
            else:
                lines.append(("  nothing on the line: still turning onto it", GREY))
    return lines


def draw_world(snap: Snapshot, memory: WorldMemory, decision: dict | None, sector_cm: list[int], max_m: float,
               view: dict | None, fov_deg: float) -> np.ndarray:
    """decision: RescueMission.describe() plus "goto" (lat, lon) or None; None without rescue logic."""
    now = monotonic()
    panel = np.full((WORLD_H, WORLD_W, 3), CARD, np.uint8)
    boat = memory.update(snap, now)
    decision = decision or {}
    clear = int(max_m * 100) + 1
    blocked = [index for index, cm in enumerate(sector_cm) if cm < clear]

    if boat is None:
        words_column(panel, snap, decision, blocked, now)
        put(panel, "waiting for a GPS fix", (MAP_W // 2, WORLD_H // 2), 18, GREY, align="centre")
        return panel

    people = [(memory.to_local(lat, lon), status) for lat, lon, status in decision.get("people", [])]
    route = [(seq, memory.to_local(lat, lon)) for seq, lat, lon in snap.nav.route]
    goto = memory.to_local(*decision["goto"]) if decision.get("goto") else None

    # The map shows everything that matters: where the boat has been, the route, the people.
    xs = [p[0] for p in memory.trail] + [p[0] for _s, p in route] + [p[0] for p, _s in people] + [boat[0]]
    ys = [p[1] for p in memory.trail] + [p[1] for _s, p in route] + [p[1] for p, _s in people] + [boat[1]]
    span_x, span_y = max(max(xs) - min(xs) + 2 * MARGIN_M, MIN_SPAN_M), max(max(ys) - min(ys) + 2 * MARGIN_M, MIN_SPAN_M)
    scale = min(MAP_W / span_x, WORLD_H / span_y)                     # pixels per metre
    centre = ((max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0)

    def px(point: tuple[float, float]) -> tuple[int, int]:
        return int(MAP_W / 2 + (point[0] - centre[0]) * scale), int(WORLD_H / 2 - (point[1] - centre[1]) * scale)

    def ahead(start: tuple[float, float], bearing_deg: float, metres: float) -> tuple[float, float]:
        b = math.radians(bearing_deg)
        return start[0] + metres * math.sin(b), start[1] + metres * math.cos(b)

    # Grid: 5 m, or 10 m once the map covers a lot.
    step = 5.0 if scale >= 6.0 else 10.0 if scale >= 3.0 else 20.0
    half_x, half_y = MAP_W / 2 / scale, WORLD_H / 2 / scale
    gx = math.floor((centre[0] - half_x) / step) * step
    while gx <= centre[0] + half_x:
        cv2.line(panel, px((gx, centre[1] - half_y)), px((gx, centre[1] + half_y)), DARK, 1)
        gx += step
    gy = math.floor((centre[1] - half_y) / step) * step
    while gy <= centre[1] + half_y:
        cv2.line(panel, px((centre[0] - half_x, gy)), px((centre[0] + half_x, gy)), DARK, 1)
        gy += step
    cv2.line(panel, (16, WORLD_H - 18), (16 + int(step * scale), WORLD_H - 18), GREY, 2)
    put(panel, f"{step:.0f} m", (16, WORLD_H - 26), 12, GREY)
    cv2.arrowedLine(panel, (MAP_W - 28, 48), (MAP_W - 28, 16), GREY, 2, cv2.LINE_AA, tipLength=0.35)
    put(panel, "N", (MAP_W - 28, 66), 13, GREY, bold=True, align="centre")

    # What the LiDAR has seen, where it saw it: fresh white, older grey.
    for (cx, cy), seen in memory.hits.items():
        point = px((cx * HIT_CELL_M, cy * HIT_CELL_M))
        if 0 <= point[0] < MAP_W and 0 <= point[1] < WORLD_H:
            shade = 235 if now - seen < 2.0 else 120
            cv2.circle(panel, point, 1, (shade, shade, shade), -1)

    # The mission: its waypoints in order, the current one marked.
    if route:
        ordered = [p for _s, p in sorted(route)]
        if len(ordered) > 1:
            cv2.polylines(panel, [np.array([px(p) for p in ordered], np.int32)], False, ROUTE, 1, cv2.LINE_AA)
        for seq, point in route:
            current = seq == snap.vehicle.mission_seq
            cv2.circle(panel, px(point), 7 if current else 4, ROUTE, -1 if current else 1, cv2.LINE_AA)
            put(panel, str(seq), (px(point)[0] + 9, px(point)[1] - 7), 13, ROUTE, bold=True)

    # Where the boat has been.
    if len(memory.trail) > 1:
        cv2.polylines(panel, [np.array([px(p) for p in memory.trail], np.int32)], False, CYAN, 2, cv2.LINE_AA)

    # People, as the rescue logic has them.
    group_radius, standoff = float(decision.get("group_radius_m", 0.0)), float(decision.get("standoff_m", 0.0))
    for point, status in people:
        colour = {"seen": GREY, "waiting": YELLOW, "target": GREEN, "attended": BLUE, "lost": ORANGE}.get(status, GREY)
        cv2.circle(panel, px(point), 5, colour, -1, cv2.LINE_AA)
        if status == "attended" and group_radius:
            cv2.circle(panel, px(point), int(group_radius * scale), colour, 1, cv2.LINE_AA)
        elif status == "target" and standoff:
            cv2.circle(panel, px(point), int(standoff * scale), colour, 1, cv2.LINE_AA)
    if goto is not None:
        cv2.drawMarker(panel, px(goto), GREEN, cv2.MARKER_TILTED_CROSS, 12, 2, cv2.LINE_AA)
        cv2.line(panel, px(boat), px(goto), GREEN, 1, cv2.LINE_AA)

    # The camera's field of view and the sectors reported as blocked, turned to the map.
    yaw = snap.imu.yaw_deg
    if view is not None:
        pan, half = float(view.get("pan_deg", 0.0)), fov_deg / 2.0
        wedge = np.array([px(boat)] + [px(ahead(boat, yaw + pan + a, max_m)) for a in np.linspace(-half, half, 9)], np.int32)
        overlay = panel.copy()
        cv2.fillPoly(overlay, [wedge], YELLOW if view.get("scanning") else GREEN)
        panel = cv2.addWeighted(overlay, 0.12, panel, 0.88, 0)
    for index, cm in enumerate(sector_cm):
        if cm < clear:
            a0, a1 = yaw + index * SECTOR_DEG - SECTOR_DEG / 2.0, yaw + index * SECTOR_DEG + SECTOR_DEG / 2.0
            arc = np.array([px(ahead(boat, a, cm / 100.0)) for a in np.linspace(a0, a1, 4)], np.int32)
            cv2.polylines(panel, [arc], False, RED, 2, cv2.LINE_AA)

    # The two arrows: straight to the waypoint, and where the autopilot actually steers.
    arrow_m = max(4.0, 60.0 / scale)
    if snap.vehicle.wp_dist_m >= 0.0:
        cv2.arrowedLine(panel, px(boat), px(ahead(boat, snap.vehicle.wp_bearing_deg, arrow_m)), ROUTE, 1, cv2.LINE_AA, tipLength=0.2)
    if snap.nav.steer_bearing_deg is not None and now - snap.nav.timestamp <= 3.0 and snap.vehicle.armed:
        cv2.arrowedLine(panel, px(boat), px(ahead(boat, snap.nav.steer_bearing_deg, arrow_m)), YELLOW, 3, cv2.LINE_AA, tipLength=0.25)

    # The boat itself, bow first.
    b = px(boat)
    nose, left, right = px(ahead(boat, yaw, 1.0)), px(ahead(boat, yaw + 140.0, 0.8)), px(ahead(boat, yaw - 140.0, 0.8))
    if math.dist(nose, b) < 9:              # zoomed out: keep the arrow big enough to see
        nose, left, right = px(ahead(boat, yaw, 9 / scale)), px(ahead(boat, yaw + 140.0, 7 / scale)), px(ahead(boat, yaw - 140.0, 7 / scale))
    cv2.fillPoly(panel, [np.array([nose, left, b, right], np.int32)], WHITE, cv2.LINE_AA)
    words_column(panel, snap, decision, blocked, now)
    return panel


def words_column(panel: np.ndarray, snap: Snapshot, decision: dict, blocked: list[int], now: float) -> None:
    """The right-hand column: what the boat is doing and why, how it steers, the legend."""
    cv2.rectangle(panel, (MAP_W, 0), (WORLD_W - 1, WORLD_H - 1), STRIP, -1)        # over whatever ran into it
    cv2.line(panel, (MAP_W, 0), (MAP_W, WORLD_H - 1), CARD_EDGE, 1)
    x, y = MAP_W + 18, 30

    def heading(text: str) -> None:
        nonlocal y
        put(panel, text, (x, y), 11, GREY, bold=True)
        y += 22

    def line(text: str, colour, size: int = 14, bold: bool = False) -> None:
        nonlocal y
        put(panel, text, (x, y), size, colour, bold=bold)
        y += size + 8

    heading("WHAT THE BOAT IS DOING")
    lines = decision.get("lines") or ["no rescue logic here"]
    for index, text in enumerate(lines[:5]):
        line(text, GREEN if index == 0 else TEXT, 15 if index == 0 else 13, bold=index == 0)
    y += 8
    heading("STEERING")
    for text, colour in steering_words(snap, now):
        line(text.strip(), colour, 13)
    nearest = min((o.distance_m for o in snap.obstacles), default=None)
    line(f"LiDAR: {len(blocked)} of 72 sectors blocked" + (f", nearest {nearest:.1f} m" if nearest is not None else ""),
         RED if blocked else GREY, 13)
    legend = ((WHITE, "boat, LiDAR returns"), (CYAN, "where the boat has been"), (ROUTE, "route, bearing to waypoint"),
              (YELLOW, "steering, waiting people"), (GREEN, "person gone to"), (BLUE, "attended person"), (RED, "blocked sectors"))
    y = max(y + 6, WORLD_H - 20 * ((len(legend) + 1) // 2) - 30)
    put(panel, "LEGEND", (x, y), 11, GREY, bold=True)
    y += 20
    column = (WORLD_W - MAP_W - 36) // 2
    for index, (colour, meaning) in enumerate(legend):
        lx, ly = x + (index % 2) * column, y + (index // 2) * 20
        cv2.circle(panel, (lx + 4, ly - 4), 4, colour, -1, cv2.LINE_AA)
        put(panel, meaning, (lx + 14, ly), 11, GREY)

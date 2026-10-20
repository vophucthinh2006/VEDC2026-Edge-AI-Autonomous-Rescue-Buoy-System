"""Live view of what the Pi perceives, in a browser: http://<host>:<port>/

Across the top, the mode, arming, phase and the boat's heading, speed and nearest obstacle. Under
it, in cards: the camera frame with the detector's box, confidence and range, and where the servo
points; the LiDAR from above, bow up: the returns, the 72 sectors as reported, the keep-out
circles, the camera's field of view and the person being followed. Below, the decision map
(modules/world_map.py): north up, the mission route, where the boat is steered against where the
waypoint is, the people as the rescue logic has them, and in words what the boat is doing and why.

It reads the same state the rescue logic reads and changes nothing. One MJPEG stream, a few frames
per second: for watching and for recording a demo, not for control.
"""
from __future__ import annotations

import logging
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from time import monotonic, sleep
from typing import Callable

import cv2
import numpy as np

from modules.edge_model import EDGES, RISK_NAMES
from modules.obstacle_feed import SECTOR_DEG, objects_from, sectors
from modules.state_store import Snapshot, StateStore
from modules.ui_draw import (BG, BLUE, CARD, CARD_EDGE, CYAN, DIM, GREEN, GRID, GUTTER, MUTED, ORANGE, RED, TEXT, WHITE, YELLOW,
                             card, chip, put, tag, translucent)
from modules.world_map import WorldMemory, draw_world

CAMERA_W, CAMERA_H = 640, 480
MAP = 444                      # the LiDAR panel is MAP x CAMERA_H pixels
HEADER = 52                    # title, status chips and readouts across the top
RISK_COLOURS = (GREEN, YELLOW, ORANGE, RED)   # SAFE, MEDIUM, HIGH, CRITICAL (modules/edge_model.py)

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>RedgeSCUE - Pi view</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;background:#0f1114;color:#c8c8c8;font:14px system-ui,sans-serif;text-align:center}
img{max-width:100%;height:auto;margin:12px auto 4px;display:block}p{margin:6px 16px;color:#8c8c8c}
b{color:#ddd;font-weight:600}</style></head>
<body><img src="/stream" alt="camera, LiDAR and map as the Pi sees them">
<p><b>Camera</b>: the detector's box, confidence and range &middot; <b>LiDAR</b> from above, bow up: white = returns,
red = blocked sectors, orange = keep-out circles, wedge = camera (yellow searching, green following)</p>
<p><b>Map</b>, north up: thin blue arrow = straight to the waypoint, thick yellow arrow = where the boat is steered.
When they differ the boat is going round something.</p></body></html>"""


def draw_camera(view: dict | None) -> np.ndarray:
    """view: frame (BGR) or None, box (ymin, xmin, ymax, xmax in 0..1) or None, confidence,
    distance_m, pan_deg, scanning, age_s; with the YOLO + MoveNet model also keypoints, in_water
    and risk of the person followed, boats and on_boat (boxes), model (its name). others: the
    other people, (box, confidence, distance_m[, risk])."""
    panel = np.full((CAMERA_H, CAMERA_W, 3), CARD, np.uint8)
    if not view or view.get("frame") is None:
        put(panel, "no camera frame" if view is None else view.get("note", "no camera frame"), (CAMERA_W // 2, CAMERA_H // 2), 18, MUTED,
            align="centre")
        return panel
    panel = cv2.resize(view["frame"], (CAMERA_W, CAMERA_H)).copy()
    # Horizon of a level boat and the centre of the lens, faint.
    cv2.line(panel, (0, CAMERA_H // 2), (CAMERA_W, CAMERA_H // 2), DIM, 1, cv2.LINE_AA)
    cv2.line(panel, (CAMERA_W // 2, CAMERA_H // 2 - 10), (CAMERA_W // 2, CAMERA_H // 2 + 10), DIM, 1, cv2.LINE_AA)
    fresh = view.get("age_s", 0.0) < 1.0
    if fresh:
        for other in view.get("others", []):      # people in view the camera is not following
            (ymin, xmin, ymax, xmax), confidence, distance = other[:3]
            risk = other[3] if len(other) > 3 else None
            colour = ORANGE if risk is None else RISK_COLOURS[risk]
            p1 = (int(xmin * CAMERA_W), int(ymin * CAMERA_H))
            cv2.rectangle(panel, p1, (int(xmax * CAMERA_W), int(ymax * CAMERA_H)), colour, 1, cv2.LINE_AA)
            tag(panel, ("person" if risk is None else RISK_NAMES[risk]) + f" {confidence:.2f}" + (f" · {distance:.1f} m" if distance else ""),
                (p1[0], p1[1] - 22), colour, 12, filled=False)
        for kind, boxes in (("boat", view.get("boats", [])), ("on boat · SAFE", view.get("on_boat", []))):
            for ymin, xmin, ymax, xmax in boxes:
                p1 = (int(xmin * CAMERA_W), int(ymin * CAMERA_H))
                cv2.rectangle(panel, p1, (int(xmax * CAMERA_W), int(ymax * CAMERA_H)), BLUE, 1, cv2.LINE_AA)
                tag(panel, kind, (p1[0], p1[1] - 22), BLUE, 12, filled=False)
    box = view.get("box")
    if box is not None and fresh:
        ymin, xmin, ymax, xmax = box
        p1, p2 = (int(xmin * CAMERA_W), int(ymin * CAMERA_H)), (int(xmax * CAMERA_W), int(ymax * CAMERA_H))
        keypoints = view.get("keypoints") or ()
        bone = RED if view.get("in_water") else YELLOW
        for a, b in EDGES if keypoints else ():
            if keypoints[a][2] > 0.25 and keypoints[b][2] > 0.25:
                cv2.line(panel, (int(keypoints[a][0] * CAMERA_W), int(keypoints[a][1] * CAMERA_H)),
                         (int(keypoints[b][0] * CAMERA_W), int(keypoints[b][1] * CAMERA_H)), bone, 2, cv2.LINE_AA)
        for x, y, score in keypoints:
            if score > 0.25:
                cv2.circle(panel, (int(x * CAMERA_W), int(y * CAMERA_H)), 3, YELLOW, -1, cv2.LINE_AA)
        # A thin box with heavy corners: the person stays visible inside it.
        cv2.rectangle(panel, p1, p2, GREEN, 1, cv2.LINE_AA)
        arm = max(6, min(18, (p2[0] - p1[0]) // 3, (p2[1] - p1[1]) // 3))
        for (x, y), (dx, dy) in (((p1[0], p1[1]), (1, 1)), ((p2[0], p1[1]), (-1, 1)), ((p1[0], p2[1]), (1, -1)), ((p2[0], p2[1]), (-1, -1))):
            cv2.line(panel, (x, y), (x + dx * arm, y), GREEN, 3, cv2.LINE_AA)
            cv2.line(panel, (x, y), (x, y + dy * arm), GREEN, 3, cv2.LINE_AA)
        distance = view.get("distance_m")
        label = ("PERSON" if view.get("risk") is None else RISK_NAMES[view["risk"]]) + f" {view.get('confidence', 0.0):.2f}" + (f"  ·  {distance:.1f} m" if distance else "  ·  range ?")
        tag(panel, label, (p1[0], p1[1] - 26), GREEN)
        if view.get("risk") is not None and view.get("in_water") is not None:
            tag(panel, RISK_NAMES[view["risk"]] + (" · IN WATER" if view["in_water"] else " · BODY VISIBLE"), (p1[0], p2[1] + 5),
                RISK_COLOURS[view["risk"]], 12)
        elif view.get("in_water") is not None:
            tag(panel, "IN WATER" if view["in_water"] else "BODY VISIBLE", (p1[0], p2[1] + 5), RED if view["in_water"] else YELLOW, 12)
    return panel


def camera_meta(view: dict | None) -> tuple[str, tuple[int, int, int]]:
    """The camera card's right-hand note: where the servo points and what it is doing."""
    if not view:
        return "", MUTED
    pan = f"pan {view.get('pan_deg', 0.0):+.0f}°"
    if view.get("scanning"):
        return f"{pan}  ·  searching", YELLOW
    if view.get("box") is not None and view.get("age_s", 9) < 1.0:
        return f"{pan}  ·  following", GREEN
    return f"{pan}  ·  ahead", TEXT


def draw_map(snap: Snapshot, sector_cm: list[int], max_m: float, keep_outs: list[tuple[float, float, float]],
             view: dict | None, fov_deg: float) -> np.ndarray:
    """keep_outs: (bearing from the bow in degrees, distance in m, radius in m)."""
    panel = np.full((CAMERA_H, MAP, 3), CARD, np.uint8)
    cx, cy = MAP // 2, CAMERA_H // 2 + 8
    scale = (min(MAP, CAMERA_H) / 2 - 28) / max_m          # pixels per metre

    def point(bearing_deg: float, distance_m: float) -> tuple[int, int]:
        b = math.radians(bearing_deg)
        return int(cx + distance_m * math.sin(b) * scale), int(cy - distance_m * math.cos(b) * scale)

    # Spokes every 30 deg and a ring every 2 m, the outer one a little brighter.
    for angle in range(0, 360, 30):
        cv2.line(panel, (cx, cy), point(angle, max_m), GRID, 1, cv2.LINE_AA)
    for ring in range(2, int(max_m) + 1, 2):
        cv2.circle(panel, (cx, cy), int(ring * scale), DIM if ring == int(max_m) else GRID, 1, cv2.LINE_AA)
        x, y = point(45.0, ring)
        put(panel, f"{ring} m", (x + 4, y - 3), 11, MUTED)
    # The bow mark above the outer ring.
    top = point(0.0, max_m)
    cv2.fillPoly(panel, [np.array([(top[0], top[1] - 3), (top[0] - 6, top[1] - 13), (top[0] + 6, top[1] - 13)], np.int32)], MUTED)
    put(panel, "BOW", (top[0] + 10, top[1] - 4), 11, MUTED, bold=True)
    # The camera's field of view.
    if view is not None:
        pan, half = float(view.get("pan_deg", 0.0)), fov_deg / 2.0
        colour = YELLOW if view.get("scanning") else GREEN
        wedge = np.array([(cx, cy)] + [point(pan + a, max_m) for a in np.linspace(-half, half, 17)], np.int32)
        panel = translucent(panel, wedge, colour, 0.16)
        cv2.polylines(panel, [wedge], True, colour, 1, cv2.LINE_AA)
    # Sectors as reported: an arc at the reported distance.
    clear = int(max_m * 100) + 1
    for index, cm in enumerate(sector_cm):
        if cm < clear:
            a0, a1 = index * SECTOR_DEG - SECTOR_DEG / 2.0, index * SECTOR_DEG + SECTOR_DEG / 2.0
            arc = np.array([point(a, cm / 100.0) for a in np.linspace(a0, a1, 5)], np.int32)
            cv2.polylines(panel, [arc], False, RED, 3, cv2.LINE_AA)
    for o in snap.obstacles:                                   # raw LiDAR returns
        cv2.circle(panel, point(o.bearing_body_deg, o.distance_m), 2, WHITE, -1, cv2.LINE_AA)
    for bearing, distance, radius in keep_outs:
        if distance - radius <= max_m:
            cv2.circle(panel, point(bearing, distance), max(2, int(radius * scale)), ORANGE, 2, cv2.LINE_AA)
    target = snap.human_target
    for other in snap.human_targets:
        if other is not target and monotonic() - other.timestamp < 1.0:
            cv2.drawMarker(panel, point(other.bearing_body_deg, min(other.distance_m, max_m)), ORANGE, cv2.MARKER_TILTED_CROSS, 10, 1, cv2.LINE_AA)
    if target is not None and monotonic() - target.timestamp < 1.0:
        p = point(target.bearing_body_deg, min(target.distance_m, max_m))
        cv2.circle(panel, p, 9, GREEN, 2, cv2.LINE_AA)
        cv2.drawMarker(panel, p, GREEN, cv2.MARKER_TILTED_CROSS, 10, 2, cv2.LINE_AA)
        tag(panel, f"{target.distance_m:.1f} m", (p[0] + 12, p[1] - 10), GREEN, 12)
    # The boat, 1.10 x 0.60 m, bow up, drawn a little larger than to scale so it reads.
    half_l, half_w = max(10, int(0.55 * scale)), max(6, int(0.30 * scale))
    hull = np.array([(cx, cy - half_l - 5), (cx + half_w, cy - half_l // 3), (cx + half_w, cy + half_l),
                     (cx - half_w, cy + half_l), (cx - half_w, cy - half_l // 3)], np.int32)
    cv2.fillPoly(panel, [hull], CYAN, cv2.LINE_AA)
    cv2.polylines(panel, [hull], True, WHITE, 1, cv2.LINE_AA)
    return panel


def lidar_meta(snap: Snapshot, sector_cm: list[int], max_m: float) -> tuple[str, tuple[int, int, int]]:
    """The LiDAR card's right-hand note."""
    blocked = sum(1 for cm in sector_cm if cm < int(max_m * 100) + 1)
    return f"{len(snap.obstacles)} returns  ·  {blocked}/72 blocked", RED if blocked else MUTED


def draw_header(snap: Snapshot, phase: str, width: int) -> np.ndarray:
    """Name on the left; on the right the readouts, then mode, arming and phase as chips."""
    bar = np.full((HEADER, width, 3), BG, np.uint8)
    x = GUTTER + put(bar, "RedgeSCUE", (GUTTER, 34), 21, WHITE, bold=True)
    put(bar, "Pi live view", (x + 10, 34), 15, MUTED)
    vehicle = snap.vehicle
    right = width - GUTTER
    phase = phase if len(phase) <= 30 else phase[:29] + "…"
    if phase:
        right = chip(bar, phase.upper(), right, HEADER // 2, CYAN) - 8
    right = chip(bar, "ARMED" if vehicle.armed else "DISARMED", right, HEADER // 2, ORANGE if vehicle.armed else MUTED) - 8
    mode = vehicle.mode or "?"
    right = chip(bar, mode, right, HEADER // 2, GREEN if mode == "AUTO" else YELLOW if mode in ("GUIDED", "LOITER", "HOLD") else TEXT) - 18
    nearest = min((o.distance_m for o in snap.obstacles), default=None)
    readout = (f"HDG {snap.imu.yaw_deg:03.0f}°   {snap.gps.speed_mps:.1f} m/s   "
               + (f"obstacle {nearest:.1f} m" if nearest is not None else "no obstacle")
               + (f"   wp {vehicle.wp_dist_m:.0f} m" if vehicle.wp_dist_m >= 0 else ""))
    put(bar, readout, (right, 32), 14, TEXT, align="right")
    cv2.line(bar, (0, HEADER - 1), (width, HEADER - 1), CARD_EDGE, 1)
    return bar


def render(snap: Snapshot, view: dict | None, sector_cm: list[int], max_m: float,
           keep_outs: list[tuple[float, float, float]], fov_deg: float, phase: str, world: np.ndarray | None = None) -> np.ndarray:
    """world: the decision map, as wide as the two panels above it with the gap between them
    (world_map.WORLD_W); None leaves it out."""
    model = (view or {}).get("model") or ""
    camera = card(draw_camera(view), "Camera" + (f"  ·  {model}" if model else ""), *camera_meta(view))
    lidar = card(draw_map(snap, sector_cm, max_m, keep_outs, view, fov_deg), "LiDAR  ·  from above", *lidar_meta(snap, sector_cm, max_m))
    rows = [np.hstack([camera, np.full((camera.shape[0], GUTTER, 3), BG, np.uint8), lidar])]
    if world is not None:
        rows.append(card(world, "Decision map  ·  north up"))
    width = rows[0].shape[1] + 2 * GUTTER
    parts = [draw_header(snap, phase, width)]
    for row in rows:
        side = np.full((row.shape[0], GUTTER, 3), BG, np.uint8)
        parts += [np.full((GUTTER, width, 3), BG, np.uint8), np.hstack([side, row, side])]
    parts.append(np.full((GUTTER, width, 3), BG, np.uint8))
    return np.vstack(parts)


class Viewer(threading.Thread):
    def __init__(self, port: int, state: StateStore, stop_event: threading.Event, avoidance: dict, camera: dict,
                 camera_view: Callable[[], dict | None], keep_outs: Callable[[], list[tuple[float, float]]],
                 attending: Callable[[], tuple[float, float] | None], phase: Callable[[], str], rate_hz: float = 5.0,
                 decision: Callable[[], dict | None] | None = None) -> None:
        super().__init__(name="viewer", daemon=True)
        self.port, self.state, self.stop = port, state, stop_event
        self.a, self.fov = avoidance, float(camera["horizontal_fov_deg"])
        self.camera_view, self.keep_outs, self.attending, self.phase = camera_view, keep_outs, attending, phase
        self.period = 1.0 / rate_hz
        self.decision = decision                # RescueMission.describe() and the point aimed at; None: no rescue logic
        self.memory = WorldMemory()
        self.log = logging.getLogger(__name__)
        self._jpeg = b""
        self._fresh = threading.Condition()

    def _frame(self) -> np.ndarray:
        snap = self.state.snapshot()
        min_m, max_m, radius = float(self.a["min_range_m"]), float(self.a["max_range_m"]), float(self.a["keep_out_radius_m"])
        circles = list(self.keep_outs())
        objects = objects_from(snap, circles, radius, self.attending())
        # objects_from appends one circle per keep-out after the LiDAR returns.
        keep = objects[len(objects) - len(circles):] if circles and snap.gps.fix >= 3 else []
        view, sector_cm = self.camera_view(), sectors(objects, min_m, max_m)
        world = draw_world(snap, self.memory, self.decision() if self.decision else None, sector_cm, max_m, view, self.fov)
        return render(snap, view, sector_cm, max_m, keep, self.fov, self.phase(), world)

    def run(self) -> None:
        viewer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args: object) -> None:      # keep the Pi's log for the rescue logic
                return

            def do_GET(self) -> None:  # noqa: N802 - http.server name
                if self.path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(PAGE)))
                    self.end_headers()
                    self.wfile.write(PAGE)
                elif self.path == "/frame.jpg":
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(viewer._jpeg)))
                    self.end_headers()
                    self.wfile.write(viewer._jpeg)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    try:
                        while not viewer.stop.is_set():
                            with viewer._fresh:
                                viewer._fresh.wait(timeout=2.0)
                                jpeg = viewer._jpeg
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                    except (BrokenPipeError, ConnectionError):
                        return
                else:
                    self.send_error(404)

        try:
            server = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        except OSError as exc:
            self.log.error("viewer: cannot listen on port %d: %s", self.port, exc)
            return
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, name="viewer-http", daemon=True).start()
        self.log.info("viewer: http://127.0.0.1:%d/ (any address of this machine)", self.port)
        try:
            while not self.stop.is_set():
                started = monotonic()
                try:
                    ok, encoded = cv2.imencode(".jpg", self._frame(), [cv2.IMWRITE_JPEG_QUALITY, 85])
                except Exception:  # noqa: BLE001 - a drawing error must never take the rescue program down
                    self.log.exception("viewer: frame failed")
                    ok = False
                if ok:
                    with self._fresh:
                        self._jpeg = encoded.tobytes()
                        self._fresh.notify_all()
                sleep(max(0.0, self.period - (monotonic() - started)))
        finally:
            server.shutdown()

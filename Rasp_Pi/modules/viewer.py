"""Live view of what the Pi perceives, in a browser: http://<host>:<port>/

Left, the camera frame with the detector's box, confidence and range, and where the servo points.
Right, the LiDAR from above, bow up: the returns, the 72 sectors as sent to the autopilot, the
keep-out circles, the camera's field of view and the person being followed.

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

from modules.obstacle_feed import SECTOR_DEG, SECTORS, objects_from, sectors
from modules.state_store import Snapshot, StateStore

CAMERA_W, CAMERA_H = 640, 480
MAP = 480                      # the LiDAR panel is MAP x MAP pixels
BAR = 44                       # status line under both panels
WHITE, GREY, DIM = (235, 235, 235), (150, 150, 150), (70, 70, 70)
GREEN, YELLOW, RED, BLUE, ORANGE = (90, 220, 90), (60, 220, 240), (70, 70, 240), (240, 160, 60), (40, 150, 250)
FONT = cv2.FONT_HERSHEY_SIMPLEX

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>RedgeSCUE - Pi view</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;background:#111;color:#ddd;font:14px system-ui,sans-serif;text-align:center}
img{max-width:100%;height:auto;margin-top:8px}p{margin:8px}</style></head>
<body><img src="/stream" alt="camera and LiDAR as the Pi sees them">
<p>Camera with the detector's box &middot; LiDAR from above, bow up: white = returns, red = sectors sent to the autopilot,
orange = keep-out circles, yellow/green wedge = camera searching/following</p></body></html>"""


def _text(image: np.ndarray, text: str, origin: tuple[int, int], colour=WHITE, scale: float = 0.5) -> None:
    """Text on a dark plate, so it reads on any camera image."""
    (width, height), baseline = cv2.getTextSize(text, FONT, scale, 1)
    x, y = origin
    cv2.rectangle(image, (x - 3, y - height - 4), (x + width + 3, y + baseline + 2), (16, 16, 16), -1)
    cv2.putText(image, text, origin, FONT, scale, colour, 1, cv2.LINE_AA)


def draw_camera(view: dict | None) -> np.ndarray:
    """view: frame (BGR) or None, box (ymin, xmin, ymax, xmax in 0..1) or None, confidence,
    distance_m, pan_deg, scanning, age_s."""
    panel = np.full((CAMERA_H, CAMERA_W, 3), 40, np.uint8)
    if not view or view.get("frame") is None:
        _text(panel, "no camera frame" if view is None else view.get("note", "no camera frame"), (20, 40), GREY, 0.6)
    else:
        panel = cv2.resize(view["frame"], (CAMERA_W, CAMERA_H)).copy()
        box = view.get("box")
        if box is not None and view.get("age_s", 0.0) < 1.0:
            ymin, xmin, ymax, xmax = box
            p1, p2 = (int(xmin * CAMERA_W), int(ymin * CAMERA_H)), (int(xmax * CAMERA_W), int(ymax * CAMERA_H))
            cv2.rectangle(panel, p1, p2, GREEN, 2)
            distance = view.get("distance_m")
            label = f"person {view.get('confidence', 0.0):.2f}" + (f"  {distance:.1f} m" if distance else "  range ?")
            _text(panel, label, (p1[0], max(18, p1[1] - 6)), GREEN, 0.55)
        cv2.line(panel, (CAMERA_W // 2, CAMERA_H // 2 - 8), (CAMERA_W // 2, CAMERA_H // 2 + 8), DIM, 1)
        cv2.line(panel, (0, CAMERA_H // 2), (CAMERA_W, CAMERA_H // 2), DIM, 1)       # horizon of a level boat
    if view:
        state = "searching" if view.get("scanning") else "following" if view.get("box") is not None and view.get("age_s", 9) < 1.0 else "ahead"
        _text(panel, f"camera {view.get('pan_deg', 0.0):+.0f} deg  {state}", (10, CAMERA_H - 12), YELLOW if view.get("scanning") else WHITE)
    return panel


def draw_map(snap: Snapshot, sector_cm: list[int], max_m: float, keep_outs: list[tuple[float, float, float]],
             view: dict | None, fov_deg: float) -> np.ndarray:
    """keep_outs: (bearing from the bow in degrees, distance in m, radius in m)."""
    panel = np.full((MAP, MAP, 3), 24, np.uint8)
    centre, scale = MAP // 2, (MAP / 2 - 22) / max_m          # pixels per metre

    def point(bearing_deg: float, distance_m: float) -> tuple[int, int]:
        b = math.radians(bearing_deg)
        return int(centre + distance_m * math.sin(b) * scale), int(centre - distance_m * math.cos(b) * scale)

    for ring in range(2, int(max_m) + 1, 2):
        cv2.circle(panel, (centre, centre), int(ring * scale), DIM, 1, cv2.LINE_AA)
        _text(panel, f"{ring} m", (centre + 4, centre - int(ring * scale) + 14), GREY, 0.4)
    # The camera's field of view.
    if view is not None:
        pan, half = float(view.get("pan_deg", 0.0)), fov_deg / 2.0
        colour = YELLOW if view.get("scanning") else GREEN
        wedge = np.array([(centre, centre)] + [point(pan + a, max_m) for a in np.linspace(-half, half, 13)], np.int32)
        overlay = panel.copy()
        cv2.fillPoly(overlay, [wedge], colour)
        panel = cv2.addWeighted(overlay, 0.18, panel, 0.82, 0)
        cv2.polylines(panel, [wedge], True, colour, 1, cv2.LINE_AA)
    # Sectors as the autopilot gets them: an arc at the reported distance.
    clear = int(max_m * 100) + 1
    for index, cm in enumerate(sector_cm):
        if cm < clear:
            a0, a1 = index * SECTOR_DEG - SECTOR_DEG / 2.0, index * SECTOR_DEG + SECTOR_DEG / 2.0
            arc = np.array([point(a, cm / 100.0) for a in np.linspace(a0, a1, 4)], np.int32)
            cv2.polylines(panel, [arc], False, RED, 3, cv2.LINE_AA)
    for o in snap.obstacles:                                   # raw LiDAR returns
        cv2.circle(panel, point(o.bearing_body_deg, o.distance_m), 1, WHITE, -1)
    for bearing, distance, radius in keep_outs:
        if distance - radius <= max_m:
            cv2.circle(panel, point(bearing, distance), max(2, int(radius * scale)), ORANGE, 2, cv2.LINE_AA)
    target = snap.human_target
    if target is not None and monotonic() - target.timestamp < 1.0:
        p = point(target.bearing_body_deg, min(target.distance_m, max_m))
        cv2.drawMarker(panel, p, GREEN, cv2.MARKER_TILTED_CROSS, 14, 2, cv2.LINE_AA)
        _text(panel, f"{target.distance_m:.1f} m", (p[0] + 8, p[1] - 6), GREEN, 0.45)
    # The boat, 1.10 x 0.60 m, bow up.
    half_l, half_w = int(0.55 * scale), int(0.30 * scale)
    hull = np.array([(centre, centre - half_l - 4), (centre + half_w, centre - half_l // 2), (centre + half_w, centre + half_l),
                     (centre - half_w, centre + half_l), (centre - half_w, centre - half_l // 2)], np.int32)
    cv2.fillPoly(panel, [hull], BLUE)
    _text(panel, "bow", (centre - 14, 16), GREY, 0.45)
    _text(panel, f"LiDAR: {len(snap.obstacles)} returns", (8, MAP - 10), GREY, 0.45)
    return panel


def render(snap: Snapshot, view: dict | None, sector_cm: list[int], max_m: float,
           keep_outs: list[tuple[float, float, float]], fov_deg: float, phase: str) -> np.ndarray:
    top = np.hstack([draw_camera(view), draw_map(snap, sector_cm, max_m, keep_outs, view, fov_deg)])
    bar = np.full((BAR, top.shape[1], 3), 12, np.uint8)
    vehicle = snap.vehicle
    nearest = min((o.distance_m for o in snap.obstacles), default=None)
    status = (f"{vehicle.mode or '?'}  {'ARMED' if vehicle.armed else 'disarmed'}   phase {phase}   mission item {vehicle.mission_seq}"
              + (f"  ({vehicle.wp_dist_m:.0f} m)" if vehicle.wp_dist_m >= 0 else "")
              + f"   nearest obstacle {f'{nearest:.1f} m' if nearest is not None else 'none'}"
              + f"   speed {snap.gps.speed_mps:.1f} m/s   heading {snap.imu.yaw_deg:.0f}")
    _text(bar, status, (10, 28), WHITE, 0.55)
    return np.vstack([top, bar])


class Viewer(threading.Thread):
    def __init__(self, port: int, state: StateStore, stop_event: threading.Event, avoidance: dict, camera: dict,
                 camera_view: Callable[[], dict | None], keep_outs: Callable[[], list[tuple[float, float]]],
                 attending: Callable[[], tuple[float, float] | None], phase: Callable[[], str], rate_hz: float = 5.0) -> None:
        super().__init__(name="viewer", daemon=True)
        self.port, self.state, self.stop = port, state, stop_event
        self.a, self.fov = avoidance, float(camera["horizontal_fov_deg"])
        self.camera_view, self.keep_outs, self.attending, self.phase = camera_view, keep_outs, attending, phase
        self.period = 1.0 / rate_hz
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
        return render(snap, self.camera_view(), sectors(objects, min_m, max_m), max_m, keep, self.fov, self.phase())

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
                    ok, encoded = cv2.imencode(".jpg", self._frame(), [cv2.IMWRITE_JPEG_QUALITY, 80])
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

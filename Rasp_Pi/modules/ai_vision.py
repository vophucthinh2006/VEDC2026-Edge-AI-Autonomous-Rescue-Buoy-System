"""TFLite person detection and safe camera-to-LiDAR target lock.

camera.model picks the detector: yolo_movenet (the team's YOLOv8n + MoveNet, modules/edge_model.py)
or ssd (the earlier COCO SSD MobileNet v1, models/detect.tflite)."""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from pathlib import Path
from time import monotonic, sleep, strftime
from typing import Callable

import cv2
import numpy as np

from modules.edge_model import EDGES
from modules.perception import SweepPattern, associate_person, estimate_distance_bbox, make_scan, standing_score, track_pan
from modules.state_store import HumanTarget, StateStore
from utils.geometry import bearing_deg, signed_angle_deg


@dataclass(frozen=True)
class PersonDetection:
    bearing_cam_deg: float   # from the camera axis, positive right
    confidence: float
    box_height_frac: float   # bigger box = nearer person
    cropped: bool            # box touches the frame top or bottom: height understates the person
    box: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)  # ymin, xmin, ymax, xmax, 0..1; for drawing
    in_water: bool | None = None   # yolo_movenet: lower body hidden (pose); None without a pose
    keypoints: tuple[tuple[float, float, float], ...] = ()   # yolo_movenet: x, y (0..1 of the frame), score
    near_camera: bool = False      # yolo_movenet: box on the bottom edge, so not in the water (NEAR_CAMERA_SAFE)


def source_needs_box(camera: dict) -> bool:
    """Whether the range can come from the box height (everything but distance_source: lidar)."""
    return camera["distance_source"] != "lidar"


class CameraReader:
    """Frames from a capture that may drop off the bus. No frame for lost_s: one warning, the capture
    is released and reopened every lost_s until frames come again. open_capture returns an opened
    capture or None."""

    def __init__(self, open_capture: Callable[[], object | None], name: str, lost_s: float, log: logging.Logger,
                 clock: Callable[[], float] = monotonic, pause: Callable[[float], None] = sleep) -> None:
        self._open, self._name, self._lost_s, self._log = open_capture, name, lost_s, log
        self._clock, self._pause = clock, pause
        self._cap = None
        self._next_open = 0.0
        self._last_frame = clock()
        self._lost_since: float | None = None

    def read(self) -> np.ndarray | None:
        now = self._clock()
        if self._cap is None:
            if now < self._next_open:
                self._pause(0.2)
                return None
            self._cap = self._open()
            self._next_open = now + self._lost_s
            if self._cap is None:
                self._mark_lost(now, "cannot open")
                return None
            self._last_frame = now
        ok, frame = self._cap.read()
        if not ok:
            if now - self._last_frame > self._lost_s:
                self._cap.release()
                self._cap = None
                self._next_open = now + self._lost_s
                self._mark_lost(now, f"no frame for {now - self._last_frame:.0f} s from")
            else:
                self._pause(0.05)
            return None
        self._last_frame = now
        if self._lost_since is not None:
            self._log.warning("camera %s back after %.0f s", self._name, now - self._lost_since)
            self._lost_since = None
        return frame

    def _mark_lost(self, now: float, what: str) -> None:
        if self._lost_since is None:
            self._lost_since = now
            self._log.warning("%s camera %s: reopening every %.0f s", what, self._name, self._lost_s)

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class VisionWorker(threading.Thread):
    def __init__(self, camera: dict, state: StateStore, stop_event: threading.Event, on_confirmed: Callable[[HumanTarget], None], send_pan: Callable[[float], None], standoff_m: float) -> None:
        super().__init__(name="vision", daemon=True)
        self.c = camera
        self.state, self.stop, self.on_confirmed = state, stop_event, on_confirmed
        self.send_pan, self.standoff_m = send_pan, standoff_m
        self.log = logging.getLogger(__name__)
        self._last_seen = 0.0
        self._last_report = 0.0
        self._pan = 0.0          # camera servo angle from the bow, positive right
        self._sent_pan = 0.0
        self._sent_at = 0.0
        # Search sweep (camera.scan): only while the autopilot runs the search pattern.
        scan = camera.get("scan") or {}
        self._scan = make_scan(scan)
        self._scan_modes = {str(mode).upper() for mode in scan.get("modes", [])}
        self._scan_always = bool(scan.get("always", False))   # bench: no RC to arm with
        self._scanning = False
        # A smooth sweep goes out at camera.pan_rate_hz from its own thread, not once per detector
        # pass: at 0.8 s a pass the servo would jump 23 deg at a time. The lock keeps that thread and
        # this one from moving the sweep or the pan at the same time.
        self._pan_rate_hz = float(camera.get("pan_rate_hz", 20.0))
        self._pan_lock = threading.RLock()
        # camera.snapshot_dir: a JPEG of the frame each time somebody is in view, at most one per snapshot_cooldown_s.
        self._snapshot_dir = camera.get("snapshot_dir")
        self._last_snapshot = 0.0
        self._last_frame = 0.0
        self._ranged_standing: bool | None = None
        self._frame_aspect = 4.0 / 3.0       # width over height, taken from the frames once they come
        # For the live viewer (modules/viewer.py): the last frame read and what was made of the last one looked at.
        self._view_frame: np.ndarray | None = None
        self._view_result: tuple[PersonDetection | None, float, float | None, float] = (None, 0.0, None, 0.0)
        self._view_others: list[tuple[PersonDetection, float | None]] = []
        self._followed_bearing: float | None = None      # from the bow, of the person followed in the last frame
        self._model = None                                # edge_model.EdgeModel for camera.model yolo_movenet
        self._view_ignored: list[tuple[float, float, float, float]] = []   # boxes of people on a boat
        self._view_boats: list[tuple[float, float, float, float]] = []

    @staticmethod
    def _dequantize(value: np.ndarray, detail: dict) -> np.ndarray:
        scale, zero = detail.get("quantization", (0.0, 0))
        return (value.astype(np.float32) - zero) * scale if scale else value.astype(np.float32)

    def _person(self, box: tuple[float, float, float, float], confidence: float, in_water: bool | None = None,
                keypoints: tuple = (), near_camera: bool = False) -> PersonDetection:
        ymin, xmin, ymax, xmax = box
        bearing = ((xmin + xmax) / 2.0 - 0.5) * float(self.c["horizontal_fov_deg"])
        return PersonDetection(bearing, confidence, max(0.0, ymax - ymin), ymin < 0.02 or ymax > 0.98, box, in_water, keypoints,
                               near_camera)

    def _detect_edge(self, frame: np.ndarray) -> list[PersonDetection]:
        """YOLOv8n + MoveNet. People on a boat are left out: they are not the ones to rescue."""
        people, boats = self._model.detect(frame)
        self._view_boats = [b.box for b in boats]
        self._view_ignored = [p.box for p in people if p.on_boat]
        found = [self._person(p.box, p.confidence, p.in_water, p.keypoints, p.near_camera) for p in people if not p.on_boat]
        return sorted(found, key=lambda d: -d.box_height_frac)

    def _detect(self, interpreter: object, details: list[dict], frame: np.ndarray) -> list[PersonDetection]:
        """Every person in the frame over the confidence threshold, tallest box (nearest) first."""
        if self._model is not None:
            return self._detect_edge(frame)
        input_detail = details[0]
        height, width = int(input_detail["shape"][1]), int(input_detail["shape"][2])
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (width, height))
        if input_detail["dtype"] == np.float32:
            image = (resized.astype(np.float32) - 127.5) / 127.5
        elif input_detail["dtype"] == np.int8:
            image = (resized.astype(np.int16) - 128).astype(np.int8)
        else:
            # uint8 SSD (e.g. coco_ssd_mobilenet_v1 quant): its quantization (1/128, 128) already maps raw 0..255 pixels
            # to -1..1, so feed the pixels as they are.
            image = resized
        image = np.expand_dims(image, axis=0)
        interpreter.set_tensor(input_detail["index"], image)
        interpreter.invoke()
        output_pairs = [(detail, self._dequantize(interpreter.get_tensor(detail["index"])[0], detail)) for detail in details[1:]]
        outputs = [array for _, array in output_pairs]
        # Standard SSD DetectPostProcess output: boxes [N,4], classes [N], scores [N], count [1].
        boxes = next((array for array in outputs if array.ndim == 2 and array.shape[-1] == 4), None)
        vectors = [array for array in outputs if array.ndim == 1 and array.size > 1]
        if boxes is None or len(vectors) < 2:
            self.log.error("unsupported TFLite output layout; adapt ai_vision.py to this model")
            return []
        def named(token: str) -> np.ndarray | None:
            return next((array for detail, array in output_pairs if token in str(detail.get("name", "")).lower()), None)
        scores = named("score")
        classes = named("class")
        # Fallback for unnamed standard SSD graphs: output order is boxes, classes, scores, count.
        if scores is None:
            scores = vectors[1] if len(vectors) >= 2 else vectors[0]
        if classes is None:
            classes = vectors[0] if vectors[0] is not scores else vectors[1]
        found: list[PersonDetection] = []
        for index, score in enumerate(scores[: len(boxes)]):
            if score < self.c["confidence_threshold"] or index >= len(classes) or int(round(float(classes[index]))) != 0:
                continue
            ymin, xmin, ymax, xmax = (float(v) for v in boxes[index][:4])
            found.append(self._person((ymin, xmin, ymax, xmax), float(score)))
        return sorted(found, key=lambda d: -d.box_height_frac)

    def _focus_bearing(self) -> float | None:
        """Bearing from the bow of the person the rescue logic named, if it named one."""
        snap = self.state.snapshot()
        if snap.focus is None or snap.gps.fix < 3:
            return None
        return signed_angle_deg(bearing_deg(snap.gps.lat_deg, snap.gps.lon_deg, *snap.focus) - snap.imu.yaw_deg)

    def _follow(self, detections: list[PersonDetection], pan_deg: float) -> PersonDetection:
        """Which of several people the camera stays on. The one the rescue logic is going to, if it
        named one (state focus); else the one followed in the last frame; else the nearest. Without
        this the tallest box won every frame, and with two people equally far the choice, the servo
        and the position sent to the autopilot could change from one frame to the next."""
        focus = self._focus_bearing()
        wanted = focus if focus is not None else self._followed_bearing
        if wanted is None:
            return detections[0]
        nearest = min(detections, key=lambda d: abs(signed_angle_deg(pan_deg + d.bearing_cam_deg - wanted)))
        gate = float(self.c.get("follow_gate_deg", 15.0))
        return nearest if abs(signed_angle_deg(pan_deg + nearest.bearing_cam_deg - wanted)) <= gate else detections[0]

    def _distance(self, detection: PersonDetection, bearing_body_deg: float, followed: bool = True) -> float | None:
        snap = self.state.snapshot()
        # Somebody standing clear of the water (on a roof) or a swimmer: by the shape of the box and
        # where its bottom edge falls (perception.standing_score).
        score = standing_score(detection.box, self._frame_aspect, self.c["vertical_fov_deg"], snap.imu.pitch_deg)
        standing_min = float(self.c.get("standing_score_min", 2.5))
        elevated = score >= standing_min
        # Between water_score_max and standing_score_min the box could be either, and the two
        # readings of its range differ by standing_height_m / person_height_m (over three times).
        # Rolling on the waves, somebody in a doorway flickered across a single threshold and became
        # a second person next to the boat. Such a frame gives no range; a clearer one will.
        if float(self.c.get("water_score_max", standing_min)) < score < standing_min and source_needs_box(self.c):
            return None
        if followed and elevated != self._ranged_standing:
            self._ranged_standing = elevated
            self.log.info("person ranged as %s (score %.1f)", "standing" if elevated else "in the water", score)
        source = self.c["distance_source"]
        # fused: the LiDAR ranges only people standing on something, whose wall it hits. A swimmer is
        # below the scan plane: the return at their bearing is whatever stands behind them, and with
        # the camera looking sideways down a flooded street that is always a house.
        height_m = float(self.c.get("standing_height_m", self.c["person_height_m"])) if elevated else self.c["person_height_m"]
        box_range = estimate_distance_bbox(detection.box_height_frac, self.c["vertical_fov_deg"], height_m)
        if source == "lidar" or (source == "fused" and elevated):
            obstacle = associate_person(bearing_body_deg, snap.obstacles, self.c["lidar_association_half_angle_deg"], self.c["min_target_distance_m"], self.c["max_target_distance_m"])
            # fused: the return must be the wall under the person, at about the range their box gives.
            # Bamboo standing 2 m in front of somebody on a roof 8 m away put them next to the boat.
            low, high = self.c.get("lidar_box_agreement", (0.6, 1.6))
            if obstacle and (source == "lidar" or detection.cropped or low * box_range <= obstacle.distance_m <= high * box_range):
                return obstacle.distance_m
            if source == "lidar":
                return None   # never navigate from a 2-D box alone
        if detection.cropped:
            # Too close to size up. The person the boat is going to: hold at the stand-off instead of
            # guessing. Anybody else, often somebody on a roof whose head is out of the frame: no
            # range. Placed at the stand-off they became a phantom right at the boat, "reached" at once.
            return self.standoff_m if followed and snap.focus is not None else None
        # person_height_m is what a box covers of somebody in the water. Somebody standing clear of
        # it shows their whole height; with the swimmer's figure their range would read 2.5 times too short.
        return box_range if box_range <= self.c["max_target_distance_m"] else None

    def _recenter(self) -> None:
        step = float(self.c["recenter_step_deg"])
        self._pan = 0.0 if abs(self._pan) <= step else self._pan - step * (1 if self._pan > 0 else -1)
        self._publish_pan(monotonic())

    def view(self) -> dict:
        detection, pan_deg, distance_m, at = self._view_result
        return {"frame": self._view_frame, "box": detection.box if detection else None,
                "confidence": detection.confidence if detection else 0.0, "distance_m": distance_m,
                "in_water": detection.in_water if detection else None, "keypoints": detection.keypoints if detection else (),
                "others": [(d.box, d.confidence, range_m) for d, range_m in self._view_others],
                "boats": self._view_boats, "on_boat": self._view_ignored,
                "model": "YOLOv8n + MoveNet" if self._model is not None else "SSD MobileNet",
                "pan_deg": self._pan if detection is None else pan_deg, "scanning": self._scanning, "age_s": monotonic() - at}

    def _searching(self) -> bool:
        vehicle = self.state.snapshot().vehicle
        return self._scan is not None and (self._scan_always or (vehicle.armed and vehicle.mode in self._scan_modes))

    def _save_snapshot(self, frame: np.ndarray, people: list[PersonDetection], followed: PersonDetection,
                       pan_deg: float, now: float) -> None:
        """Everybody in view with their box, skeleton and state; the person followed in a thicker box,
        people on a boat (left out of people) in orange."""
        if not self._snapshot_dir or now - self._last_snapshot < float(self.c.get("snapshot_cooldown_s", 5.0)):
            return
        self._last_snapshot = now
        image = frame.copy()
        height, width = image.shape[:2]
        states = []
        for ymin, xmin, ymax, xmax in self._view_ignored:
            states.append("ON BOAT")
            cv2.rectangle(image, (int(xmin * width), int(ymin * height)), (int(xmax * width), int(ymax * height)), (0, 160, 255), 1)
            cv2.putText(image, "ON BOAT", (int(xmin * width) + 3, max(14, int(ymin * height) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 160, 255), 2)
        for person in people:
            state, colour = (("NEAR CAMERA", (0, 220, 0)) if person.near_camera else ("IN WATER", (0, 0, 255)) if person.in_water
                             else ("NO POSE", (200, 200, 200)) if person.in_water is None else ("ON LAND", (0, 220, 0)))
            states.append(state)
            ymin, xmin, ymax, xmax = person.box
            cv2.rectangle(image, (int(xmin * width), int(ymin * height)), (int(xmax * width), int(ymax * height)),
                          colour, 3 if person is followed else 1)
            cv2.putText(image, f"{state} {person.confidence:.2f}", (int(xmin * width) + 3, max(14, int(ymin * height) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            points = person.keypoints
            for a, b in EDGES:
                if len(points) > max(a, b) and points[a][2] > 0.25 and points[b][2] > 0.25:
                    cv2.line(image, (int(points[a][0] * width), int(points[a][1] * height)),
                             (int(points[b][0] * width), int(points[b][1] * height)), (255, 200, 0), 2)
            for x, y, score in points:
                if score > 0.25:
                    cv2.circle(image, (int(x * width), int(y * height)), 3, (0, 255, 255), -1)
        cv2.putText(image, f"{len(states)} in view  pan {pan_deg:+.0f} deg", (8, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        directory = Path(self._snapshot_dir)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"person_{strftime('%Y%m%d_%H%M%S')}.jpg"
        cv2.imwrite(str(path), image)
        self.log.warning("person in view: %s, pan %+.0f deg, saved %s", ", ".join(states), pan_deg, path)

    def _publish_pan(self, now: float) -> None:
        with self._pan_lock:
            if abs(self._pan - self._sent_pan) >= 0.5 or now - self._sent_at >= 0.25:
                self.send_pan(self._pan)
                self._sent_pan, self._sent_at = self._pan, now

    def _sweep_step(self, now: float) -> None:
        """Move a smooth sweep on to `now` and send the angle; nothing outside a smooth sweep."""
        with self._pan_lock:
            if self._scanning and isinstance(self._scan, SweepPattern):
                self._pan = self._scan.frame_done(now)
                self._publish_pan(now)

    def _pan_loop(self) -> None:
        period = 1.0 / self._pan_rate_hz
        while not self.stop.is_set():
            self._sweep_step(monotonic())
            self.stop.wait(period)

    def run(self) -> None:
        # tflite-runtime on the Pi; it has no wheels for Python 3.12, where its
        # successor ai-edge-litert takes over (the simulator under WSL).
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            try:
                from ai_edge_litert.interpreter import Interpreter
            except ImportError:
                self.log.error("neither tflite-runtime nor ai-edge-litert is installed; camera worker disabled")
                return
        interpreter, details = None, []
        if self.c.get("model", "ssd") == "yolo_movenet":
            from modules.edge_model import EdgeModel
            self._model = EdgeModel(Interpreter, self.c)
            self.log.info("detector: YOLOv8n %d px + %s", self._model.size, "MoveNet" if self.c.get("pose_model_path") else "no pose")
        else:
            interpreter = Interpreter(model_path=self.c["model_path"])
            interpreter.allocate_tensors()
            details = interpreter.get_input_details() + interpreter.get_output_details()
        gazebo = self.c.get("source", "device") == "gazebo"

        def open_capture():
            if gazebo:
                from modules.gazebo_io import GazeboCapture   # simulator only: needs the Gazebo bindings
                cap = GazeboCapture(self.c["gazebo_topic"])
            else:
                device = self.c["device"]
                # An index (0) or a path (/dev/v4l/by-id/...): the index follows the plugging order.
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2) if isinstance(device, str) else cv2.VideoCapture(int(device))
            if not cap.isOpened():
                cap.release()
                return None
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # a queue of old frames delays the camera servo loop
            return cap

        # A USB camera can drop off the bus and come back: camera_lost_s without a frame and it is reopened.
        camera = CameraReader(open_capture, str(self.c["gazebo_topic"] if gazebo else self.c["device"]),
                              float(self.c.get("camera_lost_s", 2.0)), self.log)
        max_fps = float(self.c.get("max_fps", 0.0))
        self.log.info("vision worker started, search sweep %s", "off" if self._scan is None
                      else f"smooth +-{self._scan.limit:.0f} deg at {self._scan.rate:.0f} deg/s" if isinstance(self._scan, SweepPattern)
                      else f"steps {self._scan.angles} deg")
        if isinstance(self._scan, SweepPattern) and self._pan_rate_hz > 0:
            threading.Thread(target=self._pan_loop, name="camera-pan", daemon=True).start()
        try:
            while not self.stop.is_set():
                frame = camera.read()
                if frame is None:
                    continue
                now = monotonic()
                self._view_frame = frame
                self._frame_aspect = frame.shape[1] / frame.shape[0]
                self._sweep_step(now)        # without the pan thread (pan_rate_hz: 0), once per frame
                pan_at_capture = self._pan   # the sweep moves on while the detector runs
                # Simulator only (camera.max_fps): hold the detector to the rate expected of the Pi.
                if max_fps and now - self._last_frame < 1.0 / max_fps:
                    continue
                if self._scanning and not self._scan.usable(now):
                    continue   # the servo is still moving: blurred frame, angle unknown
                self._last_frame = now
                detections = self._detect(interpreter, details, frame)
                now = monotonic()
                if not detections:
                    self._view_result, self._view_others = (None, self._pan, None, now), []
                    if now - self._last_seen > self.c["detection_hold_s"]:
                        self.state.update_human_target(None)
                        self._followed_bearing = None
                        focus = self._focus_bearing()
                        if focus is not None:
                            # The rescue logic is going to somebody not in the frame: look their way.
                            self._scanning = False
                            self._pan = max(-float(self.c["pan_limit_deg"]), min(float(self.c["pan_limit_deg"]), focus))
                            self._publish_pan(now)
                        elif now - self._last_seen > self.c["recenter_after_s"]:
                            if self._searching():
                                with self._pan_lock:
                                    self._pan = self._scan.frame_done(now) if self._scanning else self._scan.start(now, self._pan)
                                    self._scanning = True
                                    self._publish_pan(now)
                            else:
                                self._scanning = False
                                self._recenter()
                    continue
                with self._pan_lock:
                    self._scanning = False   # somebody in view: stop the sweep, the servo now follows them
                self._last_seen = now
                detection = self._follow(detections, pan_at_capture)
                self._save_snapshot(frame, detections, detection, pan_at_capture, now)
                with self._pan_lock:
                    # From where the camera pointed when the frame was taken, not where the sweep got to since.
                    self._pan = track_pan(pan_at_capture, detection.bearing_cam_deg, self.c["pan_gain"], self.c["pan_deadband_deg"], self.c["pan_limit_deg"])
                    self._publish_pan(now)
                # Everybody in the frame goes to the rescue logic, each with a bearing and a range.
                targets, target, others = [], None, []
                for found in detections:
                    bearing_body = signed_angle_deg(pan_at_capture + found.bearing_cam_deg)
                    distance = self._distance(found, bearing_body, followed=found is detection)
                    if found is detection:
                        self._followed_bearing = bearing_body
                        self._view_result = (detection, pan_at_capture, distance, now)
                    else:
                        others.append((found, distance))
                    if distance is not None:
                        targets.append(HumanTarget(bearing_body, distance, found.confidence, now))
                        if found is detection:
                            target = targets[-1]
                self._view_others = others
                self.state.update_human_targets(tuple(targets), target)
                if target is None:
                    continue
                if now - self._last_report >= self.c["report_cooldown_s"]:
                    self.on_confirmed(target)
                    self._last_report = now
        finally:
            camera.release()

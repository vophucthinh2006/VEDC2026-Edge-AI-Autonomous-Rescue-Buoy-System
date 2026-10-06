"""TFLite person detection and safe camera-to-LiDAR target lock."""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from time import monotonic, sleep
from typing import Callable

import cv2
import numpy as np

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
        self._scanning = False
        self._last_frame = 0.0
        self._ranged_standing: bool | None = None
        self._frame_aspect = 4.0 / 3.0       # width over height, taken from the frames once they come
        # For the live viewer (modules/viewer.py): the last frame read and what was made of the last one looked at.
        self._view_frame: np.ndarray | None = None
        self._view_result: tuple[PersonDetection | None, float, float | None, float] = (None, 0.0, None, 0.0)
        self._view_others: list[tuple[PersonDetection, float | None]] = []
        self._followed_bearing: float | None = None      # from the bow, of the person followed in the last frame

    @staticmethod
    def _dequantize(value: np.ndarray, detail: dict) -> np.ndarray:
        scale, zero = detail.get("quantization", (0.0, 0))
        return (value.astype(np.float32) - zero) * scale if scale else value.astype(np.float32)

    def _detect(self, interpreter: object, details: list[dict], frame: np.ndarray) -> list[PersonDetection]:
        """Every person in the frame over the confidence threshold, tallest box (nearest) first."""
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
            center_x = (xmin + xmax) / 2.0
            bearing = (center_x - 0.5) * float(self.c["horizontal_fov_deg"])
            height = max(0.0, ymax - ymin)
            found.append(PersonDetection(bearing, float(score), height, ymin < 0.02 or ymax > 0.98, (ymin, xmin, ymax, xmax)))
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
        elevated = score >= float(self.c.get("standing_score_min", 2.5))
        if followed and elevated != self._ranged_standing:
            self._ranged_standing = elevated
            self.log.info("person ranged as %s (score %.1f)", "standing" if elevated else "in the water", score)
        source = self.c["distance_source"]
        # fused: the LiDAR ranges only people standing on something, whose wall it hits. A swimmer is
        # below the scan plane: the return at their bearing is whatever stands behind them, and with
        # the camera looking sideways down a flooded street that is always a house.
        if source == "lidar" or (source == "fused" and elevated):
            obstacle = associate_person(bearing_body_deg, snap.obstacles, self.c["lidar_association_half_angle_deg"], self.c["min_target_distance_m"], self.c["max_target_distance_m"])
            if obstacle:
                return obstacle.distance_m
            if source == "lidar":
                return None   # never navigate from a 2-D box alone
        if detection.cropped:
            return self.standoff_m   # too close to size up: hold at the stand-off instead of guessing
        # person_height_m is what a box covers of somebody in the water. Somebody standing clear of
        # it shows their whole height; with the swimmer's figure their range would read 2.5 times too short.
        height_m = float(self.c.get("standing_height_m", self.c["person_height_m"])) if elevated else self.c["person_height_m"]
        distance = estimate_distance_bbox(detection.box_height_frac, self.c["vertical_fov_deg"], height_m)
        return distance if distance <= self.c["max_target_distance_m"] else None

    def _recenter(self) -> None:
        step = float(self.c["recenter_step_deg"])
        self._pan = 0.0 if abs(self._pan) <= step else self._pan - step * (1 if self._pan > 0 else -1)
        self._publish_pan(monotonic())

    def view(self) -> dict:
        detection, pan_deg, distance_m, at = self._view_result
        return {"frame": self._view_frame, "box": detection.box if detection else None,
                "confidence": detection.confidence if detection else 0.0, "distance_m": distance_m,
                "others": [(d.box, d.confidence, range_m) for d, range_m in self._view_others],
                "pan_deg": self._pan if detection is None else pan_deg, "scanning": self._scanning, "age_s": monotonic() - at}

    def _searching(self) -> bool:
        vehicle = self.state.snapshot().vehicle
        return self._scan is not None and vehicle.armed and vehicle.mode in self._scan_modes

    def _publish_pan(self, now: float) -> None:
        if abs(self._pan - self._sent_pan) >= 0.5 or now - self._sent_at >= 0.25:
            self.send_pan(self._pan)
            self._sent_pan, self._sent_at = self._pan, now

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
        interpreter = Interpreter(model_path=self.c["model_path"])
        interpreter.allocate_tensors()
        details = interpreter.get_input_details() + interpreter.get_output_details()
        if self.c.get("source", "device") == "gazebo":
            from modules.gazebo_io import GazeboCapture   # simulator only: needs the Gazebo bindings
            cap = GazeboCapture(self.c["gazebo_topic"])
        else:
            cap = cv2.VideoCapture(int(self.c["device"]))
        if not cap.isOpened():
            self.log.error("cannot open camera %s", self.c.get("gazebo_topic") if self.c.get("source") == "gazebo" else self.c["device"])
            return
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # a queue of old frames delays the camera servo loop
        max_fps = float(self.c.get("max_fps", 0.0))
        self.log.info("vision worker started, search sweep %s", "off" if self._scan is None
                      else f"smooth +-{self._scan.limit:.0f} deg at {self._scan.rate:.0f} deg/s" if isinstance(self._scan, SweepPattern)
                      else f"steps {self._scan.angles} deg")
        try:
            while not self.stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    sleep(0.05)
                    continue
                now = monotonic()
                self._view_frame = frame
                self._frame_aspect = frame.shape[1] / frame.shape[0]
                # A smooth sweep is commanded on every camera frame, not only on those the detector
                # gets to look at: at 5 frames a second the servo would move in 6 deg jumps.
                if self._scanning and isinstance(self._scan, SweepPattern):
                    self._pan = self._scan.frame_done(now)
                    self._publish_pan(now)
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
                                self._pan = self._scan.frame_done(now) if self._scanning else self._scan.start(now, self._pan)
                                self._scanning = True
                                self._publish_pan(now)
                            else:
                                self._scanning = False
                                self._recenter()
                    continue
                self._scanning = False   # somebody in view: stop the sweep, the servo now follows them
                self._last_seen = now
                pan_at_capture = self._pan
                detection = self._follow(detections, pan_at_capture)
                self._pan = track_pan(self._pan, detection.bearing_cam_deg, self.c["pan_gain"], self.c["pan_deadband_deg"], self.c["pan_limit_deg"])
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
            cap.release()

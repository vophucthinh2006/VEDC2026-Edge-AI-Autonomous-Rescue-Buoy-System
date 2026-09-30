"""TFLite person detection and safe camera-to-LiDAR target lock."""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from time import monotonic, sleep
from typing import Callable

import cv2
import numpy as np

from modules.perception import associate_person, estimate_distance_bbox, track_pan
from modules.state_store import HumanTarget, StateStore
from utils.geometry import signed_angle_deg


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

    @staticmethod
    def _dequantize(value: np.ndarray, detail: dict) -> np.ndarray:
        scale, zero = detail.get("quantization", (0.0, 0))
        return (value.astype(np.float32) - zero) * scale if scale else value.astype(np.float32)

    def _detect(self, interpreter: object, details: list[dict], frame: np.ndarray) -> PersonDetection | None:
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
            return None
        def named(token: str) -> np.ndarray | None:
            return next((array for detail, array in output_pairs if token in str(detail.get("name", "")).lower()), None)
        scores = named("score")
        classes = named("class")
        # Fallback for unnamed standard SSD graphs: output order is boxes, classes, scores, count.
        if scores is None:
            scores = vectors[1] if len(vectors) >= 2 else vectors[0]
        if classes is None:
            classes = vectors[0] if vectors[0] is not scores else vectors[1]
        best: PersonDetection | None = None
        for index, score in enumerate(scores[: len(boxes)]):
            if score < self.c["confidence_threshold"] or index >= len(classes) or int(round(float(classes[index]))) != 0:
                continue
            ymin, xmin, ymax, xmax = (float(v) for v in boxes[index][:4])
            center_x = (xmin + xmax) / 2.0
            bearing = (center_x - 0.5) * float(self.c["horizontal_fov_deg"])
            height = max(0.0, ymax - ymin)
            # The nearest person is the one with the tallest box.
            if best is None or height > best.box_height_frac:
                best = PersonDetection(bearing, float(score), height, ymin < 0.02 or ymax > 0.98, (ymin, xmin, ymax, xmax))
        return best

    def _distance(self, detection: PersonDetection, bearing_body_deg: float) -> float | None:
        if self.c["distance_source"] == "lidar":
            # Report only after a reliable LiDAR association: never navigate from a 2-D box alone.
            snap = self.state.snapshot()
            obstacle = associate_person(bearing_body_deg, snap.obstacles, self.c["lidar_association_half_angle_deg"], self.c["min_target_distance_m"], self.c["max_target_distance_m"])
            return obstacle.distance_m if obstacle else None
        if detection.cropped:
            return self.standoff_m   # too close to size up: hold at the stand-off instead of guessing
        distance = estimate_distance_bbox(detection.box_height_frac, self.c["vertical_fov_deg"], self.c["person_height_m"])
        return distance if distance <= self.c["max_target_distance_m"] else None

    def _recenter(self) -> None:
        step = float(self.c["recenter_step_deg"])
        self._pan = 0.0 if abs(self._pan) <= step else self._pan - step * (1 if self._pan > 0 else -1)
        self._publish_pan(monotonic())

    def _publish_pan(self, now: float) -> None:
        if abs(self._pan - self._sent_pan) >= 0.5 or now - self._sent_at >= 0.25:
            self.send_pan(self._pan)
            self._sent_pan, self._sent_at = self._pan, now

    def run(self) -> None:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            self.log.error("tflite-runtime not installed; camera worker disabled")
            return
        interpreter = Interpreter(model_path=self.c["model_path"])
        interpreter.allocate_tensors()
        details = interpreter.get_input_details() + interpreter.get_output_details()
        cap = cv2.VideoCapture(int(self.c["device"]))
        if not cap.isOpened():
            self.log.error("cannot open camera %s", self.c["device"])
            return
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # a queue of old frames delays the camera servo loop
        self.log.info("vision worker started")
        try:
            while not self.stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    sleep(0.05)
                    continue
                detection = self._detect(interpreter, details, frame)
                now = monotonic()
                if detection is None:
                    if now - self._last_seen > self.c["detection_hold_s"]:
                        self.state.update_human_target(None)
                        if now - self._last_seen > self.c["recenter_after_s"]:
                            self._recenter()
                    continue
                self._last_seen = now
                pan_at_capture = self._pan
                self._pan = track_pan(self._pan, detection.bearing_cam_deg, self.c["pan_gain"], self.c["pan_deadband_deg"], self.c["pan_limit_deg"])
                self._publish_pan(now)
                bearing_body = signed_angle_deg(pan_at_capture + detection.bearing_cam_deg)
                distance = self._distance(detection, bearing_body)
                if distance is None:
                    self.state.update_human_target(None)
                    continue
                target = HumanTarget(bearing_body, distance, detection.confidence, now)
                self.state.update_human_target(target)
                if now - self._last_report >= self.c["report_cooldown_s"]:
                    self.on_confirmed(target)
                    self._last_report = now
        finally:
            cap.release()

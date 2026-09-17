"""TFLite person detection and safe camera-to-LiDAR target lock."""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from time import monotonic, sleep
from typing import Callable

import cv2
import numpy as np

from modules.perception import associate_person
from modules.state_store import HumanTarget, StateStore


@dataclass(frozen=True)
class PersonDetection:
    bearing_body_deg: float
    confidence: float


class VisionWorker(threading.Thread):
    def __init__(self, camera: dict, state: StateStore, stop_event: threading.Event, on_confirmed: Callable[[HumanTarget], None]) -> None:
        super().__init__(name="vision", daemon=True)
        self.c = camera
        self.state, self.stop, self.on_confirmed = state, stop_event, on_confirmed
        self.log = logging.getLogger(__name__)
        self._last_seen = 0.0
        self._last_report = 0.0

    @staticmethod
    def _dequantize(value: np.ndarray, detail: dict) -> np.ndarray:
        scale, zero = detail.get("quantization", (0.0, 0))
        return (value.astype(np.float32) - zero) * scale if scale else value.astype(np.float32)

    def _detect(self, interpreter: object, details: list[dict], frame: np.ndarray) -> PersonDetection | None:
        input_detail = details[0]
        height, width = int(input_detail["shape"][1]), int(input_detail["shape"][2])
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (width, height))
        image = np.expand_dims(resized, axis=0)
        if input_detail["dtype"] == np.float32:
            image = image.astype(np.float32) / 255.0
        else:
            scale, zero = input_detail.get("quantization", (0.0, 0))
            image = image.astype(input_detail["dtype"])
            if scale:
                limits = np.iinfo(input_detail["dtype"])
                image = (resized.astype(np.float32) / 255.0 / scale + zero).clip(limits.min, limits.max).astype(input_detail["dtype"])
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
            xmin, xmax = float(boxes[index][1]), float(boxes[index][3])
            center_x = (xmin + xmax) / 2.0
            bearing = (center_x - 0.5) * float(self.c["horizontal_fov_deg"])
            if best is None or score > best.confidence:
                best = PersonDetection(bearing, float(score))
        return best

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
                    continue
                self._last_seen = now
                snap = self.state.snapshot()
                obstacle = associate_person(detection.bearing_body_deg, snap.obstacles, self.c["lidar_association_half_angle_deg"], self.c["min_target_distance_m"], self.c["max_target_distance_m"])
                if obstacle is None:
                    # Report only after a reliable LiDAR association: never navigate from a 2-D box alone.
                    self.state.update_human_target(None)
                    continue
                target = HumanTarget(obstacle.bearing_body_deg, obstacle.distance_m, detection.confidence, now)
                self.state.update_human_target(target)
                if now - self._last_report >= self.c["report_cooldown_s"]:
                    self.on_confirmed(target)
                    self._last_report = now
        finally:
            cap.release()

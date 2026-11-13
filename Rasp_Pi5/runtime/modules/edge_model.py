"""The team's edge model (YOLO_Movenet/): YOLOv8n finds people and boats, MoveNet reads the pose
of each person who is not on a boat.

Two TFLite models, both from YOLO_Movenet/weights:
  yolov8n_640_int8.tflite    input 1x3x640x640 float RGB 0..1, output 1x6x8400 float: centre x, centre y,
                             width, height (0..1 of the input), score person, score boat. The input
                             size and any quantization are read from the model: the 320 px ones load too.
  movenet_lighting_int8.tflite  input 1x192x192x3 uint8 RGB, output 1x1x17x3: y, x, score (0..1 of the crop)

A person whose box lies mostly inside a boat's box is on that boat: not somebody to rescue. The
pose of the rest says how much of them shows: hips, knees and ankles (COCO keypoints 11 to 16)
scored low on average means the lower body is under water, as in YOLO_Movenet/src/risk_analyzer.py.

Risk of each person, as in the team's diagram (risk_level):
  SAFE      0 green   on a boat
  MEDIUM    1 yellow  lower body seen: not in the water (or near the camera, or no pose read)
  HIGH      2 orange  in the water
  CRITICAL  3 red     in the water with signs of drowning (drowning_signs): both wrists raised
                      above the shoulders, or only the head above the water (shoulders not seen)
Only the nearest camera.max_poses people get a pose; the others stay MEDIUM.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from modules.state_store import CRITICAL, HIGH, MEDIUM, RISK_NAMES, SAFE  # noqa: F401  (RISK_NAMES re-exported)

PERSON, BOAT = 0, 1
LOWER_BODY = (11, 12, 13, 14, 15, 16)
HEAD = (0, 1, 2)                       # nose, eyes
SHOULDERS, WRISTS = (5, 6), (9, 10)    # left, right
# BGR, for the snapshots and the viewer: green, yellow, orange, red
RISK_COLOURS = ((0, 200, 0), (0, 220, 255), (0, 140, 255), (0, 0, 255))
# COCO skeleton, for drawing (YOLO_Movenet/src/config.py KEYPOINT_EDGES)
EDGES = ((0, 1), (0, 2), (1, 3), (2, 4), (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
         (5, 11), (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16))


@dataclass(frozen=True)
class EdgeDetection:
    box: tuple[float, float, float, float]   # ymin, xmin, ymax, xmax, 0..1 of the frame
    confidence: float
    kind: int                                # PERSON or BOAT
    on_boat: bool = False
    in_water: bool | None = None             # None: no pose read
    lower_body_score: float = 0.0
    keypoints: tuple[tuple[float, float, float], ...] = ()   # x, y (0..1 of the frame), score
    near_camera: bool = False                # box on the frame's bottom edge: legs cut off by the frame, not by water
    risk: int = MEDIUM                       # SAFE .. CRITICAL (risk_level)


@dataclass(frozen=True)
class RiskThresholds:
    """camera.risk: keypoint scores (0..1) for drowning_signs."""
    wrist_score: float = 0.3              # both wrists and both shoulders seen at least this well to judge raised arms
    shoulder_hidden_score: float = 0.2    # both shoulders below this: under the water ...
    head_score: float = 0.3               # ... while the nose or an eye is seen at least this well

    @classmethod
    def from_config(cls, risk: dict | None) -> "RiskThresholds":
        risk = risk or {}
        return cls(float(risk.get("wrist_score", cls.wrist_score)), float(risk.get("shoulder_hidden_score", cls.shoulder_hidden_score)),
                   float(risk.get("head_score", cls.head_score)))


def drowning_signs(keypoints: tuple[tuple[float, float, float], ...], limits: RiskThresholds = RiskThresholds()) -> bool:
    """keypoints: 17 COCO points, x, y (y grows downward), score. Both wrists above their shoulders
    (the arms raised, waving or climbing the water), or only the head above the water: the
    shoulders not seen while the nose or an eye is."""
    if len(keypoints) < 17:
        return False
    points = [keypoints[i] for i in SHOULDERS + WRISTS]
    if all(p[2] >= limits.wrist_score for p in points):
        (_, left_shoulder, _), (_, right_shoulder, _), (_, left_wrist, _), (_, right_wrist, _) = points
        if left_wrist < left_shoulder and right_wrist < right_shoulder:
            return True
    shoulders_hidden = all(keypoints[i][2] < limits.shoulder_hidden_score for i in SHOULDERS)
    return shoulders_hidden and max(keypoints[i][2] for i in HEAD) >= limits.head_score


def risk_level(on_boat: bool, in_water: bool | None, keypoints: tuple = (), limits: RiskThresholds = RiskThresholds()) -> int:
    """The team's flow: on a boat SAFE; lower body seen (in_water False, which near_camera forces)
    or no pose read (in_water None) MEDIUM; in the water CRITICAL with drowning_signs, else HIGH."""
    if on_boat:
        return SAFE
    if not in_water:
        return MEDIUM
    return CRITICAL if drowning_signs(keypoints, limits) else HIGH


def make_interpreter(interpreter_class: type, model_path: str, threads: int | None) -> object:
    """The TFLite interpreter on camera.num_threads CPU threads; unset, the runtime's default (one thread
    on the Pi's tflite-runtime, so YOLO had one of the Pi 4's four cores)."""
    if threads:
        try:
            return interpreter_class(model_path=model_path, num_threads=int(threads))
        except TypeError:       # a runtime without the argument
            pass
    return interpreter_class(model_path=model_path)


def letterbox(image: np.ndarray, size: int) -> tuple[np.ndarray, float, float, float]:
    """Scale into a size x size square, keeping the aspect ratio, grey padding. Returns the image,
    the scale and the left and top padding in pixels of the square."""
    height, width = image.shape[:2]
    scale = min(size / height, size / width)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    pad_x, pad_y = (size - new_w) / 2.0, (size - new_h) / 2.0
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top, left = int(round(pad_y - 0.1)), int(round(pad_x - 0.1))
    padded = cv2.copyMakeBorder(resized, top, size - new_h - top, left, size - new_w - left, cv2.BORDER_CONSTANT,
                                value=(114, 114, 114))
    return padded, scale, float(left), float(top)


def decode_yolo(raw: np.ndarray, size: int, scale: float, pad_x: float, pad_y: float, frame_w: int, frame_h: int,
                threshold: float, nms: float, channels: int = 6) -> list[EdgeDetection]:
    """raw: channels x N or N x channels (4 box values, then a score per class). Boxes per class
    after non-maximum suppression, 0..1 of the frame."""
    predictions = np.squeeze(raw)
    if predictions.shape[1] != channels:
        predictions = predictions.T
    scores = predictions[:, 4:]
    kinds = scores.argmax(axis=1)
    best = scores[np.arange(len(scores)), kinds]
    keep = best >= threshold
    if not keep.any():
        return []
    boxes_in, kinds, best = predictions[keep, :4].astype(np.float32), kinds[keep], best[keep]
    # The TFLite export gives 0..1 of the input, the ONNX one pixels of it.
    if float(np.abs(boxes_in).max()) <= 2.0:
        boxes_in = boxes_in * size
    cx, cy, w, h = boxes_in.T
    x1 = np.clip((cx - w / 2.0 - pad_x) / scale, 0, frame_w)
    y1 = np.clip((cy - h / 2.0 - pad_y) / scale, 0, frame_h)
    x2 = np.clip((cx + w / 2.0 - pad_x) / scale, 0, frame_w)
    y2 = np.clip((cy + h / 2.0 - pad_y) / scale, 0, frame_h)
    found: list[EdgeDetection] = []
    for kind in np.unique(kinds):
        index = np.flatnonzero(kinds == kind)
        rects = [[float(x1[i]), float(y1[i]), float(x2[i] - x1[i]), float(y2[i] - y1[i])] for i in index]
        kept = cv2.dnn.NMSBoxes(rects, [float(best[i]) for i in index], threshold, nms)
        for k in np.array(kept).flatten():
            i = index[int(k)]
            if x2[i] - x1[i] < 4 or y2[i] - y1[i] < 4:
                continue
            found.append(EdgeDetection((float(y1[i] / frame_h), float(x1[i] / frame_w), float(y2[i] / frame_h), float(x2[i] / frame_w)),
                                       float(best[i]), int(kind)))
    return found


def overlap_fraction(inner: tuple[float, float, float, float], outer: tuple[float, float, float, float]) -> float:
    """Share of inner's area that lies inside outer. Boxes ymin, xmin, ymax, xmax."""
    area = max(0.0, inner[2] - inner[0]) * max(0.0, inner[3] - inner[1])
    if area <= 0.0:
        return 0.0
    h = max(0.0, min(inner[2], outer[2]) - max(inner[0], outer[0]))
    w = max(0.0, min(inner[3], outer[3]) - max(inner[1], outer[1]))
    return h * w / area


class EdgeModel:
    """Both models behind one call: detect(frame) -> people (with pose), boats."""

    def __init__(self, interpreter_class: type, camera: dict) -> None:
        self.threshold = float(camera["confidence_threshold"])
        self.nms = float(camera.get("nms_threshold", 0.45))
        self.on_boat = float(camera.get("on_boat_overlap", 0.4))
        self.water = float(camera.get("water_lower_body_score", 0.25))
        # NEAR_CAMERA_SAFE of YOLO_Movenet/src/risk_analyzer.py: a box reaching this close to the bottom
        # edge (fraction of the frame height; 15 px of 480 there) is somebody close to the camera.
        self.near_camera = float(camera.get("near_camera_margin", 0.03))
        self.max_poses = int(camera.get("max_poses", 3))
        self.risk = RiskThresholds.from_config(camera.get("risk"))
        self._yolo = make_interpreter(interpreter_class, camera["model_path"], camera.get("num_threads"))
        self._yolo.allocate_tensors()
        self._yolo_in = self._yolo.get_input_details()[0]
        self._yolo_out = self._yolo.get_output_details()[0]
        shape = self._yolo_in["shape"]
        self._nchw = len(shape) == 4 and shape[1] == 3
        self.size = int(shape[2] if self._nchw else shape[1])
        self._pose = None
        if camera.get("pose_model_path"):
            self._pose = make_interpreter(interpreter_class, camera["pose_model_path"], camera.get("num_threads"))
            self._pose.allocate_tensors()
            self._pose_in = self._pose.get_input_details()[0]
            self._pose_out = self._pose.get_output_details()[0]
            self.pose_size = int(self._pose_in["shape"][1])

    @staticmethod
    def _dequantize(value: np.ndarray, detail: dict) -> np.ndarray:
        scale, zero = detail.get("quantization", (0.0, 0))
        return (value.astype(np.float32) - zero) * scale if scale else value.astype(np.float32)

    def _boxes(self, frame: np.ndarray) -> list[EdgeDetection]:
        square, scale, pad_x, pad_y = letterbox(frame, self.size)
        image = cv2.cvtColor(square, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        if self._nchw:
            image = image.transpose(2, 0, 1)
        dtype = self._yolo_in["dtype"]
        if dtype in (np.int8, np.uint8):
            q_scale, zero = self._yolo_in["quantization"]
            image = np.clip(image / q_scale + zero, np.iinfo(dtype).min, np.iinfo(dtype).max).astype(dtype)
        self._yolo.set_tensor(self._yolo_in["index"], image[None])
        self._yolo.invoke()
        raw = self._dequantize(self._yolo.get_tensor(self._yolo_out["index"]), self._yolo_out)
        return decode_yolo(raw, self.size, scale, pad_x, pad_y, frame.shape[1], frame.shape[0], self.threshold, self.nms,
                           channels=min(raw.shape[1:]))

    def _read_pose(self, frame: np.ndarray, box: tuple[float, float, float, float]) -> tuple[bool, float, tuple]:
        """Crop round the person as YOLO_Movenet/main.py does (30 % above, 15 % each side), MoveNet on it."""
        frame_h, frame_w = frame.shape[:2]
        x, y = box[1] * frame_w, box[0] * frame_h
        w, h = (box[3] - box[1]) * frame_w, (box[2] - box[0]) * frame_h
        x1, y1 = int(max(0, x - 0.15 * w)), int(max(0, y - 0.3 * h))
        x2, y2 = int(min(frame_w, x + w + 0.15 * w)), int(min(frame_h, y + h))
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return False, 0.0, ()
        image = cv2.cvtColor(cv2.resize(crop, (self.pose_size, self.pose_size)), cv2.COLOR_BGR2RGB)
        dtype = self._pose_in["dtype"]
        if dtype == np.int8:
            image = (image.astype(np.int16) - 128).astype(np.int8)
        elif dtype != np.uint8:
            image = image.astype(np.float32)
        self._pose.set_tensor(self._pose_in["index"], image[None])
        self._pose.invoke()
        points = self._dequantize(self._pose.get_tensor(self._pose_out["index"]), self._pose_out).reshape(17, 3)
        lower = float(np.mean(points[list(LOWER_BODY), 2]))
        keypoints = tuple(((x1 + float(px) * (x2 - x1)) / frame_w, (y1 + float(py) * (y2 - y1)) / frame_h, float(score))
                          for py, px, score in points)
        return lower < self.water, lower, keypoints

    def detect(self, frame: np.ndarray) -> tuple[list[EdgeDetection], list[EdgeDetection]]:
        """People (on a boat or not, with a pose for the nearest max_poses of the others), each with
        a risk level, and boats."""
        found = self._boxes(frame)
        boats = [d for d in found if d.kind == BOAT]
        people = sorted((d for d in found if d.kind == PERSON), key=lambda d: d.box[0] - d.box[2])   # tallest first
        out: list[EdgeDetection] = []
        poses = 0
        for person in people:
            if any(overlap_fraction(person.box, boat.box) >= self.on_boat for boat in boats):
                out.append(EdgeDetection(person.box, person.confidence, PERSON, on_boat=True, risk=SAFE))
                continue
            if self._pose is not None and poses < self.max_poses:
                poses += 1
                in_water, lower, keypoints = self._read_pose(frame, person.box)
                # The frame, not water, hides the legs of somebody whose box runs off its bottom edge.
                near = person.box[2] >= 1.0 - self.near_camera
                in_water = False if near else in_water
                out.append(EdgeDetection(person.box, person.confidence, PERSON, False, in_water, lower, keypoints, near,
                                         risk_level(False, in_water, keypoints, self.risk)))
            else:
                out.append(EdgeDetection(person.box, person.confidence, PERSON, risk=MEDIUM))   # no pose: MEDIUM
        return out, boats

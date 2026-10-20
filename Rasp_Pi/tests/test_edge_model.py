import unittest
from pathlib import Path

try:
    import numpy as np
    from modules.edge_model import (BOAT, CRITICAL, HIGH, MEDIUM, PERSON, SAFE, EdgeDetection, EdgeModel, RiskThresholds, decode_yolo,
                                    drowning_signs, letterbox, overlap_fraction, risk_level)
except ImportError:          # OpenCV or numpy missing
    decode_yolo = None

try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        Interpreter = None

WEIGHTS = Path(__file__).resolve().parents[2] / "YOLO_Movenet" / "weights"
VIDEO = Path(__file__).resolve().parents[2] / "YOLO_Movenet" / "videos" / "rescue_test1.mp4"


@unittest.skipIf(decode_yolo is None, "OpenCV not installed")
class DecodeTest(unittest.TestCase):
    def test_letterbox_keeps_the_aspect_ratio(self):
        square, scale, pad_x, pad_y = letterbox(np.zeros((480, 640, 3), np.uint8), 320)
        self.assertEqual(square.shape, (320, 320, 3))
        self.assertAlmostEqual(scale, 0.5)
        self.assertEqual((pad_x, pad_y), (0.0, 40.0))

    def test_normalised_boxes_map_back_to_the_frame_and_overlaps_are_suppressed(self):
        # 640 x 480 frame letterboxed to 320: scale 0.5, 40 px of padding above and below.
        raw = np.zeros((6, 3), np.float32)
        raw[:, 0] = [0.5, 0.5, 0.25, 0.25, 0.9, 0.0]     # person in the middle
        raw[:, 1] = [0.5, 0.5, 0.24, 0.25, 0.7, 0.0]     # the same person again, lower score
        raw[:, 2] = [0.2, 0.5, 0.1, 0.1, 0.0, 0.8]       # a boat on the left
        found = decode_yolo(raw, 320, 0.5, 0.0, 40.0, 640, 480, 0.5, 0.45)
        people = [d for d in found if d.kind == PERSON]
        boats = [d for d in found if d.kind == BOAT]
        self.assertEqual(len(people), 1)
        self.assertAlmostEqual(people[0].confidence, 0.9, places=5)
        ymin, xmin, ymax, xmax = people[0].box
        self.assertAlmostEqual(xmin, 0.375, places=3)
        self.assertAlmostEqual(xmax, 0.625, places=3)
        self.assertAlmostEqual(ymin, (160 - 40 - 40) * 2 / 480, places=3)
        self.assertEqual(len(boats), 1)

    def test_nothing_over_the_threshold(self):
        raw = np.zeros((6, 4), np.float32)
        raw[4] = 0.3
        self.assertEqual(decode_yolo(raw, 320, 1.0, 0.0, 0.0, 320, 320, 0.5, 0.45), [])

    def test_overlap_fraction(self):
        self.assertAlmostEqual(overlap_fraction((0.0, 0.0, 1.0, 1.0), (0.0, 0.0, 1.0, 0.5)), 0.5)
        self.assertEqual(overlap_fraction((0.0, 0.0, 0.1, 0.1), (0.5, 0.5, 1.0, 1.0)), 0.0)


@unittest.skipIf(decode_yolo is None or Interpreter is None or not (WEIGHTS / "yolov8n_640_int8.tflite").exists(),
                 "no TFLite interpreter or weights")
class ModelTest(unittest.TestCase):
    def test_finds_the_person_in_the_team_video_and_reads_a_pose(self):
        import cv2
        model = EdgeModel(Interpreter, {"model_path": str(WEIGHTS / "yolov8n_640_int8.tflite"),
                                        "pose_model_path": str(WEIGHTS / "movenet_lighting_int8.tflite"),
                                        "confidence_threshold": 0.35})
        capture = cv2.VideoCapture(str(VIDEO))
        ok, frame = capture.read()
        capture.release()
        if not ok:
            self.skipTest("cannot read the team video")
        people, _boats = model.detect(frame)
        self.assertTrue(people)
        self.assertEqual(len(people[0].keypoints), 17)
        self.assertIsNotNone(people[0].in_water)

    def test_an_empty_frame_has_nobody(self):
        model = EdgeModel(Interpreter, {"model_path": str(WEIGHTS / "yolov8n_640_int8.tflite"), "confidence_threshold": 0.35})
        people, boats = model.detect(np.full((480, 640, 3), 120, np.uint8))
        self.assertEqual((people, boats), ([], []))


@unittest.skipIf(decode_yolo is None, "OpenCV or numpy not installed")
class NearCameraTest(unittest.TestCase):
    """NEAR_CAMERA_SAFE: legs out of sight because the box runs off the bottom of the frame are not water."""

    def model(self, box, extra=(), keypoints=()):
        model = EdgeModel.__new__(EdgeModel)       # no TFLite: the two stages are stand-ins
        model.on_boat, model.max_poses, model.near_camera, model._pose = 0.4, 3, 0.03, object()
        model.risk = RiskThresholds()
        model._boxes = lambda _frame: [EdgeDetection(box, 0.5, PERSON), *extra]
        model._read_pose = lambda _frame, _box: (True, 0.05, keypoints)   # no hips, knees or ankles seen
        return model

    def test_a_box_on_the_bottom_edge_is_near_the_camera_not_in_the_water(self):
        people, _ = self.model((0.2, 0.3, 0.99, 0.7)).detect(None)
        self.assertTrue(people[0].near_camera)
        self.assertFalse(people[0].in_water)
        self.assertEqual(people[0].risk, MEDIUM)

    def test_a_box_clear_of_the_bottom_keeps_the_pose_reading(self):
        people, _ = self.model((0.4, 0.3, 0.8, 0.5)).detect(None)
        self.assertFalse(people[0].near_camera)
        self.assertTrue(people[0].in_water)
        self.assertEqual(people[0].risk, HIGH)

    def test_somebody_in_a_boat_is_safe_and_kept(self):
        people, boats = self.model((0.4, 0.3, 0.6, 0.4), extra=(EdgeDetection((0.3, 0.2, 0.7, 0.6), 0.6, BOAT),)).detect(None)
        self.assertEqual(len(boats), 1)
        self.assertTrue(people[0].on_boat)
        self.assertEqual(people[0].risk, SAFE)

    def test_people_past_max_poses_are_medium(self):
        model = self.model((0.4, 0.3, 0.8, 0.5), keypoints=raised_arms())
        model.max_poses = 0
        people, _ = model.detect(None)
        self.assertIsNone(people[0].in_water)
        self.assertEqual(people[0].risk, MEDIUM)

    def test_raised_arms_in_the_water_are_critical(self):
        people, _ = self.model((0.4, 0.3, 0.8, 0.5), keypoints=raised_arms()).detect(None)
        self.assertEqual(people[0].risk, CRITICAL)


def pose(**points):
    """17 COCO keypoints (x, y, score), none seen but those given by name."""
    names = ("nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow",
             "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle")
    return tuple(points.get(name, (0.5, 0.5, 0.0)) for name in names)


def raised_arms():
    return pose(nose=(0.5, 0.40, 0.8), left_shoulder=(0.45, 0.50, 0.7), right_shoulder=(0.55, 0.50, 0.7),
                left_wrist=(0.42, 0.30, 0.6), right_wrist=(0.58, 0.32, 0.6))


@unittest.skipIf(decode_yolo is None, "OpenCV or numpy not installed")
class RiskTest(unittest.TestCase):
    """The team's flow: boat SAFE, body seen MEDIUM, in the water HIGH, drowning signs CRITICAL."""

    def test_both_wrists_above_the_shoulders_are_a_drowning_sign(self):
        self.assertTrue(drowning_signs(raised_arms()))

    def test_one_wrist_down_is_not(self):
        points = list(raised_arms())
        points[10] = (0.58, 0.60, 0.6)           # right wrist below its shoulder
        self.assertFalse(drowning_signs(tuple(points)))

    def test_wrists_seen_too_poorly_are_not_judged(self):
        points = list(raised_arms())
        points[9] = (0.42, 0.30, 0.1)
        self.assertFalse(drowning_signs(tuple(points)))

    def test_only_the_head_above_the_water_is_a_drowning_sign(self):
        self.assertTrue(drowning_signs(pose(nose=(0.5, 0.4, 0.7), left_eye=(0.48, 0.38, 0.6))))

    def test_head_and_shoulders_seen_with_the_arms_down_is_not(self):
        calm = pose(nose=(0.5, 0.4, 0.7), left_shoulder=(0.45, 0.5, 0.6), right_shoulder=(0.55, 0.5, 0.6),
                    left_wrist=(0.42, 0.7, 0.5), right_wrist=(0.58, 0.7, 0.5))
        self.assertFalse(drowning_signs(calm))

    def test_nothing_seen_is_not(self):
        self.assertFalse(drowning_signs(pose()))
        self.assertFalse(drowning_signs(()))

    def test_risk_levels(self):
        self.assertEqual(risk_level(True, True, raised_arms()), SAFE)       # a boat wins over everything
        self.assertEqual(risk_level(False, False, raised_arms()), MEDIUM)   # lower body seen
        self.assertEqual(risk_level(False, None), MEDIUM)                   # no pose read
        self.assertEqual(risk_level(False, True, pose(nose=(0.5, 0.4, 0.1))), HIGH)
        self.assertEqual(risk_level(False, True, raised_arms()), CRITICAL)

    def test_thresholds_come_from_the_config(self):
        strict = RiskThresholds.from_config({"wrist_score": 0.9})
        self.assertFalse(drowning_signs(raised_arms(), strict))
        self.assertEqual(RiskThresholds.from_config(None), RiskThresholds())


if __name__ == "__main__":
    unittest.main()

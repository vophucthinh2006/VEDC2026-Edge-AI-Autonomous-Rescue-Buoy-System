import unittest
from pathlib import Path

try:
    import numpy as np
    from modules.edge_model import BOAT, PERSON, EdgeModel, decode_yolo, letterbox, overlap_fraction
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


@unittest.skipIf(decode_yolo is None or Interpreter is None or not (WEIGHTS / "yolov8n_int8.tflite").exists(),
                 "no TFLite interpreter or weights")
class ModelTest(unittest.TestCase):
    def test_finds_the_person_in_the_team_video_and_reads_a_pose(self):
        import cv2
        model = EdgeModel(Interpreter, {"model_path": str(WEIGHTS / "yolov8n_int8.tflite"),
                                        "pose_model_path": str(WEIGHTS / "movenet_lighting_int8.tflite"),
                                        "confidence_threshold": 0.5})
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
        model = EdgeModel(Interpreter, {"model_path": str(WEIGHTS / "yolov8n_int8.tflite"), "confidence_threshold": 0.5})
        people, boats = model.detect(np.full((480, 640, 3), 120, np.uint8))
        self.assertEqual((people, boats), ([], []))


if __name__ == "__main__":
    unittest.main()

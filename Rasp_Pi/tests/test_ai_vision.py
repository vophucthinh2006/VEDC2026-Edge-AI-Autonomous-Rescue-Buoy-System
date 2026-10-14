import threading
import unittest

try:
    from modules.ai_vision import PersonDetection, VisionWorker
except ImportError:          # OpenCV or numpy missing
    VisionWorker = None

from modules.state_store import StateStore

CAMERA = {"vertical_fov_deg": 46.8, "standing_score_min": 2.25, "distance_source": "bbox", "person_height_m": 0.57,
          "standing_height_m": 1.95, "max_target_distance_m": 12.0, "horizontal_fov_deg": 60.0}


@unittest.skipIf(VisionWorker is None, "OpenCV not installed")
class CroppedBoxTest(unittest.TestCase):
    def setUp(self):
        self.state = StateStore()
        self.worker = VisionWorker(CAMERA, self.state, threading.Event(), lambda _t: None, lambda _p: None, 2.0)
        # Somebody on a roof close by: the top of their box cut by the frame.
        self.cropped = PersonDetection(10.0, 0.5, 0.6, True, (0.0, 0.45, 0.6, 0.55))

    def test_a_cropped_box_of_somebody_new_gets_no_range(self):
        self.assertIsNone(self.worker._distance(self.cropped, 10.0))

    def test_the_person_being_gone_to_is_held_at_the_stand_off(self):
        self.state.set_focus((10.8835, 106.7961))
        self.assertEqual(self.worker._distance(self.cropped, 10.0), 2.0)

    def test_a_whole_box_is_ranged_by_its_height(self):
        swimmer = PersonDetection(0.0, 0.5, 0.11, False, (0.45, 0.47, 0.56, 0.53))
        self.assertAlmostEqual(self.worker._distance(swimmer, 0.0), 0.57 / (0.865 * 0.11), delta=0.1)


if __name__ == "__main__":
    unittest.main()

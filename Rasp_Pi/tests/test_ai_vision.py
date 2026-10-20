import logging
import threading
import unittest

try:
    from modules.ai_vision import CameraReader, PersonDetection, VisionWorker
except ImportError:          # OpenCV or numpy missing
    VisionWorker = None

from modules.perception import Obstacle
from modules.state_store import CRITICAL, HIGH, MEDIUM, SAFE, StateStore

CAMERA = {"vertical_fov_deg": 46.8, "standing_score_min": 2.45, "water_score_max": 2.1, "distance_source": "bbox", "person_height_m": 0.57,
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

    def test_a_box_that_could_be_either_gets_no_range(self):
        # 0.3 of the frame high, 0.102 wide at 4:3: h/w 2.2, bottom 0.47 deg above the horizon: scores 2.39.
        either = PersonDetection(0.0, 0.5, 0.3, False, (0.19, 0.449, 0.49, 0.551))
        self.assertIsNone(self.worker._distance(either, 0.0))


@unittest.skipIf(VisionWorker is None, "OpenCV not installed")
class FusedRangeTest(unittest.TestCase):
    def setUp(self):
        camera = dict(CAMERA, distance_source="fused", lidar_association_half_angle_deg=3.0,
                      min_target_distance_m=1.0, max_target_distance_m=12.0)
        self.state = StateStore()
        self.worker = VisionWorker(camera, self.state, threading.Event(), lambda _t: None, lambda _p: None, 2.0)
        # Somebody standing on a roof: tall narrow box, bottom above the horizon. 0.27 of the frame: about 8.3 m.
        self.roof = PersonDetection(20.0, 0.7, 0.27, False, (0.20, 0.48, 0.47, 0.52))

    def test_the_wall_under_them_gives_the_range(self):
        self.state.update_obstacles((Obstacle(20.0, 7.9, 0.3),))
        self.assertAlmostEqual(self.worker._distance(self.roof, 20.0), 7.9)

    def test_something_close_in_front_of_them_is_not_their_range(self):
        self.state.update_obstacles((Obstacle(20.5, 2.3, 0.5),))      # bamboo between the boat and the house
        self.assertAlmostEqual(self.worker._distance(self.roof, 20.0), 1.95 / (0.865 * 0.27), delta=0.2)


@unittest.skipIf(VisionWorker is None, "OpenCV not installed")
class SweepStepTest(unittest.TestCase):
    def test_the_sweep_goes_out_in_small_steps_between_detector_passes(self):
        sent = []
        camera = dict(CAMERA, scan={"enabled": True, "mode": "sweep", "sweep_limit_deg": 60, "sweep_rate_deg_s": 30})
        worker = VisionWorker(camera, StateStore(), threading.Event(), lambda _t: None, sent.append, 2.0)
        worker._pan = worker._scan.start(0.0, 0.0)
        worker._scanning = True
        for tick in range(1, 21):              # 1 s at 20 Hz, as the camera-pan thread does
            worker._sweep_step(tick * 0.05)
        steps = [abs(b - a) for a, b in zip(sent, sent[1:])]
        self.assertEqual(len(sent), 20)
        self.assertAlmostEqual(max(steps), 1.5, places=3)  # 30 deg/s in 50 ms
        self.assertAlmostEqual(abs(sent[-1]), 30.0, places=3)

    def test_no_sweep_step_while_following_somebody(self):
        sent = []
        camera = dict(CAMERA, scan={"enabled": True, "mode": "sweep", "sweep_limit_deg": 60, "sweep_rate_deg_s": 30})
        worker = VisionWorker(camera, StateStore(), threading.Event(), lambda _t: None, sent.append, 2.0)
        worker._scanning = False
        worker._sweep_step(1.0)
        self.assertEqual(sent, [])


class FakeCapture:
    def __init__(self, frames):
        self.frames, self.released = list(frames), False

    def read(self):
        return (True, "frame") if self.frames and self.frames.pop(0) else (False, None)

    def release(self):
        self.released = True


@unittest.skipIf(VisionWorker is None, "OpenCV not installed")
class CameraReaderTest(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.opened = []      # what each open returns, in order; None: the camera is not there
        self.log = logging.getLogger("test_camera")

    def reader(self):
        return CameraReader(lambda: self.opened.pop(0) if self.opened else None, "cam", 2.0, self.log,
                            clock=lambda: self.now, pause=lambda _s: None)

    def read_at(self, reader, t):
        self.now = t
        return reader.read()

    def test_a_camera_that_stops_is_released_and_reopened(self):
        first, second = FakeCapture([1, 0, 0, 0]), FakeCapture([1])
        self.opened = [first, second]
        reader = self.reader()
        self.assertIsNotNone(self.read_at(reader, 0.0))
        self.assertIsNone(self.read_at(reader, 1.0))        # a failed read inside lost_s: wait
        self.assertFalse(first.released)
        with self.assertLogs("test_camera", "WARNING") as logs:
            self.assertIsNone(self.read_at(reader, 2.5))    # past lost_s: let it go
        self.assertTrue(first.released)
        self.assertIn("no frame", logs.output[0])
        self.assertIsNone(self.read_at(reader, 3.0))        # not before lost_s has passed again
        with self.assertLogs("test_camera", "WARNING") as logs:
            self.assertIsNotNone(self.read_at(reader, 4.6))
        self.assertIn("back after 2 s", logs.output[0])

    def test_a_camera_missing_at_start_is_waited_for_with_one_warning(self):
        self.opened = [None, None, FakeCapture([1])]
        reader = self.reader()
        with self.assertLogs("test_camera", "WARNING") as logs:
            self.assertIsNone(self.read_at(reader, 0.0))
            self.assertIsNone(self.read_at(reader, 2.1))
            self.assertIsNotNone(self.read_at(reader, 4.2))
        self.assertEqual(len(logs.output), 2)               # "cannot open" once, then "back"
        self.assertIn("cannot open", logs.output[0])


@unittest.skipIf(VisionWorker is None, "OpenCV not installed")
class RiskChoiceTest(unittest.TestCase):
    """The camera stays on the highest risk, nearest first, and never on somebody in a boat."""

    def setUp(self):
        self.worker = VisionWorker(CAMERA, StateStore(), threading.Event(), lambda _t: None, lambda _p: None, 2.0)

    @staticmethod
    def person(bearing, height, risk, on_boat=False):
        return PersonDetection(bearing, 0.5, height, False, (0.4, 0.45, 0.4 + height, 0.55), risk=risk, on_boat=on_boat)

    def test_only_people_in_a_boat_is_nobody_to_follow(self):
        self.assertIsNone(self.worker._follow([self.person(0.0, 0.5, SAFE, True)], 0.0))

    def test_a_far_drowning_person_beats_a_near_one_on_the_bank(self):
        near, far = self.person(-10.0, 0.5, MEDIUM), self.person(20.0, 0.1, CRITICAL)
        self.assertIs(self.worker._follow([near, far], 0.0), far)

    def test_the_followed_one_is_kept_unless_somebody_at_a_higher_risk_shows(self):
        a, b = self.person(-10.0, 0.5, HIGH), self.person(20.0, 0.1, HIGH)
        self.worker._followed_bearing = 20.0
        self.assertIs(self.worker._follow([a, b], 0.0), b)
        c = self.person(-25.0, 0.1, CRITICAL)
        self.assertIs(self.worker._follow([a, b, c], 0.0), c)


if __name__ == "__main__":
    unittest.main()

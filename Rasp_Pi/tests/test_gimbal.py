"""The pan-tilt gimbal: the raster sweep, following in tilt, people on a roof and the stand-off from them."""
import threading
import unittest
from math import cos, radians
from time import monotonic

try:
    from modules.ai_vision import PersonDetection, VisionWorker
except ImportError:          # OpenCV or numpy missing
    VisionWorker = None

from modules.navigation import BapfNavigator
from modules.perception import RasterSweep, SweepPattern, make_scan, track_pan
from modules.state_store import GpsState, HumanTarget, ImuState, Snapshot, StateStore, SysState

CAMERA = {"vertical_fov_deg": 47.0, "standing_score_min": 2.5, "water_score_max": 2.5, "distance_source": "bbox", "person_height_m": 0.57,
          "standing_height_m": 1.7, "max_target_distance_m": 30.0, "horizontal_fov_deg": 60.0, "pan_gain": 0.25,
          "pan_deadband_deg": 2.0, "pan_limit_deg": 45.0, "tilt_limit_deg": [0, 60], "tilt_home_deg": 10.0, "roof_min_deg": 5.0}


class RasterSweepTest(unittest.TestCase):
    def run_sweep(self, seconds, dt=0.05):
        sweep = RasterSweep(45, 15, [0, 30])
        sweep.start(0.0, 0.0)
        samples = [(t * dt, sweep.frame_done(t * dt), sweep.tilt_deg) for t in range(1, int(seconds / dt) + 1)]
        return sweep, samples

    def test_the_row_changes_at_each_end_of_the_pan(self):
        _, samples = self.run_sweep(30.0)
        changes = [(pan, tilt) for (_, _, before), (_, pan, tilt) in zip(samples, samples[1:]) if tilt != before]
        self.assertGreaterEqual(len(changes), 4)
        for pan, _ in changes:
            self.assertAlmostEqual(abs(pan), 45.0, delta=15 * 0.05 + 1e-6)
        self.assertEqual({tilt for _, _, tilt in samples}, {0.0, 30.0})

    def test_a_round_of_both_rows_takes_12_s(self):
        _, samples = self.run_sweep(40.0)
        ups = [t for (_, _, before), (t, _, tilt) in zip(samples, samples[1:]) if before == 0.0 and tilt == 30.0]
        self.assertAlmostEqual(ups[1] - ups[0], 12.0, delta=0.1)

    def test_the_tilt_stays_in_the_gimbal_range_and_starts_on_the_water(self):
        sweep, samples = self.run_sweep(60.0)
        self.assertTrue(all(0.0 <= tilt <= 60.0 for _, _, tilt in samples))
        self.assertEqual(sweep.start(100.0, 20.0), 20.0)
        self.assertEqual(sweep.tilt_deg, 0.0)

    def test_frames_are_dropped_while_the_tilt_moves(self):
        sweep = RasterSweep(45, 15, [0, 30], settle_s=0.6)
        sweep.start(0.0, -44.0)                 # 1 deg from the end: the row changes after 1/15 s
        sweep.frame_done(0.1)
        self.assertEqual(sweep.tilt_deg, 30.0)
        self.assertFalse(sweep.usable(0.5))
        self.assertTrue(sweep.usable(0.75))

    def test_make_scan_builds_it_and_the_others_stay(self):
        scan = {"enabled": True, "sweep_limit_deg": 45, "sweep_rate_deg_s": 15, "tilt_rows_deg": [0, 30]}
        self.assertIsInstance(make_scan(dict(scan, mode="raster")), RasterSweep)
        self.assertNotIsInstance(make_scan(dict(scan, mode="sweep")), RasterSweep)
        self.assertIsInstance(make_scan(dict(scan, mode="sweep")), SweepPattern)

    def test_track_pan_keeps_the_tilt_in_its_own_range(self):
        self.assertEqual(track_pan(10.0, -40.0, 0.5, 2.0, 60.0, 0.0), 0.0)
        self.assertEqual(track_pan(50.0, 40.0, 0.5, 2.0, 60.0, 0.0), 60.0)
        self.assertEqual(track_pan(10.0, 1.0, 0.5, 2.0, 60.0, 0.0), 10.0)       # deadband
        self.assertEqual(track_pan(-40.0, -40.0, 0.5, 2.0, 45.0), -45.0)       # the pan as before


@unittest.skipIf(VisionWorker is None, "OpenCV not installed")
class GimbalVisionTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.worker = VisionWorker(CAMERA, StateStore(), threading.Event(), lambda _t: None,
                                   lambda *a: self.sent.append(a), 2.0, tilt=True)

    def test_elevation_is_from_the_box_centre(self):
        person = self.worker._person((0.1, 0.4, 0.3, 0.6), 0.8)   # centre 0.2 of the frame from the top
        self.assertAlmostEqual(person.elevation_cam_deg, 0.3 * 47.0)

    def test_up_to_a_roof_the_range_is_the_one_along_the_water(self):
        # Standing (tall, narrow), whole box 0.2 of the frame high, centre 10 deg above the axis, tilt 30.
        roof = self.worker._person((0.187, 0.48, 0.387, 0.52), 0.8)
        slant = 1.7 / (2 * 0.4348 * 0.2)
        elevation = 30.0 + roof.elevation_cam_deg
        self.assertAlmostEqual(self.worker._distance(roof, 0.0, tilt_deg=30.0), slant * cos(radians(elevation)), delta=0.05)

    def test_feet_above_the_horizon_are_on_a_roof(self):
        feet_on_axis = self.worker._person((0.3, 0.48, 0.5, 0.52), 0.8)
        self.assertFalse(self.worker._on_roof(feet_on_axis, 0.0))   # bottom at the horizon: the water
        self.assertTrue(self.worker._on_roof(feet_on_axis, 10.0))   # tilted 10 deg up: feet 10 deg above it
        swimmer = self.worker._person((0.5, 0.48, 0.6, 0.52), 0.8)
        self.assertFalse(self.worker._on_roof(swimmer, 0.0))

    def test_a_low_roof_far_off_is_a_roof_by_the_height_of_the_feet(self):
        # Feet 1 deg above the horizon, under roof_min_deg: 0.4 + 13 m x tan(0.94 deg) = 0.61 m above the water.
        feet = self.worker._person((0.28, 0.49, 0.48, 0.51), 0.8)
        self.assertFalse(self.worker._on_roof(feet, 0.0))
        self.assertTrue(self.worker._on_roof(feet, 0.0, 13.0))
        self.assertFalse(self.worker._on_roof(feet, 0.0, 3.0))       # 0.45 m: a doorstep
        swimmer = self.worker._person((0.5, 0.48, 0.58, 0.52), 0.8)
        self.assertFalse(self.worker._on_roof(swimmer, 0.0, 8.0))

    def test_a_wide_box_above_the_waterline_gets_no_swimmer_range(self):
        # Wide and short, so "a swimmer", but its bottom is 1 deg above the horizon: a swimmer 8 m away
        # would have theirs 2.8 deg below it. People side by side on a roof: no range, not a 3x short one.
        group = self.worker._person((0.40, 0.40, 0.48, 0.52), 0.8)
        self.assertIsNone(self.worker._distance(group, 0.0))
        swimmer = self.worker._person((0.50, 0.48, 0.58, 0.52), 0.8)
        self.assertAlmostEqual(self.worker._distance(swimmer, 0.0), 0.57 / (2 * 0.4348 * 0.08), delta=0.3)

    def test_somebody_standing_is_ranged_past_max_target_distance(self):
        worker = VisionWorker(dict(CAMERA, max_target_distance_m=8.0, max_standing_distance_m=20.0), StateStore(),
                              threading.Event(), lambda _t: None, lambda *a: None, 2.0, tilt=True)
        far = worker._person((0.37, 0.49, 0.49, 0.51), 0.8)          # tall and narrow, 1.7 m over 0.12 of the frame: 16 m
        self.assertAlmostEqual(worker._distance(far, 0.0), 1.7 / (2 * 0.4348 * 0.12), delta=0.5)
        self.assertIsNone(VisionWorker(dict(CAMERA, max_target_distance_m=8.0), StateStore(), threading.Event(), lambda _t: None,
                                       lambda *a: None, 2.0)._distance(far, 0.0))

    def test_the_tilt_goes_out_with_the_pan_and_follows_the_raster(self):
        worker = VisionWorker(dict(CAMERA, scan={"enabled": True, "mode": "raster", "sweep_limit_deg": 45, "sweep_rate_deg_s": 15,
                                                 "tilt_rows_deg": [0, 30]}),
                              StateStore(), threading.Event(), lambda _t: None, lambda *a: self.sent.append(a), 2.0, tilt=True)
        worker._pan = worker._scan.start(0.0, -44.0)
        worker._scanning = True
        for tick in range(1, 21):
            worker._sweep_step(tick * 0.05)
        self.assertTrue(all(len(a) == 2 for a in self.sent))
        self.assertEqual(self.sent[0][1], 0.0)
        self.assertEqual(self.sent[-1][1], 30.0)

    def test_without_a_gimbal_only_the_pan_goes_out(self):
        sent = []
        worker = VisionWorker(CAMERA, StateStore(), threading.Event(), lambda _t: None, lambda *a: sent.append(a), 2.0)
        worker._publish_pan(1.0)
        self.assertEqual(sent, [(0.0,)])

    def test_out_of_a_sweep_the_tilt_comes_home(self):
        camera = dict(CAMERA, recenter_step_deg=3.0)
        worker = VisionWorker(camera, StateStore(), threading.Event(), lambda _t: None, lambda *a: None, 2.0, tilt=True)
        worker._tilt = 30.0
        for _ in range(10):
            worker._recenter()
        self.assertEqual(worker._tilt, 10.0)


class RoofStandoffTest(unittest.TestCase):
    vehicle = {"boat_radius_m": 0.35, "safety_margin_m": 0.4, "influence_distance_m": 3.0, "target_standoff_m": 1.5,
               "approach_speed_mps": 0.2, "max_speed_mps": 0.5, "arrive_radius_m": 1.0, "stop_distance_m": 0.8,
               "slow_distance_m": 2.0, "turn_room_m": 0.4, "roof_standoff_m": 8.0}
    planner = {"attractive_gain": 1.0, "repulsive_gain": 0.75, "bias_gain": 0.22}

    def _person(self, distance, on_roof):
        now = monotonic()
        return Snapshot(ImuState(yaw_deg=0.0, imu_ok=True, timestamp=now), GpsState(lat_deg=10, lon_deg=106, fix=3, timestamp=now),
                        SysState(timestamp=now), (), now, None, HumanTarget(0.0, distance, 0.8, now, on_roof=on_roof))

    def test_somebody_on_a_roof_is_held_off_at_the_roof_standoff(self):
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(self._person(7.0, True)).reason, "human_standoff")
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(self._person(12.0, True)).reason, "approach_human")
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(self._person(7.0, False)).reason, "approach_human")


if __name__ == "__main__":
    unittest.main()

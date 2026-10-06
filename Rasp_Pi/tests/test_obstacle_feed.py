import unittest

from modules.obstacle_feed import objects_from, sectors
from modules.perception import Obstacle, ScanPattern, SweepPattern, box_bottom_elevation_deg, make_scan, ray_obstacles, standing_score
from modules.state_store import GpsState, ImuState, Snapshot, SysState
from utils.geometry import offset_latlon

CLEAR = 801   # 8 m range: 800 cm + 1


class SectorsTest(unittest.TestCase):
    def test_no_objects_is_all_clear(self):
        self.assertEqual(sectors([], 0.15, 8.0), [CLEAR] * 72)

    def test_object_ahead_fills_the_bow_sectors_with_the_range_to_its_edge(self):
        out = sectors([(0.0, 4.0, 0.5)], 0.15, 8.0)       # half-angle asin(0.5 / 4) = 7.2 deg
        self.assertEqual(out[0], 350)
        self.assertEqual(out[1], 350)
        self.assertEqual(out[71], 350)
        self.assertEqual(out[2], CLEAR)
        self.assertEqual(out[70], CLEAR)

    def test_bearing_is_clockwise_from_the_bow(self):
        out = sectors([(90.0, 3.0, 0.1)], 0.15, 8.0)      # starboard beam
        self.assertEqual(out[18], 290)
        self.assertEqual(out[54], CLEAR)                  # port beam untouched

    def test_out_of_range_and_inside_cases(self):
        self.assertEqual(sectors([(0.0, 20.0, 0.5)], 0.15, 8.0), [CLEAR] * 72)
        self.assertEqual(sectors([(0.0, 0.3, 1.0)], 0.15, 8.0), [15] * 72)   # boat inside the circle

    def test_nearest_object_wins_a_shared_sector(self):
        out = sectors([(0.0, 6.0, 0.2), (2.0, 3.0, 0.2)], 0.15, 8.0)
        self.assertEqual(out[0], 280)


class ObjectsFromTest(unittest.TestCase):
    def test_keep_out_is_placed_relative_to_the_bow(self):
        lat, lon = 10.883438, 106.796019
        victim = offset_latlon(lat, lon, 90.0, 5.0)        # 5 m east
        snap = Snapshot(ImuState(yaw_deg=0.0, imu_ok=True), GpsState(lat_deg=lat, lon_deg=lon, fix=3), SysState(),
                        (Obstacle(-30.0, 2.0, 0.4),), 0.0, None, None)   # boat heading north
        found = objects_from(snap, [victim], 1.0)
        self.assertEqual(found[0], (-30.0, 2.0, 0.2))      # LiDAR cluster: radius is half its width
        self.assertAlmostEqual(found[1][0], 90.0, delta=0.5)
        self.assertAlmostEqual(found[1][1], 5.0, delta=0.05)
        self.assertEqual(found[1][2], 1.0)

    def test_the_person_being_attended_is_not_an_obstacle(self):
        lat, lon = 10.883438, 106.796019
        victim = offset_latlon(lat, lon, 0.0, 2.0)         # 2 m ahead of a boat heading north
        snap = Snapshot(ImuState(yaw_deg=0.0, imu_ok=True), GpsState(lat_deg=lat, lon_deg=lon, fix=3), SysState(),
                        (Obstacle(2.0, 1.9, 0.05), Obstacle(40.0, 2.0, 0.05)), 0.0, None, None)
        self.assertEqual(objects_from(snap, [], 0.8, attending=victim), [(40.0, 2.0, 0.025)])
        self.assertEqual(len(objects_from(snap, [], 0.8)), 2)

    def test_no_keep_outs_without_a_fix(self):
        snap = Snapshot(ImuState(), GpsState(fix=0), SysState(), (), 0.0, None, None)
        self.assertEqual(objects_from(snap, [(10.0, 106.0)], 1.0), [])


class RayObstaclesTest(unittest.TestCase):
    def test_a_wall_keeps_its_shape(self):
        # A wall 3 m ahead, square to the bow: the range grows as 3 / cos(bearing).
        import math
        rays = [(float(b), 3.0 / math.cos(math.radians(b))) for b in range(-40, 41)]
        out = sectors([(o.bearing_body_deg, o.distance_m, o.width_m / 2.0) for o in ray_obstacles(rays)], 0.15, 8.0)
        self.assertAlmostEqual(out[0], 300, delta=5)
        # A sector reports its nearest return: the 30 deg sector spans 27.5 to 32.5 deg.
        self.assertAlmostEqual(out[6], 300 / math.cos(math.radians(27.5)), delta=6)     # starboard
        self.assertAlmostEqual(out[66], 300 / math.cos(math.radians(27.5)), delta=6)    # port
        self.assertEqual(out[12], CLEAR)                                               # beyond the wall's end

    def test_a_lone_return_is_dropped(self):
        self.assertEqual(ray_obstacles([(10.0, 4.0)]), ())
        self.assertEqual(ray_obstacles([(10.0, 4.0), (11.0, 6.5)]), ())          # neighbours in angle, not in range
        self.assertEqual(len(ray_obstacles([(10.0, 4.0), (11.0, 4.05)])), 2)

    def test_range_limits_and_the_wrap_at_the_stern(self):
        self.assertEqual(ray_obstacles([(0.0, 0.05), (1.0, 0.06), (5.0, 9.0), (6.0, 9.0)]), ())
        self.assertEqual(len(ray_obstacles([(179.5, 2.0), (-179.5, 2.0)])), 2)


class SweepPatternTest(unittest.TestCase):
    def test_turns_at_a_constant_rate_and_reverses_at_the_limits(self):
        sweep = SweepPattern(60, 30)
        self.assertEqual(sweep.start(0.0, 0.0), 0.0)
        self.assertAlmostEqual(sweep.frame_done(1.0), -30.0)
        self.assertAlmostEqual(sweep.frame_done(2.0), -60.0)
        self.assertAlmostEqual(sweep.frame_done(3.0), -30.0)          # on the way back
        self.assertAlmostEqual(sweep.frame_done(6.0), 60.0)
        self.assertAlmostEqual(sweep.frame_done(6.5), 45.0)
        self.assertAlmostEqual(sweep.frame_done(14.5), 45.0)          # one full cycle later: 8 s at 30 deg/s

    def test_small_steps_between_frames(self):
        sweep = SweepPattern(60, 30)
        sweep.start(0.0, 0.0)
        angles = [sweep.frame_done(i / 15.0) for i in range(1, 60)]   # one command per camera frame
        self.assertLessEqual(max(abs(b - a) for a, b in zip(angles, angles[1:])), 2.01)
        self.assertTrue(sweep.usable(0.0))                            # no frame is thrown away

    def test_resumes_from_where_the_camera_points(self):
        sweep = SweepPattern(60, 30)
        self.assertEqual(sweep.start(0.0, 48.0), 48.0)                # was following somebody to starboard
        self.assertAlmostEqual(sweep.frame_done(0.2), 54.0)           # finishes that side first
        self.assertEqual(sweep.start(0.0, -75.0), -60.0)              # beyond the sweep: comes back to its end

    def test_make_scan_picks_the_pattern(self):
        base = {"enabled": True, "angles_deg": [0, -60, 0, 60], "settle_s": 0.25, "dwell_frames": 3,
                "sweep_limit_deg": 60, "sweep_rate_deg_s": 30}
        self.assertIsInstance(make_scan(base), ScanPattern)
        self.assertIsInstance(make_scan({**base, "mode": "sweep"}), SweepPattern)
        self.assertIsNone(make_scan({**base, "enabled": False}))
        self.assertIsNone(make_scan(None))


class BoxElevationTest(unittest.TestCase):
    def test_swimmer_is_below_the_horizon_and_a_person_on_a_roof_above_it(self):
        # Camera 0.3 m above the water, 47 deg vertical field of view, level boat.
        # Swimmer 4 m away: waterline 4.3 deg below the horizon, so the box ends at 0.5 + 4.3 / 47.
        self.assertAlmostEqual(box_bottom_elevation_deg(0.5 + 4.3 / 47.0, 47.0, 0.0), -4.3, places=3)
        # Feet on a roof 0.25 m above the camera, 9 m away: 1.6 deg above it.
        self.assertGreater(box_bottom_elevation_deg(0.5 - 1.6 / 47.0, 47.0, 0.0), 0.5)

    def test_standing_score_separates_a_roof_from_a_swimmer_even_with_the_boat_pitching(self):
        def box(height_over_width, bottom_deg, height=0.2):
            width = height / height_over_width * 3 / 4          # 4:3 frame
            ymax = 0.5 - bottom_deg / 47.0
            return (ymax - height, 0.5 - width / 2, ymax, 0.5 + width / 2)
        frame = 4 / 3
        # The two hardest cases measured at 8 m, each with 2 deg of unaccounted pitch against it.
        swimmer = standing_score(box(1.87, -1.4 + 2.0), frame, 47.0, 0.0)
        on_roof = standing_score(box(3.44, 1.3 - 2.0), frame, 47.0, 0.0)
        self.assertLess(swimmer, 2.5)
        self.assertGreater(on_roof, 2.5)
        # Near, they are far apart.
        self.assertLess(standing_score(box(1.16, -9.6, 0.4), frame, 47.0, 0.0), 0.0)
        self.assertGreater(standing_score(box(2.53, 2.9, 0.44), frame, 47.0, 0.0), 3.0)

    def test_pitch_is_taken_out(self):
        # Bow down 3 deg: the swimmer's box moves up the frame by 3 deg, and still reads as below the horizon.
        self.assertAlmostEqual(box_bottom_elevation_deg(0.5 + (4.3 - 3.0) / 47.0, 47.0, -3.0), -4.3, places=3)


class ScanPatternTest(unittest.TestCase):
    def test_steps_through_the_angles_after_the_dwell(self):
        scan = ScanPattern([0, -60, 0, 60], settle_s=0.25, dwell_frames=3)
        self.assertEqual(scan.start(10.0, 0.0), 0.0)
        seen = []
        now = 10.0
        for _ in range(12):
            now += 0.3
            seen.append(scan.frame_done(now))
        self.assertEqual(seen, [0.0, 0.0, -60.0, -60.0, -60.0, 0.0, 0.0, 0.0, 60.0, 60.0, 60.0, 0.0])

    def test_frames_while_the_servo_moves_are_not_usable(self):
        scan = ScanPattern([0, -60, 0, 60], settle_s=0.25, dwell_frames=1)
        scan.start(10.0, 0.0)
        self.assertFalse(scan.usable(10.1))
        self.assertTrue(scan.usable(10.3))
        scan.frame_done(10.3)                      # steps to -60
        self.assertFalse(scan.usable(10.4))
        self.assertTrue(scan.usable(10.6))

    def test_resumes_from_the_angle_nearest_to_the_camera(self):
        scan = ScanPattern([0, -60, 0, 60], settle_s=0.25, dwell_frames=3)
        self.assertEqual(scan.start(0.0, 48.0), 60.0)       # was following somebody to starboard
        self.assertEqual(scan.start(0.0, -70.0), -60.0)


if __name__ == "__main__":
    unittest.main()

import tempfile
import unittest
from pathlib import Path

from tools.lake_report import analyse, cross_track_m, heading_error, legs, load, sample, speeds, turns
from utils.geometry import offset_latlon

LAT, LON = 10.77, 106.66


def r(t, yaw=0.0, cmd=0.0, speed=0.5, cmd_speed=0.5, lat=LAT, lon=LON, wp="", wp_pos=None):
    return {"t": t, "lat": lat, "lon": lon, "fix": 3, "speed": speed, "yaw": yaw, "course": yaw, "mode": "AUTO",
            "cmd_speed": cmd_speed, "cmd_heading": cmd, "reason": "", "wp": wp,
            "wp_lat": wp_pos[0] if wp_pos else None, "wp_lon": wp_pos[1] if wp_pos else None}


class LakeReportTest(unittest.TestCase):
    def test_heading_error_wraps_through_north(self):
        self.assertAlmostEqual(heading_error(r(0, yaw=355.0, cmd=5.0)), -10.0)
        self.assertAlmostEqual(heading_error(r(0, yaw=5.0, cmd=355.0)), 10.0)

    def test_cross_track_is_the_distance_off_the_leg_right_positive(self):
        end = offset_latlon(LAT, LON, 0.0, 50.0)
        east = offset_latlon(*offset_latlon(LAT, LON, 0.0, 20.0), 90.0, 2.0)
        self.assertAlmostEqual(cross_track_m((LAT, LON), end, *east), 2.0, delta=0.01)
        west = offset_latlon(*offset_latlon(LAT, LON, 0.0, 20.0), 270.0, 2.0)
        self.assertAlmostEqual(cross_track_m((LAT, LON), end, *west), -2.0, delta=0.01)

    def test_a_turn_settles_and_overshoots(self):
        # Command steps 0 -> 90 at t=1; yaw swings to 100 (10 past), then holds 90.
        yaws = [0.0] * 10 + [30.0, 60.0, 90.0, 100.0, 100.0] + [90.0] * 40
        rows = [r(i * 0.1, yaw=y, cmd=0.0 if i < 10 else 90.0) for i, y in enumerate(yaws)]
        found = turns(rows)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0]["step"], 90.0)
        self.assertAlmostEqual(found[0]["overshoot"], 10.0)
        self.assertAlmostEqual(found[0]["settle"], 0.5, places=5)   # yaw 90 at 1.2 s, but 100 (error 10, not < 10) follows; held from 1.5 s

    def test_a_turn_never_settled(self):
        rows = [r(i * 0.1, yaw=0.0, cmd=0.0 if i < 10 else 90.0) for i in range(100)]
        self.assertIsNone(turns(rows)[0]["settle"])

    def test_legs_start_at_the_previous_waypoint_and_end_when_the_boat_moves_on(self):
        a, b = offset_latlon(LAT, LON, 0.0, 20.0), offset_latlon(LAT, LON, 45.0, 30.0)
        near_a = offset_latlon(*a, 180.0, 2.5)
        rows = [r(0, wp="1/2", wp_pos=a), r(1, wp="1/2", wp_pos=a, lat=near_a[0], lon=near_a[1]),
                r(2, wp="2/2", wp_pos=b, lat=near_a[0], lon=near_a[1]), r(3, wp="2/2", wp_pos=b)]
        out = legs(rows)
        self.assertEqual([x["wp"] for x in out], ["1/2", "2/2"])
        self.assertEqual(out[0]["start"], (LAT, LON))
        self.assertEqual(out[1]["start"], a)
        self.assertAlmostEqual(out[0]["end_m"], 2.5, delta=0.01)
        self.assertEqual((out[0]["reached"], out[1]["reached"]), (True, False))

    def test_speed_counts_once_the_command_has_stood(self):
        rows = [r(i * 0.1, speed=0.4) for i in range(60)]
        out = speeds(rows)
        self.assertEqual((out[0]["cmd"], out[0]["n"]), (0.5, 30))
        self.assertAlmostEqual(out[0]["ratio"], 0.8)

    def test_the_sample_square_passes_end_to_end(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "s.csv"
            sample(path)
            result = analyse(load([path]))
        self.assertEqual(len(result["legs"]), 4)
        self.assertEqual(len(result["turns"]), 3)
        self.assertTrue(all(ok for _n, _v, ok, _lo, _hi in result["checks"]), result["checks"])
        self.assertAlmostEqual(result["values"]["compass_offset"], -3.0, delta=0.5)


if __name__ == "__main__":
    unittest.main()

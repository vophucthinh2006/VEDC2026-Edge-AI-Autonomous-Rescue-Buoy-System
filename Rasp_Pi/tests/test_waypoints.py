import tempfile
import unittest
from pathlib import Path
from time import monotonic

from modules.navigation import NavCommand
from modules.safety import SafetyArbiter
from modules.state_store import GpsState, ImuState, Snapshot, SysState
from modules.waypoints import Route, load_route, read_qgc_wpl
from utils.geometry import offset_latlon

A = (10.762600, 106.660100)
B = offset_latlon(*A, 90.0, 20.0)      # 20 m east of A
FENCE = [offset_latlon(*A, bearing, 30.0) for bearing in (315, 45, 135, 225)]   # a 42 m square round A

WPL = """QGC WPL 110
0\t1\t0\t16\t0\t0\t0\t0\t10.7600000\t106.6600000\t5.0\t1
1\t0\t3\t16\t0\t0\t0\t0\t10.7626000\t106.6601000\t0.0\t1
2\t0\t3\t178\t1\t0.5\t0\t0\t0\t0\t0\t1
3\t0\t3\t16\t0\t0\t0\t0\t10.7627000\t106.6602000\t0.0\t1
"""


class RouteTest(unittest.TestCase):
    def test_moves_on_inside_the_arrive_radius_and_loops(self):
        route = Route([A, B], arrive_radius_m=3.0, loop=True)
        self.assertEqual(route.current, A)
        self.assertFalse(route.update(*offset_latlon(*A, 0.0, 5.0)))     # 5 m off: not there yet
        self.assertTrue(route.update(*offset_latlon(*A, 0.0, 2.0)))
        self.assertEqual(route.current, B)
        self.assertTrue(route.update(*B))
        self.assertEqual(route.current, A)                              # round again
        self.assertEqual(route.laps, 1)

    def test_without_loop_the_route_ends(self):
        route = Route([A], arrive_radius_m=3.0, loop=False)
        route.update(*A)
        self.assertIsNone(route.current)

    def test_reads_the_waypoints_of_a_mission_planner_file_and_leaves_home_out(self):
        self.assertEqual(read_qgc_wpl(WPL), [(10.7626, 106.6601), (10.7627, 106.6602)])

    def test_loads_yaml_with_a_fence(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "route.yaml"
            path.write_text(f"loop: false\nwaypoints:\n  - [{A[0]}, {A[1]}]\ngeofence:\n"
                            + "".join(f"  - [{lat}, {lon}]\n" for lat, lon in FENCE), encoding="utf-8")
            route = load_route(path, 3.0)
        self.assertEqual(route.waypoints, [A])
        self.assertFalse(route.loop)
        self.assertTrue(route.inside_fence(*A))
        self.assertFalse(route.inside_fence(*offset_latlon(*A, 0.0, 40.0)))

    def test_no_fence_is_always_inside(self):
        self.assertTrue(Route([], 3.0).inside_fence(0.0, 0.0))


class GeofenceTest(unittest.TestCase):
    def snap(self, position):
        now = monotonic()
        return Snapshot(ImuState(yaw_deg=0, imu_ok=True, timestamp=now), GpsState(*position, fix=3, timestamp=now),
                        SysState(timestamp=now), (), now, None, None)

    def test_the_boat_stops_outside_the_fence(self):
        route = Route([], 3.0, geofence=FENCE)
        safety = SafetyArbiter({"max_pitch_deg": 35, "max_roll_deg": 35, "imu_stale_s": 1.0, "lidar_required": False,
                                "lidar_stale_s": 1.0, "gps_stale_s": 2.0}, route.inside_fence)
        go = NavCommand("AUTO", 0.2, 90.0, "approach_human")
        self.assertEqual(safety.check(self.snap(A), go), go)
        self.assertEqual(safety.check(self.snap(offset_latlon(*A, 0.0, 40.0)), go).reason, "outside_geofence")


if __name__ == "__main__":
    unittest.main()

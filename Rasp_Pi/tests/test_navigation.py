import unittest
from time import monotonic

from modules.navigation import BapfNavigator, PaperBapfNavigator, VoNavigator, cpa
from modules.perception import Obstacle
from modules.state_store import GpsState, ImuState, Snapshot, SysState


class NavigationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.vehicle = {"boat_radius_m": 0.35, "safety_margin_m": 0.4, "influence_distance_m": 3.0, "target_standoff_m": 1.5, "approach_speed_mps": 0.2, "max_speed_mps": 0.5, "arrive_radius_m": 1.0, "stop_distance_m": 0.8, "slow_distance_m": 2.0}
        self.planner = {"attractive_gain": 1.0, "repulsive_gain": 0.75, "bias_gain": 0.22}

    def _snap(self, obstacles):
        now = monotonic()
        return Snapshot(ImuState(yaw_deg=0, imu_ok=True, timestamp=now), GpsState(lat_deg=10, lon_deg=106, fix=3, timestamp=now), SysState(timestamp=now), tuple(obstacles), now, (10.001, 106), None)

    def test_close_obstacle_ahead_is_left_at_a_creep_not_waited_out(self) -> None:
        # 0.8 m dead ahead, the waypoint behind it. The field points away from it: the boat used to
        # stop here for good, now it follows that course slowly.
        command = BapfNavigator(self.vehicle, self.planner).plan(self._snap([Obstacle(0, 0.8, 0.2)]))
        self.assertEqual(command.mode, "AUTO")
        self.assertEqual(command.reason, "leaving_obstacle")
        self.assertGreater(abs((command.heading_deg + 180.0) % 360.0 - 180.0), 90.0)     # away from the obstacle
        self.assertAlmostEqual(command.speed_mps, 0.3 * 0.5)

    def test_stops_when_the_course_leads_onto_a_close_obstacle(self) -> None:
        # Boxed in on all sides: the pushes cancel, the course stays on the waypoint and on the obstacle ahead.
        ring = [Obstacle(float(b), 0.8, 0.2) for b in range(-180, 180, 30)]
        command = BapfNavigator(self.vehicle, {**self.planner, "bias_gain": 0.0}).plan(self._snap(ring))
        self.assertEqual(command.mode, "STOP")
        self.assertEqual(command.reason, "obstacle_too_close")

    def test_far_obstacle_changes_nothing(self) -> None:
        command = BapfNavigator(self.vehicle, self.planner).plan(self._snap([Obstacle(0, 6.0, 0.2)]))
        self.assertEqual((command.mode, command.reason), ("AUTO", "waypoint"))
        self.assertAlmostEqual(command.speed_mps, 0.5)


VEHICLE = {"boat_radius_m": 0.55, "max_speed_mps": 1.0, "arrive_radius_m": 2.0}
BAPF = {"a": 2.0, "b": 0.25, "bias_a": 1.5, "bias_b": 0.2, "bias_offset_m": 0.73, "att_max": 4.0, "control_max": 8.0,
        "tcpa_max_s": 8.0, "dcpa_min_m": 1.5}
VO = {"radius_m": 1.0, "tcpa_max_s": 6.0, "heading_step_deg": 5.0, "switch_deg": 20.0}


def snap_with(obstacles, yaw=0.0, speed=1.0, course=0.0):
    """Boat at (10, 106) heading north, waypoint 100 m north."""
    now = monotonic()
    return Snapshot(ImuState(yaw_deg=yaw, imu_ok=True, timestamp=now),
                    GpsState(lat_deg=10.0, lon_deg=106.0, fix=3, speed_mps=speed, course_deg=course, timestamp=now),
                    SysState(timestamp=now), tuple(obstacles), now, (10.0009, 106.0), None)


def turn(command, yaw=0.0):
    return (command.heading_deg - yaw + 180.0) % 360.0 - 180.0      # positive: to starboard


class CpaTest(unittest.TestCase):
    def test_point_ahead_and_point_abeam(self):
        self.assertEqual(cpa(5.0, 0.0, 1.0, 0.0), (5.0, 0.0))            # dead ahead: reached in 5 s
        tcpa, dcpa = cpa(5.0, 3.0, 1.0, 0.0)
        self.assertAlmostEqual(tcpa, 5.0)
        self.assertAlmostEqual(dcpa, 3.0)                                # passes 3 m to the side
        self.assertLess(cpa(-4.0, 0.0, 1.0, 0.0)[0], 0.0)                # astern: already past


class PaperBapfTest(unittest.TestCase):
    def test_open_water_goes_straight_for_the_waypoint(self):
        command = PaperBapfNavigator(VEHICLE, BAPF).plan(snap_with([]))
        self.assertAlmostEqual(turn(command), 0.0, delta=1.0)
        self.assertAlmostEqual(command.speed_mps, 1.0, delta=0.05)

    def test_obstacle_dead_ahead_is_passed_to_starboard(self):
        # Head on, the plain field pushes straight back and the boat would stall; the bias breaks the tie.
        command = PaperBapfNavigator(VEHICLE, BAPF).plan(snap_with([Obstacle(0.0, 2.5, 0.1)]))
        self.assertGreater(turn(command), 5.0)

    def test_a_wall_alongside_does_not_push(self):
        # Returns 3 m to port, the boat running parallel to them: DCPA 3 m is over the gate.
        wall = [Obstacle(-90.0 + a, 3.0 / max(0.2, abs(__import__("math").cos(__import__("math").radians(a)))), 0.1) for a in (-40, -20, 0, 20, 40)]
        command = PaperBapfNavigator(VEHICLE, BAPF).plan(snap_with(wall))
        self.assertAlmostEqual(turn(command), 0.0, delta=1.0)

    def test_arrived(self):
        now = monotonic()
        snap = Snapshot(ImuState(imu_ok=True, timestamp=now), GpsState(lat_deg=10.0, lon_deg=106.0, fix=3, timestamp=now),
                        SysState(timestamp=now), (), now, (10.00001, 106.0), None)
        self.assertEqual(PaperBapfNavigator(VEHICLE, BAPF).plan(snap).mode, "STOP")


class VoTest(unittest.TestCase):
    def test_open_water_goes_straight_for_the_waypoint(self):
        command = VoNavigator(VEHICLE, VO).plan(snap_with([]))
        self.assertAlmostEqual(turn(command), 0.0, delta=0.1)
        self.assertEqual(command.reason, "waypoint")

    def test_picks_the_nearest_heading_that_misses_the_obstacle(self):
        # A point 4 m ahead, 0.3 m to starboard: passing 1 m clear needs about 11 deg to port
        # (asin(1 / 4) = 14.5 deg from its bearing of 4.3 deg), fewer than to starboard.
        command = VoNavigator(VEHICLE, VO).plan(snap_with([Obstacle(4.3, 4.0, 0.1)]))
        self.assertLess(turn(command), -9.0)   # the waypoint is 100 m off: the full horizon applies
        self.assertGreater(turn(command), -21.0)
        self.assertEqual(command.reason, "avoid")

    def test_an_obstacle_off_the_course_is_ignored(self):
        command = VoNavigator(VEHICLE, VO).plan(snap_with([Obstacle(60.0, 4.0, 0.1), Obstacle(180.0, 1.5, 0.1)]))
        self.assertAlmostEqual(turn(command), 0.0, delta=0.1)

    def test_keeps_its_side_when_both_are_as_good(self):
        navigator = VoNavigator(VEHICLE, VO)
        first = turn(navigator.plan(snap_with([Obstacle(-1.0, 4.0, 0.1)])))        # slightly to port: go starboard
        self.assertGreater(first, 0.0)
        second = turn(navigator.plan(snap_with([Obstacle(1.0, 4.0, 0.1)])))        # now slightly to starboard
        self.assertGreater(second, 0.0)                                            # not worth changing side

    def test_boxed_in_creeps_along_the_widest_miss(self):
        ring = [Obstacle(float(b), 0.8, 0.1) for b in range(-180, 180, 10)]
        command = VoNavigator(VEHICLE, VO).plan(snap_with(ring))
        self.assertEqual(command.reason, "boxed_in")
        self.assertLess(command.speed_mps, 0.4)


def test_bbox_distance_and_pan_tracking():
    from modules.perception import estimate_distance_bbox, track_pan
    # Half the frame height at 70 deg vertical FOV, 1.7 m person.
    assert abs(estimate_distance_bbox(0.5, 70.0, 1.7) - 2.43) < 0.02
    assert estimate_distance_bbox(0.9, 70.0, 1.7) < estimate_distance_bbox(0.3, 70.0, 1.7)
    assert track_pan(10.0, 1.0, 0.5, 2.0, 80.0) == 10.0          # inside the deadband
    assert track_pan(10.0, 20.0, 0.5, 2.0, 80.0) == 20.0         # follows the person to the right
    assert track_pan(75.0, 40.0, 0.5, 2.0, 80.0) == 80.0         # limited

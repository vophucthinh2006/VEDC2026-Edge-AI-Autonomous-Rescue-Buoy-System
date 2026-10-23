import unittest
from time import monotonic

from modules.navigation import BapfNavigator, PaperBapfNavigator, VoNavigator, cpa
from modules.perception import Obstacle
from modules.state_store import GpsState, HumanTarget, ImuState, Snapshot, SysState


class NavigationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.vehicle = {"boat_radius_m": 0.35, "safety_margin_m": 0.4, "influence_distance_m": 3.0, "target_standoff_m": 1.5, "approach_speed_mps": 0.2, "max_speed_mps": 0.5, "arrive_radius_m": 1.0, "stop_distance_m": 0.8, "slow_distance_m": 2.0, "turn_room_m": 0.4}
        self.planner = {"attractive_gain": 1.0, "repulsive_gain": 0.75, "bias_gain": 0.22}
        # A NAV round trip and the coast measured in the simulator (0.65 m/s to rest in 2.1 m).
        self.braking = {"reaction_s": 0.5, "coast_s": 3.0}

    def _snap(self, obstacles, speed=0.0, course=0.0):
        now = monotonic()
        return Snapshot(ImuState(yaw_deg=0, imu_ok=True, timestamp=now), GpsState(lat_deg=10, lon_deg=106, fix=3, speed_mps=speed, course_deg=course, timestamp=now),
                        SysState(timestamp=now), tuple(obstacles), now, (10.001, 106), None)

    def test_close_obstacle_ahead_is_left_at_a_creep_not_waited_out(self) -> None:
        # 1.4 m dead ahead (0.55 m clear of the hull, inside the stop distance), the waypoint behind
        # it. The field points away from it and the bow has room to swing: the boat follows that
        # course slowly instead of waiting there for good.
        command = BapfNavigator(self.vehicle, self.planner).plan(self._snap([Obstacle(0, 1.4, 0.2)]))
        self.assertEqual(command.mode, "AUTO")
        self.assertEqual(command.reason, "leaving_obstacle")
        self.assertGreater(abs((command.heading_deg + 180.0) % 360.0 - 180.0), 90.0)     # away from the obstacle
        self.assertAlmostEqual(command.speed_mps, 0.3 * 0.5)

    def test_no_swinging_away_with_the_bow_against_an_obstacle(self) -> None:
        # 0.8 m dead ahead: no room for the bow. Any speed to turn away drives the boat on along its
        # bow first, onto the obstacle (the wall of the central scene, 2026-10-21). With the stern clear
        # it backs off with the front motor (NAV BACK); it used to stay stopped there for good.
        command = BapfNavigator(self.vehicle, self.planner).plan(self._snap([Obstacle(0, 0.8, 0.2)]))
        self.assertEqual((command.mode, command.speed_mps, command.reason), ("BACK", 0.3, "backing_off"))
        self.assertAlmostEqual(command.heading_deg, 0.0)
        # Something astern too: it stays stopped.
        stern = Obstacle(180, 0.8, 0.2)
        navigator = BapfNavigator(self.vehicle, self.planner)
        command = navigator.plan(self._snap([Obstacle(0, 0.8, 0.2), stern]))
        self.assertEqual((command.mode, command.speed_mps), ("STOP", 0.0))
        # As close abeam (clear of the inflated hull) it is no reason to stay: the bow points past it.
        command = BapfNavigator(self.vehicle, self.planner).plan(self._snap([Obstacle(90, 1.0, 0.2)]))
        self.assertEqual(command.mode, "AUTO")
        # Stopped, the boat waits until the bow is clear by the release margin, not by a hair.
        hull = 0.75 + 0.1
        stern = Obstacle(180, 1.6, 0.2)          # astern, within the stop distance, too far to push much
        navigator = BapfNavigator(self.vehicle, self.planner)
        self.assertEqual(navigator.plan(self._snap([Obstacle(0, 0.8, 0.2), stern])).mode, "STOP")
        self.assertEqual(navigator.plan(self._snap([Obstacle(0, hull + 0.4 + 0.1, 0.2), stern])).mode, "STOP")
        self.assertEqual(navigator.plan(self._snap([Obstacle(0, hull + 0.4 + 0.3, 0.2), stern])).mode, "AUTO")

    def test_the_bow_swings_away_from_an_obstacle_on_the_other_side(self) -> None:
        # The central scene (2026-10-23): the end of the wrecked boat ahead to starboard (the LiDAR's
        # cluster at 25 deg, 1.55 m, 1.2 m wide), the waypoint to port. Stopped, the boat stood there for
        # good: the F407 steers only with some throttle. It turns away at a creep instead.
        wreck = Obstacle(25.0, 1.55, 1.2)
        port = (10.0003, 105.9992)                       # about 70 deg to port of a bow facing north
        now = monotonic()
        snap = Snapshot(ImuState(yaw_deg=0, imu_ok=True, timestamp=now), GpsState(lat_deg=10, lon_deg=106, fix=3, timestamp=now),
                        SysState(timestamp=now), (wreck,), now, port, None)
        command = BapfNavigator(self.vehicle, self.planner).plan(snap)
        self.assertEqual((command.mode, command.reason), ("AUTO", "turning_away"))
        self.assertAlmostEqual(command.speed_mps, 0.3 * 0.5)
        self.assertLess((command.heading_deg + 180.0) % 360.0 - 180.0, -30.0)
        # Coasted on against its side: inside the inflated hull abeam to starboard, still the way out to port.
        side = (Obstacle(24.0, 1.01, 0.3), Obstacle(51.0, 0.74, 0.44), Obstacle(87.0, 0.72, 0.45))
        command = BapfNavigator(self.vehicle, self.planner).plan(Snapshot(snap.imu, snap.gps, snap.system, side, now, port, None))
        self.assertEqual((command.mode, command.reason), ("AUTO", "turning_away"))
        # The waypoint right behind the wreck's end (dead ahead): the field points at it. From rest the boat
        # creeps along the nearest clear course instead, to port, away from the wreck.
        end = (Obstacle(10.0, 1.44, 0.21), Obstacle(24.0, 1.01, 0.3))
        command = BapfNavigator(self.vehicle, self.planner).plan(self._snap(end))
        self.assertEqual((command.mode, command.reason), ("AUTO", "turning_away"))
        self.assertLess((command.heading_deg + 180.0) % 360.0 - 180.0, -5.0)
        # The same on the other side of the bow too: whichever way it swings it would hit one, so it backs off.
        boxed = Snapshot(snap.imu, snap.gps, snap.system, (wreck, Obstacle(-25.0, 1.55, 1.2)), now, port, None)
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(boxed).reason, "backing_off")
        # Right on the bow it is no way out ahead either (the wall of the central scene).
        ahead = Snapshot(snap.imu, snap.gps, snap.system, (Obstacle(2.0, 1.2, 0.2),), now, port, None)
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(ahead).reason, "backing_off")

    def test_a_long_wall_abeam_leaves_the_bow_free(self) -> None:
        # The wrecked boat's side 1.6 m to starboard, 3 m of it in view (2026-10-23): as one cluster it
        # was a circle 3 m wide reaching into the hull, and the boat stood still in open water.
        from math import atan2, degrees, hypot
        from modules.lidar_ld14 import LidarPoint
        from modules.perception import cluster_scan
        wall = tuple(LidarPoint(degrees(atan2(1.6, x / 100)), hypot(1.6, x / 100), 100) for x in range(-150, 151, 3))
        obstacles = cluster_scan(wall)
        self.assertGreater(len(obstacles), 4)
        self.assertTrue(all(o.width_m <= 0.5 + 1e-6 for o in obstacles))
        self.assertTrue(all(o.distance_m - 0.75 - o.width_m / 2 > 0.5 for o in obstacles))
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(self._snap(obstacles)).mode, "AUTO")
        # A short object is still one obstacle.
        post = tuple(LidarPoint(a / 10, 3.0, 100) for a in range(-30, 31, 5))
        self.assertEqual(len(cluster_scan(post)), 1)

    def test_lidar_avoidance_off_ignores_every_obstacle(self) -> None:
        # Boxed in at 0.8 m: STOP with avoidance, straight for the waypoint at full speed without.
        ring = [Obstacle(float(b), 0.8, 0.2) for b in range(-180, 180, 30)]
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(self._snap(ring)).mode, "STOP")
        command = BapfNavigator({**self.vehicle, "lidar_avoidance": False}, self.planner).plan(self._snap(ring))
        self.assertEqual((command.mode, command.reason), ("AUTO", "waypoint"))
        self.assertAlmostEqual(command.speed_mps, 0.5)
        self.assertAlmostEqual(command.heading_deg, 0.0, places=3)

    def test_without_a_fix_and_gps_not_required_the_boat_runs_along_its_bow(self) -> None:
        now = monotonic()
        no_fix = Snapshot(ImuState(yaw_deg=250.0, imu_ok=True, timestamp=now), GpsState(fix=0, timestamp=now),
                          SysState(timestamp=now), (), now, (10.001, 106), None)
        command = BapfNavigator({**self.vehicle, "gps_required": False}, self.planner).plan(no_fix)
        self.assertEqual((command.mode, command.reason), ("AUTO", "bow_no_gps"))
        self.assertAlmostEqual(command.heading_deg, 250.0)
        self.assertAlmostEqual(command.speed_mps, 0.5)
        # The LiDAR still has its say: an obstacle on the bow stops it (and, the stern clear, backs it off).
        blocked = Snapshot(no_fix.imu, no_fix.gps, no_fix.system, (Obstacle(0, 0.8, 0.2),), now, (10.001, 106), None)
        self.assertEqual(BapfNavigator({**self.vehicle, "gps_required": False}, self.planner).plan(blocked).mode, "BACK")
        # By default the old way: a bearing to the waypoint (the safety arbiter then stops it).
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(no_fix).reason, "waypoint")

    def test_the_way_over_the_ground_counts_as_much_as_the_bow(self) -> None:
        # Bow clear, but the boat slides sideways at 0.5 m/s (course 90) onto an obstacle to starboard.
        command = BapfNavigator(self.vehicle, self.planner).plan(self._snap([Obstacle(90, 1.2, 0.2)], speed=0.5, course=90.0))
        self.assertEqual(command.reason, "bow_blocked")
        # Too slow for the GPS course to mean anything: only the bow is looked at.
        command = BapfNavigator(self.vehicle, self.planner).plan(self._snap([Obstacle(90, 1.2, 0.2)], speed=0.1, course=90.0))
        self.assertEqual(command.mode, "AUTO")

    def test_the_stop_distance_grows_with_the_speed_the_gps_measures(self) -> None:
        vehicle = {**self.vehicle, **self.braking}
        navigator = BapfNavigator(vehicle, self.planner)
        # 1.43 m/s (the simulator, told 0.29 m/s): 0.7 m reacting, 4.3 m coasting.
        self.assertAlmostEqual(navigator.braking_m(self._snap([], speed=1.43)), 1.43 * 3.5)
        self.assertEqual(navigator.braking_m(self._snap([])), 0.0)
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).braking_m(self._snap([], speed=1.43)), 0.0)  # not configured
        # 6 m dead ahead: nothing to do at rest, the throttle off already at 1.43 m/s.
        self.assertEqual(navigator.plan(self._snap([Obstacle(0, 6.0, 0.2)])).reason, "waypoint")
        command = BapfNavigator(vehicle, self.planner).plan(self._snap([Obstacle(0, 6.0, 0.2)], speed=1.43))
        self.assertEqual((command.mode, command.reason), ("STOP", "obstacle_too_close"))

    def test_the_throttle_goes_off_while_the_boat_runs_faster_than_told(self) -> None:
        navigator = BapfNavigator({**self.vehicle, "overspeed_mps": 0.3}, self.planner)
        # Told 0.5 m/s (open water): 0.75 m/s is within the margin, 0.85 is not.
        self.assertAlmostEqual(navigator.plan(self._snap([], speed=0.75)).speed_mps, 0.5)
        command = navigator.plan(self._snap([], speed=0.85))
        self.assertEqual((command.mode, command.speed_mps, command.reason), ("AUTO", 0.0, "overspeed"))
        # Coasting, the throttle comes back only below 0.5 + 0.15 m/s, not at the first dip.
        self.assertEqual(navigator.plan(self._snap([], speed=0.7)).reason, "overspeed")
        self.assertEqual(navigator.plan(self._snap([], speed=0.6)).reason, "waypoint")
        # Unset: off, whatever the GPS says.
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(self._snap([], speed=3.0)).reason, "waypoint")
        # Told a creep (0.3 x 0.5 m/s, an obstacle near): the margin shrinks with it, 0.15 + 0.09.
        creeping = BapfNavigator({**self.vehicle, "overspeed_mps": 0.3}, self.planner)
        self.assertEqual(creeping.plan(self._snap([Obstacle(0, 1.4, 0.2)], speed=0.2)).reason, "leaving_obstacle")
        self.assertEqual(creeping.plan(self._snap([Obstacle(0, 1.4, 0.2)], speed=0.26)).reason, "overspeed")

    def test_the_person_is_kept_off_the_bow_and_coasted_up_to(self) -> None:
        vehicle = {**self.vehicle, **self.braking, "person_gap_m": 2.5, "bow_ahead_m": 0.25}
        navigator = BapfNavigator(vehicle, self.planner)
        self.assertAlmostEqual(navigator.standoff_m, 2.75)       # range from the LiDAR, bow 0.25 m ahead of it
        self.assertAlmostEqual(BapfNavigator(self.vehicle, self.planner).standoff_m, 1.5)   # target_standoff_m alone
        # At rest 2.8 m away: arrived.
        self.assertEqual(navigator.plan(self._person(3.0, 2.8)).reason, "human_standoff")
        # 5 m away at rest: closing in; at 1.0 m/s (3.5 m of braking) the throttle is already off.
        self.assertEqual(BapfNavigator(vehicle, self.planner).plan(self._person(3.0, 5.0)).reason, "approach_human")
        self.assertEqual(BapfNavigator(vehicle, self.planner).plan(self._person(3.0, 5.0, speed=1.0)).reason, "human_standoff")

    def test_stops_when_the_course_leads_onto_a_close_obstacle(self) -> None:
        # Boxed in on all sides: the pushes cancel, the course stays on the waypoint and on the obstacle ahead.
        ring = [Obstacle(float(b), 0.8, 0.2) for b in range(-180, 180, 30)]
        command = BapfNavigator(self.vehicle, {**self.planner, "bias_gain": 0.0}).plan(self._snap(ring))
        self.assertEqual(command.mode, "STOP")
        self.assertEqual(command.reason, "obstacle_too_close")

    def _person(self, bearing, distance, speed=0.0):
        now = monotonic()
        return Snapshot(ImuState(yaw_deg=30.0, imu_ok=True, timestamp=now), GpsState(lat_deg=10, lon_deg=106, fix=3, speed_mps=speed, course_deg=30.0, timestamp=now),
                        SysState(timestamp=now), (), now, None, HumanTarget(bearing, distance, 0.8, now))

    def test_the_bow_turns_to_the_person_before_the_boat_goes_to_them(self) -> None:
        navigator = BapfNavigator(self.vehicle, self.planner)
        # 50 deg to starboard, 6 m away: heading for them (30 + 50), at a creep.
        command = navigator.plan(self._person(50.0, 6.0))
        self.assertEqual((command.mode, command.reason), ("AUTO", "face_human"))
        self.assertAlmostEqual(command.heading_deg, 80.0)
        self.assertLess(command.speed_mps, 0.15 + 1e-9)
        self.assertGreater(command.speed_mps, 0.0)
        # Bow on them: the approach speed.
        command = navigator.plan(self._person(4.0, 6.0))
        self.assertEqual(command.reason, "approach_human")
        self.assertAlmostEqual(command.speed_mps, 0.2 * (1.0 - 4.0 / 120.0))

    def test_the_bow_stays_on_the_person_at_the_standoff_but_never_closer(self) -> None:
        navigator = BapfNavigator(self.vehicle, self.planner)
        # At the stand-off (1.5 m), bow on them: stopped.
        command = navigator.plan(self._person(3.0, 1.5))
        self.assertEqual((command.reason, command.speed_mps), ("human_standoff", 0.0))
        # At the stand-off with the bow 40 deg off: a creep, so the STM32 keeps steering to them.
        command = navigator.plan(self._person(-40.0, 1.5))
        self.assertEqual(command.reason, "face_human")
        self.assertGreater(command.speed_mps, 0.0)
        self.assertAlmostEqual(command.heading_deg, 350.0)
        # Well inside the stand-off: no creeping towards them, whatever the bow does.
        command = navigator.plan(self._person(-40.0, 1.0))
        self.assertEqual((command.reason, command.speed_mps), ("human_standoff", 0.0))

    def test_a_stop_for_an_obstacle_holds_until_it_is_clearly_behind_the_stop_distance(self) -> None:
        # Boxed in: STOP. Hull 0.75 m + half the width 0.1 m: a ring at d leaves d - 0.85 m clear.
        navigator = BapfNavigator(self.vehicle, {**self.planner, "bias_gain": 0.0})
        ring = lambda d: [Obstacle(float(b), d, 0.2) for b in range(-180, 180, 30)]   # noqa: E731
        self.assertEqual(navigator.plan(self._snap(ring(0.8))).reason, "obstacle_too_close")
        # 0.1 m past the stop distance (0.8 m): a fresh plan would go on; this one stays stopped.
        just_past = 0.8 + 0.85 + 0.1
        self.assertEqual(BapfNavigator(self.vehicle, {**self.planner, "bias_gain": 0.0}).plan(self._snap(ring(just_past))).mode, "AUTO")
        self.assertEqual(navigator.plan(self._snap(ring(just_past))).reason, "obstacle_too_close")
        # Clear by more than the release margin: on again.
        self.assertEqual(navigator.plan(self._snap(ring(0.8 + 0.85 + 0.3))).mode, "AUTO")

    def test_the_standoff_holds_while_the_range_jitters(self) -> None:
        navigator = BapfNavigator(self.vehicle, self.planner)
        # Stand-off 1.5 m: arrived at 1.6 m.
        self.assertEqual(navigator.plan(self._person(3.0, 1.6)).reason, "human_standoff")
        # A box range reading 0.4 m further does not start the approach again...
        self.assertEqual(navigator.plan(self._person(3.0, 1.9)).reason, "human_standoff")
        self.assertEqual(BapfNavigator(self.vehicle, self.planner).plan(self._person(3.0, 1.9)).reason, "approach_human")
        # ...a person really moving off does.
        self.assertEqual(navigator.plan(self._person(3.0, 2.1)).reason, "approach_human")
        self.assertEqual(navigator.plan(self._person(3.0, 1.9)).reason, "approach_human")

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

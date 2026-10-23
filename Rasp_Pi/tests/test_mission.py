import unittest

from modules.mission import FOLLOW, HOME_HOLD, IDLE, PATROL, RETURN, Mission
from modules.state_store import CRITICAL, HIGH, MEDIUM, SAFE, GpsState, HumanTarget, ImuState, LinkState, Snapshot, SysState
from modules.waypoints import Route
from utils.geometry import haversine_m, offset_latlon

BOAT = (10.762600, 106.660100)
WP1 = offset_latlon(*BOAT, 0.0, 20.0)       # 20 m north
WP2 = offset_latlon(*BOAT, 90.0, 20.0)      # 20 m east
# A square pool 100 m a side round the boat.
FENCE = [offset_latlon(*offset_latlon(*BOAT, north, 50.0), east, 50.0) for north, east in ((0, 270), (0, 90), (180, 90), (180, 270))]
HOME = offset_latlon(*BOAT, 180.0, 30.0)    # the shore station, inside the pool's fence
CONFIG = {"follow": {"confirm_frames": 3, "confirm_s": 2.5, "lost_s": 4.0, "report_every_s": 20.0, "ignore_s": 60.0},
          "return": {"arrive_radius_m": 3.0, "fence_margin_m": 2.0}}


def snap(t, at=BOAT, people=(), shore_ok=True, home=None, presses=-1, fix=3, yaw=0.0, battery=-1):
    """people: (bearing from the bow, distance, risk) seen in a detector pass at t."""
    targets = tuple(HumanTarget(bearing, distance, 0.6, t, risk) for bearing, distance, risk in people)
    return Snapshot(ImuState(yaw_deg=yaw, imu_ok=True, timestamp=t), GpsState(*at, fix=fix, timestamp=t),
                    SysState(timestamp=t, confirm_presses=presses, battery_pct=battery), (), t, None, targets[0] if targets else None,
                    human_targets=targets, link=LinkState(shore_ok, 1.0 if shore_ok else 75.0, home, t))


class MissionTest(unittest.TestCase):
    def setUp(self):
        self.mission = Mission(CONFIG, Route([], 3.0, False, FENCE), only_on_route=True)

    def give_route(self, *points):
        self.mission.set_route(Route(list(points), 3.0, False, FENCE))

    def run_steps(self, times, **kwargs):
        steps = [self.mission.step(snap(t, **kwargs), t) for t in times]
        return steps[-1], [r for s in steps for r in s.reports]

    def test_idle_without_a_route_then_patrol_once_one_comes(self):
        step = self.mission.step(snap(0.0, people=[(10.0, 5.0, HIGH)]), 0.0)
        self.assertEqual(self.mission.state, IDLE)
        self.assertIsNone(step.planned.waypoint)
        self.assertIsNone(step.planned.human_target)
        self.assertFalse(self.mission.sweeping)
        self.assertFalse(self.mission.detecting)        # no route: the detector rests
        self.give_route(WP1, WP2)
        step = self.mission.step(snap(1.0), 1.0)
        self.assertEqual(self.mission.state, PATROL)
        self.assertEqual(step.planned.waypoint, WP1)
        self.assertTrue(self.mission.sweeping)
        self.assertTrue(self.mission.detecting)

    def test_the_detector_runs_on_the_way_home_too(self):
        self.give_route(WP1, WP2)
        self.mission.step(snap(0.0, home=HOME), 0.0)
        self.mission.step(snap(1.0, shore_ok=False, home=HOME), 1.0)
        self.assertNotEqual(self.mission.state, IDLE)
        self.assertTrue(self.mission.detecting)

    def test_route_done_is_idle(self):
        self.give_route(WP1)
        self.mission.step(snap(0.0), 0.0)
        step = self.mission.step(snap(1.0, at=WP1), 1.0)
        self.assertEqual(self.mission.state, IDLE)
        self.assertIsNone(step.planned.waypoint)

    def test_patrol_follow_and_back_to_patrol_when_the_supplies_are_taken(self):
        self.give_route(WP1, WP2)
        self.mission.step(snap(0.0, presses=0), 0.0)
        step, _ = self.run_steps((0.2, 1.0, 1.8), people=[(20.0, 5.0, HIGH)], presses=0)
        self.assertEqual(self.mission.state, FOLLOW)
        self.assertAlmostEqual(step.planned.human_target.bearing_body_deg, 20.0)
        step = self.mission.step(snap(2.6, people=[(20.0, 2.0, HIGH)], presses=1), 2.6)
        self.assertTrue(step.supplies_taken)
        self.assertEqual(self.mission.state, PATROL)
        self.assertEqual(step.planned.waypoint, WP1)
        self.assertIsNone(step.planned.human_target)
        # The same person, still in view: left alone.
        self.run_steps((3.4, 4.2, 5.0, 5.8), people=[(20.0, 2.0, HIGH)], presses=1)
        self.assertEqual(self.mission.state, PATROL)
        # Somebody else, elsewhere: followed.
        self.run_steps((6.6, 7.4, 8.2), people=[(-60.0, 6.0, MEDIUM)], presses=1)
        self.assertEqual(self.mission.state, FOLLOW)

    def test_three_minutes_with_a_person_and_no_press_is_patrol_again(self):
        mission = Mission({**CONFIG, "follow": {**CONFIG["follow"], "max_hold_s": 180.0, "hold_ignore_s": 600.0}},
                          Route([WP1, WP2], 3.0, False, FENCE), only_on_route=True)
        person = [(20.0, 3.0, HIGH)]
        for t in (0.0, 0.8, 1.6):
            mission.step(snap(t, people=person), t)
        self.assertEqual(mission.state, FOLLOW)
        # Lost for 10 s (waves) at 100 s and found again: the same stay, the clock runs on.
        for t in range(2, 100):
            mission.step(snap(float(t), people=person), float(t))
        for t in range(100, 110):
            mission.step(snap(float(t)), float(t))
        self.assertEqual(mission.state, PATROL)
        for t in (110.0, 110.8, 111.6):
            mission.step(snap(t, people=person), t)
        self.assertEqual(mission.state, FOLLOW)
        # Followed from 1.6 s on: 180 s with them at 181.6 s.
        for t in range(112, 182):
            mission.step(snap(float(t), people=person), float(t))
        self.assertEqual(mission.state, FOLLOW)
        with self.assertLogs("modules.mission", "INFO") as logs:
            step = mission.step(snap(182.0, people=person), 182.0)
        self.assertEqual(mission.state, PATROL)
        self.assertEqual(step.planned.waypoint, WP1)
        self.assertIsNone(step.planned.human_target)
        self.assertTrue(any("180 s with that person and no confirm" in line for line in logs.output), logs.output)
        # Still in view: left alone for hold_ignore_s, the route goes on...
        for t in range(183, 600, 5):
            mission.step(snap(float(t), people=person), float(t))
        self.assertEqual(mission.state, PATROL)
        # ...and somebody else is followed meanwhile.
        for t in (600.0, 600.8, 601.6):
            mission.step(snap(t, people=[(-60.0, 6.0, MEDIUM)]), t)
        self.assertEqual(mission.state, FOLLOW)

    def test_no_hold_limit_by_default(self):
        self.give_route(WP1)
        for t in range(0, 400, 1):
            self.mission.step(snap(float(t), people=[(0.0, 3.0, HIGH)]), float(t))
        self.assertEqual(self.mission.state, FOLLOW)

    def test_a_person_lost_for_lost_s_is_patrol_again(self):
        self.give_route(WP1)
        self.run_steps((0.0, 0.8, 1.6), people=[(0.0, 6.0, MEDIUM)])
        self.assertEqual(self.mission.state, FOLLOW)
        self.run_steps((3.0, 5.0, 6.0))
        self.assertEqual(self.mission.state, PATROL)

    def test_the_highest_risk_is_followed_and_never_somebody_on_a_boat(self):
        self.give_route(WP1)
        step, _ = self.run_steps((0.0, 0.8, 1.6), people=[(-30.0, 3.0, SAFE), (10.0, 7.0, MEDIUM), (40.0, 8.0, CRITICAL)])
        self.assertEqual(self.mission.state, FOLLOW)
        self.assertAlmostEqual(step.planned.human_target.bearing_body_deg, 40.0)
        self.mission = Mission(CONFIG, Route([WP1], 3.0, False, FENCE), only_on_route=True)
        self.run_steps((0.0, 0.8, 1.6, 2.4), people=[(-30.0, 3.0, SAFE)])
        self.assertEqual(self.mission.state, PATROL)

    def test_lost_link_return_home_hold_and_back_to_the_same_waypoint(self):
        self.give_route(WP1, WP2)
        self.mission.step(snap(0.0), 0.0)
        self.mission.step(snap(1.0, at=WP1), 1.0)                 # WP1 reached: going to WP2
        step = self.mission.step(snap(2.0, at=WP1, shore_ok=False, home=HOME, people=[(0.0, 4.0, CRITICAL)]), 2.0)
        self.assertEqual(self.mission.state, RETURN)
        self.assertEqual(step.planned.waypoint, HOME)
        self.assertIsNone(step.planned.human_target)              # nobody followed on the way home
        self.run_steps((2.8, 3.6, 4.4), at=WP1, shore_ok=False, home=HOME, people=[(0.0, 4.0, CRITICAL)])
        self.assertEqual(self.mission.state, RETURN)
        step = self.mission.step(snap(30.0, at=offset_latlon(*HOME, 0.0, 2.0), shore_ok=False), 30.0)
        self.assertEqual(self.mission.state, HOME_HOLD)
        self.assertIsNone(step.planned.waypoint)
        step = self.mission.step(snap(40.0, at=HOME, shore_ok=False), 40.0)
        self.assertEqual(self.mission.state, HOME_HOLD)
        step = self.mission.step(snap(50.0, at=HOME), 50.0)
        self.assertEqual(self.mission.state, PATROL)
        self.assertEqual(step.planned.waypoint, WP2)

    def test_lost_link_while_following_returns_too(self):
        self.give_route(WP1)
        self.run_steps((0.0, 0.8, 1.6), people=[(0.0, 6.0, HIGH)], home=HOME)
        self.assertEqual(self.mission.state, FOLLOW)
        self.mission.step(snap(2.0, people=[(0.0, 6.0, HIGH)], shore_ok=False), 2.0)
        self.assertEqual(self.mission.state, RETURN)
        self.assertEqual(self.mission.home_goal(), HOME)          # the last home heard is kept

    def test_idle_does_not_return(self):
        self.mission.step(snap(0.0, shore_ok=False, home=HOME), 0.0)
        self.assertEqual(self.mission.state, IDLE)

    def test_without_a_home_the_boat_goes_back_to_where_the_route_was_accepted(self):
        self.give_route(WP1, WP2)
        start = offset_latlon(*BOAT, 270.0, 5.0)
        self.mission.step(snap(0.0, at=start, fix=0), 0.0)        # no fix yet: the first fix counts
        self.mission.step(snap(1.0, at=start), 1.0)
        step = self.mission.step(snap(5.0, at=BOAT, shore_ok=False), 5.0)
        self.assertEqual(self.mission.state, RETURN)
        self.assertEqual(step.planned.waypoint, start)

    def test_a_home_outside_the_fence_is_brought_inside_it(self):
        self.give_route(WP1)
        outside = offset_latlon(*BOAT, 180.0, 70.0)               # 20 m beyond the south edge
        self.mission.step(snap(0.0), 0.0)
        step = self.mission.step(snap(1.0, shore_ok=False, home=outside), 1.0)
        goal = step.planned.waypoint
        route = self.mission.route
        self.assertTrue(route.inside_fence(*goal))
        self.assertAlmostEqual(haversine_m(*BOAT, *goal), 48.0, delta=0.2)   # 2 m inside the south edge
        self.assertAlmostEqual(goal[1], BOAT[1], places=6)


class BatteryHomeTest(unittest.TestCase):
    """Below battery_low_pct the boat goes home whatever it does, until a fresh pack reads battery_ok_pct."""

    def setUp(self):
        self.mission = Mission(CONFIG, Route([], 3.0, False, FENCE), only_on_route=True)
        self.mission.set_route(Route([WP1, WP2], 3.0, False, FENCE))

    def test_a_low_pack_takes_the_boat_home_and_keeps_it_there(self):
        step = self.mission.step(snap(0.0, home=HOME, battery=60), 0.0)
        self.assertEqual((self.mission.state, self.mission.code, self.mission.waypoint_field), (PATROL, "P", "1/2"))
        step = self.mission.step(snap(1.0, home=HOME, battery=19), 1.0)
        self.assertEqual((self.mission.state, self.mission.code), (RETURN, "B"))
        self.assertEqual(step.planned.waypoint, HOME)
        self.assertIsNone(step.planned.human_target)
        # 25 %: still low (no going back out on a pack that only recovered at rest)
        self.mission.step(snap(2.0, home=HOME, battery=25), 2.0)
        self.assertEqual(self.mission.state, RETURN)
        self.mission.step(snap(3.0, at=HOME, home=HOME, battery=25), 3.0)
        self.assertEqual((self.mission.state, self.mission.code), (HOME_HOLD, "H"))
        # A fresh pack: back to the route, and the log names the pack, not the shore link
        with self.assertLogs("modules.mission", "INFO") as logs:
            self.mission.step(snap(4.0, at=HOME, home=HOME, battery=95), 4.0)
        self.assertEqual(self.mission.state, PATROL)
        self.assertTrue(any("battery back" in line for line in logs.output), logs.output)

    def test_a_sag_under_the_motors_is_not_a_low_pack(self):
        mission = Mission({**CONFIG, "return": {**CONFIG["return"], "battery_hold_s": 30.0}},
                          Route([WP1, WP2], 3.0, False, FENCE), only_on_route=True)
        mission.step(snap(0.0, home=HOME, battery=60), 0.0)
        # The stand, 2026-10-22: 15 % for 1.5 s, then 33 %. The boat runs on.
        for t, pct in ((1.0, 15), (1.5, 15), (2.5, 33), (3.0, 15), (4.0, 40)):
            mission.step(snap(t, home=HOME, battery=pct), t)
            self.assertEqual(mission.state, PATROL, t)
        # Low for 30 s on end: home.
        for t in range(10, 40):
            mission.step(snap(float(t), home=HOME, battery=17), float(t))
        self.assertEqual(mission.state, PATROL)
        mission.step(snap(40.0, home=HOME, battery=17), 40.0)
        self.assertEqual((mission.state, mission.code), (RETURN, "B"))
        # At rest the pack reads 33 % again: not enough to set off, unless it holds for 30 s.
        for t in range(41, 70):
            mission.step(snap(float(t), home=HOME, battery=33 if t % 10 else 18), float(t))
        self.assertEqual(mission.state, RETURN)
        for t in range(70, 101):
            mission.step(snap(float(t), home=HOME, battery=95), float(t))
        self.assertEqual(mission.state, PATROL)

    def test_standing_idle_with_a_low_pack_goes_home_too(self):
        mission = Mission(CONFIG, Route([], 3.0, False, FENCE), only_on_route=True)
        mission.step(snap(0.0, home=HOME, battery=60), 0.0)
        self.assertEqual((mission.state, mission.code, mission.waypoint_field), (IDLE, "I", "0/0"))
        step = mission.step(snap(1.0, home=HOME, battery=15), 1.0)
        self.assertEqual(mission.state, RETURN)
        self.assertEqual(step.planned.waypoint, HOME)

    def test_the_shore_link_coming_back_does_not_end_a_battery_return(self):
        self.mission.step(snap(0.0, home=HOME, battery=10, shore_ok=False), 0.0)
        self.assertEqual((self.mission.state, self.mission.code), (RETURN, "B"))
        self.mission.step(snap(1.0, home=HOME, battery=10, shore_ok=True), 1.0)
        self.assertEqual(self.mission.state, RETURN)

    def test_shore_lost_alone_is_code_r(self):
        self.mission.step(snap(0.0, home=HOME, battery=80), 0.0)
        self.mission.step(snap(1.0, home=HOME, battery=80, shore_ok=False), 1.0)
        self.assertEqual((self.mission.state, self.mission.code), (RETURN, "R"))

    def test_no_battery_reading_never_sends_it_home(self):
        for t in range(5):
            self.mission.step(snap(float(t), home=HOME, battery=-1), float(t))
        self.assertEqual(self.mission.state, PATROL)


class FenceClampTest(unittest.TestCase):
    def setUp(self):
        self.route = Route([], 3.0, False, FENCE)

    def test_inside_is_left_alone(self):
        self.assertEqual(self.route.clamp_inside_fence(*HOME, 2.0), HOME)

    def test_beyond_an_edge_comes_square_inside_it(self):
        lat, lon = self.route.clamp_inside_fence(*offset_latlon(*BOAT, 90.0, 80.0), 2.0)
        self.assertTrue(self.route.inside_fence(lat, lon))
        self.assertAlmostEqual(haversine_m(*BOAT, lat, lon), 48.0, delta=0.2)

    def test_beyond_a_corner_comes_inside_too(self):
        corner_out = offset_latlon(*offset_latlon(*BOAT, 0.0, 70.0), 90.0, 70.0)
        lat, lon = self.route.clamp_inside_fence(*corner_out, 2.0)
        self.assertTrue(self.route.inside_fence(lat, lon))
        self.assertLess(haversine_m(lat, lon, *FENCE[1]), 4.0)

    def test_no_fence_no_clamp(self):
        far = offset_latlon(*BOAT, 0.0, 500.0)
        self.assertEqual(Route([], 3.0).clamp_inside_fence(*far, 2.0), far)


class ReportTest(unittest.TestCase):
    def setUp(self):
        self.mission = Mission(CONFIG, Route([WP1, WP2], 3.0, False, FENCE), only_on_route=True)

    def reports(self, times, **kwargs):
        return [r for t in times for r in self.mission.step(snap(t, **kwargs), t).reports]

    def test_a_person_is_reported_once_with_position_and_risk(self):
        reports = self.reports((0.0, 0.8, 1.6, 2.4, 3.2), people=[(30.0, 6.0, HIGH)])
        self.assertEqual([(r.why, r.risk) for r in reports], [("found", HIGH)])
        self.assertLess(haversine_m(reports[0].lat, reports[0].lon, *offset_latlon(*BOAT, 30.0, 6.0)), 0.01)

    def test_somebody_on_a_boat_is_reported_once_but_not_followed(self):
        reports = self.reports((0.0, 0.8, 1.6, 2.4), people=[(-20.0, 7.0, SAFE)])
        self.assertEqual([(r.why, r.risk) for r in reports], [("found", SAFE)])
        self.assertEqual(self.mission.state, PATROL)

    def test_the_same_person_seen_again_later_is_not_reported_twice(self):
        self.reports((0.0, 0.8, 1.6), people=[(30.0, 6.0, MEDIUM)])
        self.reports((3.0, 10.0))                                 # out of sight
        again = self.reports((11.0, 11.8, 12.6), people=[(32.0, 6.5, MEDIUM)])
        self.assertEqual(again, [])

    def test_two_people_are_two_reports(self):
        reports = self.reports((0.0, 0.8, 1.6), people=[(30.0, 6.0, MEDIUM), (-40.0, 5.0, HIGH)])
        self.assertEqual(sorted(r.risk for r in reports), [MEDIUM, HIGH])

    def test_without_a_fix_the_position_is_zero(self):
        reports = self.reports((0.0, 0.8, 1.6), people=[(30.0, 6.0, MEDIUM)], fix=0)
        self.assertEqual([(r.lat, r.lon) for r in reports], [(0.0, 0.0)])

    def test_a_change_of_risk_while_following_is_reported_once_seen_twice(self):
        self.reports((0.0, 0.8, 1.6), people=[(0.0, 6.0, HIGH)])
        self.assertEqual(self.mission.state, FOLLOW)
        self.assertEqual(self.reports((2.4,), people=[(0.0, 6.0, CRITICAL)]), [])
        reports = self.reports((3.2,), people=[(0.0, 6.0, CRITICAL)])
        self.assertEqual([(r.why, r.risk) for r in reports], [("risk", CRITICAL)])
        self.assertEqual(self.reports((4.0, 4.8), people=[(0.0, 6.0, CRITICAL)]), [])

    def test_a_one_frame_flicker_of_risk_is_not_reported(self):
        self.reports((0.0, 0.8, 1.6), people=[(0.0, 6.0, HIGH)])
        self.reports((2.4,), people=[(0.0, 6.0, MEDIUM)])
        self.assertEqual(self.reports((3.2, 4.0), people=[(0.0, 6.0, HIGH)]), [])

    def test_a_lower_risk_waits_for_report_every_s(self):
        self.reports((0.0, 0.8, 1.6), people=[(0.0, 6.0, HIGH)])
        # Flickering HIGH / MEDIUM, two frames each, for 18 s: nothing before report_every_s.
        times = [1.6 + 0.8 * i for i in range(1, 22)]             # to 18.4 s
        risks = [MEDIUM if (i // 2) % 2 == 0 else HIGH for i in range(len(times))]
        reports = [r for t, risk in zip(times, risks) for r in self.mission.step(snap(t, people=[(0.0, 6.0, risk)]), t).reports]
        self.assertEqual(reports, [])
        # Held past report_every_s, the lower risk goes out.
        later = [19.2 + 0.8 * i for i in range(6)]
        reports = [r for t in later for r in self.mission.step(snap(t, people=[(0.0, 6.0, MEDIUM)]), t).reports]
        self.assertEqual([(r.why, r.risk) for r in reports], [("risk", MEDIUM)])

    def test_the_followed_position_every_report_every_s_when_it_moved(self):
        self.reports((0.0, 0.8, 1.6), people=[(0.0, 6.0, HIGH)])
        times = [1.6 + 0.8 * i for i in range(1, 30)]             # to 24.8 s
        # Drifting 0.2 m a pass away from the boat: about 5 m in 20 s.
        reports = [r for i, t in enumerate(times) for r in self.mission.step(snap(t, people=[(0.0, 6.0 + 0.2 * (i + 1), HIGH)]), t).reports]
        self.assertEqual([r.why for r in reports], ["moved"])
        self.assertEqual(self.mission.state, FOLLOW)

    def test_no_periodic_report_when_the_person_stays_put(self):
        self.reports((0.0, 0.8, 1.6), people=[(0.0, 6.0, HIGH)])
        times = [1.6 + 0.8 * i for i in range(1, 30)]
        self.assertEqual(self.reports(times, people=[(0.0, 6.0, HIGH)]), [])

    def test_people_are_still_reported_on_the_way_home(self):
        self.mission.step(snap(0.0, home=HOME), 0.0)
        reports = self.reports((1.0, 1.8, 2.6), shore_ok=False, home=HOME, people=[(45.0, 5.0, CRITICAL)])
        self.assertEqual(self.mission.state, RETURN)
        self.assertEqual([(r.why, r.risk) for r in reports], [("found", CRITICAL)])


if __name__ == "__main__":
    unittest.main()

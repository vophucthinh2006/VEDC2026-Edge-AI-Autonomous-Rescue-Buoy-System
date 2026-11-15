import unittest

from modules.formation import FormationKeeper
from modules.mission import FORMATION, PATROL, RETURN, Mission
from modules.state_store import GpsState, ImuState, LeaderState, LinkState, Snapshot, SysState
from modules.waypoints import Route
from utils.geometry import bearing_deg, haversine_m, offset_latlon

LEADER = (10.762600, 106.660100)


def snap(t, at, leader=None, follow=("PHAO-01", 10.0, 0.0), ret=False):
    return Snapshot(ImuState(yaw_deg=90.0, imu_ok=True, timestamp=t), GpsState(*at, fix=3, timestamp=t),
                    SysState(timestamp=t), (), t, None, None, link=LinkState(True, 1.0, None, t, ret=ret, follow=follow), leader=leader)


class KeeperTest(unittest.TestCase):
    def setUp(self):
        self.keeper = FormationKeeper({"lookahead_m": 6.0, "speed_gain_per_m": 0.1, "leader_stale_s": 15.0}, 0.42)

    def test_place_right_of_a_leader_going_east_is_south_and_reckoned_forward(self):
        leader = LeaderState("PHAO-01", *LEADER, 90.0, 0.3, 100.0)
        place = offset_latlon(*offset_latlon(*LEADER, 90.0, 0.3 * 4.0), 180.0, 10.0)
        goal = self.keeper.goal(snap(104.0, place), leader, 10.0, 0.0, 104.0)
        self.assertLess(haversine_m(*goal.place, *place), 0.05)
        self.assertLess(goal.error_m, 0.05)
        self.assertAlmostEqual(goal.speed_mps, 0.3, places=2)          # at its place: the leader's speed
        self.assertAlmostEqual(bearing_deg(*goal.place, *goal.point), 90.0, delta=0.5)   # steering along, ahead

    def test_behind_its_place_it_goes_faster_up_to_its_top_speed_ahead_slower(self):
        leader = LeaderState("PHAO-01", *LEADER, 90.0, 0.3, 100.0)
        place = offset_latlon(*LEADER, 180.0, 10.0)
        behind = self.keeper.goal(snap(100.0, offset_latlon(*place, 270.0, 1.0)), leader, 10.0, 0.0, 100.0)
        far_behind = self.keeper.goal(snap(100.0, offset_latlon(*place, 270.0, 20.0)), leader, 10.0, 0.0, 100.0)
        ahead = self.keeper.goal(snap(100.0, offset_latlon(*place, 90.0, 2.0)), leader, 10.0, 0.0, 100.0)
        self.assertAlmostEqual(behind.speed_mps, 0.4, places=2)
        self.assertEqual(far_behind.speed_mps, 0.42)
        self.assertAlmostEqual(ahead.speed_mps, 0.1, places=2)

    def hold(self, heading, seconds, t0, at=LEADER):
        """The leader at `at` holding `heading` for `seconds` beacons, one a second: the last goal."""
        here = offset_latlon(*LEADER, 180.0, 10.0)
        for t in range(t0, t0 + seconds):
            goal = self.keeper.goal(snap(t, here), LeaderState("PHAO-01", *at, heading, 0.3, t), 10.0, 0.0, t)
        return goal

    def test_the_side_stays_with_the_water_when_the_leader_turns_about(self):
        # A lawnmower: lane 1 east (right of the leader: south), the short leg south across, lane 2 west. The
        # follower keeps to the south of the leader all the way (its left on lane 2), never crossing its way.
        goal = self.hold(90.0, 30, 100)
        self.assertAlmostEqual(bearing_deg(*LEADER, *goal.place), 180.0, delta=1.0)
        south = offset_latlon(*LEADER, 180.0, 20.0)
        goal = self.hold(180.0, 12, 130, south)     # the leg across: shorter than the lane, square to it
        self.assertAlmostEqual(bearing_deg(*south, *goal.place), 180.0, delta=1.0)   # still square to the lane, south
        goal = self.hold(270.0, 10, 142, south)     # lane 2, west
        self.assertAlmostEqual(bearing_deg(*south, *goal.place), 180.0, delta=1.0)
        self.assertAlmostEqual(haversine_m(*south, *goal.place), 10.0, delta=0.1)

    def test_a_leg_square_to_the_lane_held_longer_than_it_is_the_lane_now(self):
        # From the start to the first lane the leader goes north 8 s, then east along the lane: once east is held
        # longer than north was, the place is right of east (south), not right of north (east).
        self.hold(0.0, 8, 100)
        goal = self.hold(90.0, 6, 108)
        self.assertAlmostEqual(bearing_deg(*LEADER, *goal.place), 90.0, delta=1.0)    # not yet: right of north
        goal = self.hold(90.0, 6, 114)
        self.assertAlmostEqual(bearing_deg(*LEADER, *goal.place), 180.0, delta=1.0)   # right of east

    def test_ned_offsets_are_north_and_east_of_the_leader_whatever_its_heading(self):
        keeper = FormationKeeper({"offsets": "ned"}, 0.42)
        for heading in (90.0, 180.0, 270.0):
            leader = LeaderState("PHAO-01", *LEADER, heading, 0.0, 100.0)
            goal = keeper.goal(snap(100.0, LEADER), leader, -6.0, 2.0, 100.0)   # 6 m south, 2 m east
            self.assertLess(haversine_m(*goal.place, *offset_latlon(*offset_latlon(*LEADER, 180.0, 6.0), 90.0, 2.0)), 0.05)

    def test_a_still_leader_and_the_boat_at_its_place_stop_it_a_stale_one_gives_nothing(self):
        leader = LeaderState("PHAO-01", *LEADER, 90.0, 0.0, 100.0)
        place = offset_latlon(*LEADER, 180.0, 10.0)
        self.assertEqual(self.keeper.goal(snap(101.0, place), leader, 10.0, 0.0, 101.0).speed_mps, 0.0)
        self.assertIsNone(self.keeper.goal(snap(116.0, place), leader, 10.0, 0.0, 116.0))
        self.assertIsNone(self.keeper.goal(snap(101.0, place), None, 10.0, 0.0, 101.0))


class FormationMissionTest(unittest.TestCase):
    def setUp(self):
        self.mission = Mission({"vehicle": {"max_speed_mps": 0.3}}, Route([], 3.0, False, None), only_on_route=True)

    def test_a_follower_goes_to_its_place_at_the_speed_set_then_home_when_called(self):
        leader = LeaderState("PHAO-01", *LEADER, 90.0, 0.3, 0.0)
        here = offset_latlon(*LEADER, 180.0, 25.0)
        step = self.mission.step(snap(1.0, here, leader), 1.0)
        self.assertEqual(self.mission.state, FORMATION)
        self.assertEqual(self.mission.code, "O")
        self.assertIsNotNone(step.planned.waypoint)
        self.assertGreater(step.planned.goal_speed, 0.3)
        self.assertTrue(self.mission.detecting)
        self.mission.step(snap(2.0, here, leader, ret=True), 2.0)
        self.assertEqual(self.mission.state, RETURN)

    def test_the_leader_not_heard_stands_still_out_of_formation_back_to_the_route(self):
        step = self.mission.step(snap(1.0, LEADER, None), 1.0)
        self.assertEqual(self.mission.state, FORMATION)
        self.assertIsNone(step.planned.waypoint)
        self.mission.set_route(Route([offset_latlon(*LEADER, 0.0, 20.0)], 3.0, False, None))
        self.mission.step(snap(2.0, LEADER, None, follow=None), 2.0)
        self.assertEqual(self.mission.state, PATROL)

    def test_the_leader_given_a_place_0_m_right_and_back_sails_its_route_slower(self):
        self.mission.set_route(Route([offset_latlon(*LEADER, 0.0, 20.0)], 3.0, False, None))
        step = self.mission.step(snap(1.0, LEADER, None, follow=("PHAO-01", 0.0, 0.0)), 1.0)
        self.assertEqual(self.mission.state, PATROL)                      # its route, not a place beside itself
        self.assertIsNotNone(step.planned.waypoint)
        self.assertIsNone(step.planned.goal_speed)
        self.assertAlmostEqual(step.planned.speed_limit, 0.7 * 0.3)
        step = self.mission.step(snap(2.0, LEADER, None, follow=None), 2.0)
        self.assertIsNone(step.planned.speed_limit)                       # out of the formation: its own speed


if __name__ == "__main__":
    unittest.main()

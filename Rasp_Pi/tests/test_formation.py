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


if __name__ == "__main__":
    unittest.main()

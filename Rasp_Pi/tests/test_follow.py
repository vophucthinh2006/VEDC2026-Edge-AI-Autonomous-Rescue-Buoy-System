import unittest

from modules.follow import FOLLOW, SEARCH, Follow
from modules.state_store import GpsState, HumanTarget, ImuState, Snapshot, SysState
from utils.geometry import haversine_m, offset_latlon

BOAT = (10.762600, 106.660100)
WAYPOINT = (10.762700, 106.660100)


def snap(t, yaw=0.0, target=None):
    return Snapshot(ImuState(yaw_deg=yaw, imu_ok=True, timestamp=t), GpsState(*BOAT, fix=3, timestamp=t), SysState(timestamp=t),
                    (), t, WAYPOINT, target)


class FollowTest(unittest.TestCase):
    def setUp(self):
        self.follow = Follow({"confirm_frames": 3, "confirm_s": 2.5, "lost_s": 4.0})

    def sight(self, t, bearing=20.0, distance=5.0, yaw=0.0):
        return self.follow.step(snap(t, yaw, HumanTarget(bearing, distance, 0.5, t)), t)

    def test_one_sighting_is_not_enough_and_the_route_goes_on(self):
        planned, events = self.sight(0.0)
        self.assertEqual(self.follow.phase, SEARCH)
        self.assertIsNone(planned.human_target)
        self.assertEqual(planned.waypoint, WAYPOINT)
        self.assertEqual(events, [])

    def test_three_sightings_in_time_start_following_and_report_where_the_person_is(self):
        self.sight(0.0)
        self.sight(0.8)
        planned, events = self.sight(1.6)
        self.assertEqual(self.follow.phase, FOLLOW)
        self.assertEqual([e.kind for e in events], ["found"])
        self.assertAlmostEqual(planned.human_target.bearing_body_deg, 20.0)
        expected = offset_latlon(*BOAT, 20.0, 5.0)
        self.assertLess(haversine_m(events[0].lat, events[0].lon, *expected), 0.01)

    def test_without_a_gps_fix_the_person_has_no_position(self):
        self.follow.step(Snapshot(ImuState(imu_ok=True, timestamp=0.0), GpsState(fix=0, timestamp=0.0), SysState(timestamp=0.0),
                                  (), 0.0, None, HumanTarget(20.0, 5.0, 0.5, 0.0)), 0.0)
        for t in (0.8, 1.6):
            target = HumanTarget(20.0, 5.0, 0.5, t)
            _, events = self.follow.step(Snapshot(ImuState(imu_ok=True, timestamp=t), GpsState(fix=0, timestamp=t), SysState(timestamp=t),
                                                  (), t, None, target), t)
        self.assertEqual(events[0].kind, "found")
        self.assertIsNone(events[0].lat)

    def test_sightings_too_far_apart_never_confirm(self):
        for t in (0.0, 2.0, 4.0, 6.0):
            self.sight(t)
        self.assertEqual(self.follow.phase, SEARCH)

    def test_between_looks_the_person_is_held_on_their_compass_bearing(self):
        for t in (0.0, 0.8, 1.6):
            self.sight(t)
        # No new sighting; the boat has turned 15 deg to the right meanwhile.
        planned, _ = self.follow.step(snap(2.4, yaw=15.0), 2.4)
        self.assertAlmostEqual(planned.human_target.bearing_body_deg, 5.0)

    def test_out_of_sight_for_lost_s_goes_back_to_the_route(self):
        for t in (0.0, 0.8, 1.6):
            self.sight(t)
        planned, events = self.follow.step(snap(5.0), 5.0)
        self.assertEqual(self.follow.phase, FOLLOW)
        planned, events = self.follow.step(snap(5.7), 5.7)
        self.assertEqual(self.follow.phase, SEARCH)
        self.assertEqual([e.kind for e in events], ["lost"])
        self.assertIsNone(planned.human_target)

    def test_the_same_sighting_is_counted_once(self):
        target = HumanTarget(20.0, 5.0, 0.5, 0.0)
        for t in (0.0, 0.1, 0.2, 0.3):
            self.follow.step(snap(t, target=target), t)
        self.assertEqual(self.follow.phase, SEARCH)


if __name__ == "__main__":
    unittest.main()

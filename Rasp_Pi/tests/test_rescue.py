import unittest

from modules.perception import Obstacle
from modules.rescue import Phase, RescueMission
from modules.state_store import GpsState, HumanTarget, ImuState, Snapshot, SysState, VehicleState
from utils.geometry import haversine_m

VEHICLE = {"target_standoff_m": 1.5}
RESCUE = {"search_mode": "AUTO", "hold_mode": "LOITER", "after_hold": "AUTO", "confirm_s": 1.0, "hold_s": 20.0,
          "lost_s": 5.0, "arrive_m": 0.5, "revisit_radius_m": 8.0, "goto_period_s": 1.0}
LAT, LON = 10.883438, 106.796019


def snap(now, mode="AUTO", armed=True, target=None, lat=LAT, lon=LON, yaw=90.0, fix=3, seq=1, total=0, wp_dist=-1.0, obstacle_m=None, wp_bearing=270.0):
    human = HumanTarget(target[0], target[1], 0.8, now) if target else None
    obstacles = (Obstacle(0.0, obstacle_m, 0.1),) if obstacle_m is not None else ()
    return Snapshot(ImuState(yaw_deg=yaw, imu_ok=True, timestamp=now), GpsState(lat_deg=lat, lon_deg=lon, fix=fix, timestamp=now),
                    SysState(timestamp=now), obstacles, now, None, human, VehicleState(mode, armed, seq, now, total, wp_dist, wp_bearing))


def kinds(actions):
    return [(a.kind, a.mode or a.text) for a in actions]


class RescueMissionTest(unittest.TestCase):
    def test_single_frame_does_not_take_over(self):
        mission = RescueMission(VEHICLE, RESCUE)
        self.assertEqual(mission.step(snap(0.0, target=(0.0, 6.0)), 0.0), [])
        self.assertEqual(mission.step(snap(1.5), 1.5), [])                      # gone for over a second: timer restarts
        self.assertEqual(mission.step(snap(2.2, target=(0.0, 6.0)), 2.2), [])
        self.assertEqual(mission.step(snap(2.6, target=(0.0, 6.0)), 2.6), [])
        self.assertIs(mission.phase, Phase.SEARCH)

    def test_a_short_dropout_does_not_restart_the_confirmation(self):
        mission = RescueMission(VEHICLE, RESCUE)
        mission.step(snap(0.0, target=(0.0, 6.0)), 0.0)
        mission.step(snap(0.4, target=(0.0, 6.0)), 0.4)
        self.assertEqual(mission.step(snap(0.8), 0.8), [])                      # one frame missed
        actions = mission.step(snap(1.2, target=(0.0, 6.0)), 1.2)
        self.assertEqual(kinds(actions), [("mode", "GUIDED"), ("report", "VICTIM SEEN")])

    def test_two_frames_are_not_enough(self):
        mission = RescueMission(VEHICLE, RESCUE)
        mission.step(snap(0.0, target=(0.0, 6.0)), 0.0)
        self.assertEqual(mission.step(snap(0.9), 0.9), [])
        self.assertEqual(mission.step(snap(1.2, target=(0.0, 6.0)), 1.2), [])   # long enough, but only two frames
        self.assertIs(mission.phase, Phase.SEARCH)

    def test_takes_over_after_a_persistent_detection_and_locates_the_victim(self):
        mission = RescueMission(VEHICLE, RESCUE)
        mission.step(snap(0.0, target=(0.0, 6.0)), 0.0)
        mission.step(snap(0.5, target=(0.0, 6.0)), 0.5)
        actions = mission.step(snap(1.1, target=(0.0, 6.0)), 1.1)
        self.assertEqual(kinds(actions), [("mode", "GUIDED"), ("report", "VICTIM SEEN")])
        self.assertIs(mission.phase, Phase.APPROACH)
        # Boat heading east (90), person dead ahead at 6 m: 6 m east of the boat.
        self.assertAlmostEqual(haversine_m(LAT, LON, *mission.victim), 6.0, delta=0.05)
        self.assertAlmostEqual(mission.victim[0], LAT, delta=1e-6)
        self.assertGreater(mission.victim[1], LON)

    def test_ignores_detections_outside_the_search_mode_or_disarmed(self):
        for kwargs in ({"mode": "MANUAL"}, {"mode": "HOLD"}, {"armed": False}, {"fix": 1}):
            mission = RescueMission(VEHICLE, RESCUE)
            mission.step(snap(0.0, target=(0.0, 6.0), **kwargs), 0.0)
            self.assertEqual(mission.step(snap(2.0, target=(0.0, 6.0), **kwargs), 2.0), [], kwargs)

    def _approaching(self):
        mission = RescueMission(VEHICLE, RESCUE)
        mission.step(snap(0.0, target=(0.0, 6.0)), 0.0)
        mission.step(snap(0.5, target=(0.0, 6.0)), 0.5)
        mission.step(snap(1.1, target=(0.0, 6.0)), 1.1)
        return mission

    def test_aims_at_the_standoff_point_not_at_the_person(self):
        mission = self._approaching()
        actions = mission.step(snap(1.5, mode="GUIDED", target=(0.0, 6.0)), 1.5)
        self.assertEqual([a.kind for a in actions], ["goto"])
        self.assertAlmostEqual(haversine_m(actions[0].lat, actions[0].lon, *mission.victim), 1.5, delta=0.05)
        self.assertEqual(mission.step(snap(1.8, mode="GUIDED", target=(0.0, 6.0)), 1.8), [])   # goto is rate limited

    def test_waits_for_the_autopilot_to_report_guided(self):
        mission = self._approaching()
        self.assertEqual(mission.step(snap(1.5, mode="AUTO", target=(0.0, 6.0)), 1.5), [])     # still AUTO, within grace
        self.assertIs(mission.phase, Phase.APPROACH)

    def test_operator_mode_change_ends_the_takeover(self):
        mission = self._approaching()
        mission.step(snap(1.5, mode="GUIDED", target=(0.0, 6.0)), 1.5)
        self.assertEqual(mission.step(snap(5.0, mode="MANUAL", target=(0.0, 4.0)), 5.0), [])
        self.assertIs(mission.phase, Phase.SEARCH)

    def test_holds_at_the_standoff_then_resumes_and_does_not_revisit(self):
        mission = self._approaching()
        victim = mission.victim
        # The boat has moved to 1.8 m short of the person (stand-off 1.5 + arrive 0.5).
        near_lon = victim[1] - 1.8 / (111_320.0 * 0.982)
        actions = mission.step(snap(10.0, mode="GUIDED", target=(0.0, 1.8), lon=near_lon), 10.0)
        self.assertEqual(kinds(actions), [("mode", "LOITER"), ("report", "VICTIM REACHED")])
        self.assertEqual(mission.keep_outs(), [])              # not an obstacle while the boat holds next to them
        self.assertEqual(mission.step(snap(20.0, mode="LOITER", lon=near_lon), 20.0), [])
        actions = mission.step(snap(30.5, mode="LOITER", lon=near_lon), 30.5)
        self.assertEqual(kinds(actions), [("mode", "AUTO"), ("report", "RESUMING AUTO")])
        self.assertEqual(len(mission.keep_outs()), 1)          # the resumed pattern must go around them
        self.assertLess(haversine_m(*mission.keep_outs()[0], *victim), 0.1)
        mission.step(snap(31.0, mode="AUTO", lon=near_lon), 31.0)
        self.assertIs(mission.phase, Phase.SEARCH)
        # Same person still in view: already attended, no second takeover.
        mission.step(snap(32.0, target=(0.0, 1.8), lon=near_lon), 32.0)
        self.assertEqual(mission.step(snap(34.0, target=(0.0, 1.8), lon=near_lon), 34.0), [])

    def test_stops_short_when_a_wall_is_close_ahead_on_the_last_metres(self):
        mission = self._approaching()
        victim = mission.victim
        # 3.5 m from the person (stand-off 1.5), a wall 1.2 m ahead: hold here.
        near_lon = victim[1] - 3.5 / (111_320.0 * 0.982)
        actions = mission.step(snap(5.0, mode="GUIDED", target=(0.0, 3.5), lon=near_lon, obstacle_m=1.2), 5.0)
        self.assertEqual(kinds(actions), [("mode", "LOITER"), ("report", "VICTIM REACHED")])
        # The same wall while still 6 m out is the autopilot's business: keep going.
        mission = self._approaching()
        self.assertEqual(mission.step(snap(2.0, mode="GUIDED", target=(0.0, 6.0), obstacle_m=1.2), 2.0)[0].kind, "goto")
        # Unless it is about to touch the hulls.
        actions = mission.step(snap(3.0, mode="GUIDED", target=(0.0, 6.0), obstacle_m=0.7), 3.0)
        self.assertEqual(kinds(actions), [("mode", "LOITER"), ("report", "VICTIM UNREACHABLE")])

    def test_reports_a_person_it_cannot_get_closer_to(self):
        mission = self._approaching()
        victim = mission.victim
        # Stuck 4 m short (a house in the way) with the person in view all along.
        stuck_lon = victim[1] - 4.0 / (111_320.0 * 0.982)
        self.assertEqual(mission.step(snap(2.0, mode="GUIDED", target=(0.0, 4.0), lon=stuck_lon), 2.0)[0].kind, "goto")
        self.assertEqual(mission.step(snap(11.5, mode="GUIDED", target=(0.0, 4.0), lon=stuck_lon), 11.5)[0].kind, "goto")
        self.assertIsNotNone(mission.attending())
        actions = mission.step(snap(12.5, mode="GUIDED", target=(0.0, 4.0), lon=stuck_lon), 12.5)
        self.assertEqual(kinds(actions), [("mode", "LOITER"), ("report", "VICTIM UNREACHABLE")])
        self.assertIs(mission.phase, Phase.HOLD)
        actions = mission.step(snap(33.0, mode="LOITER", lon=stuck_lon), 33.0)
        self.assertEqual(kinds(actions), [("mode", "AUTO"), ("report", "RESUMING AUTO")])
        self.assertEqual(len(mission.keep_outs()), 1)          # not approached again

    def test_gives_up_when_the_person_is_lost_on_the_way(self):
        mission = self._approaching()
        mission.step(snap(1.5, mode="GUIDED", target=(0.0, 6.0)), 1.5)
        self.assertEqual(mission.step(snap(4.0, mode="GUIDED"), 4.0)[0].kind, "goto")          # keeps going on the last estimate
        actions = mission.step(snap(7.0, mode="GUIDED"), 7.0)
        self.assertEqual(kinds(actions), [("mode", "AUTO"), ("report", "VICTIM LOST")])


class BlockedWaypointTest(unittest.TestCase):
    def test_skips_a_waypoint_the_boat_cannot_get_closer_to(self):
        mission = RescueMission(VEHICLE, RESCUE)
        blocked = {"seq": 3, "total": 7, "wp_dist": 3.0, "obstacle_m": 2.5}
        self.assertEqual(mission.step(snap(0.0, **blocked), 0.0), [])
        self.assertEqual(mission.step(snap(19.0, **blocked), 19.0), [])
        actions = mission.step(snap(21.0, **blocked), 21.0)
        self.assertEqual([(a.kind, a.seq or a.text) for a in actions], [("skip", 4), ("report", "WP 3 BLOCKED, SKIPPED")])
        self.assertEqual(mission.step(snap(22.0, **blocked), 22.0), [])          # one skip, then the clock restarts

    def test_progress_a_far_waypoint_or_open_water_do_not_skip(self):
        for label, later in (("progress", {"wp_dist": 2.0, "obstacle_m": 2.5}), ("far", {"wp_dist": 20.0, "obstacle_m": 2.5}),
                             ("open water", {"wp_dist": 3.0}), ("next item", {"wp_dist": 3.0, "obstacle_m": 2.5, "seq": 4})):
            mission = RescueMission(VEHICLE, RESCUE)
            first = {"wp_dist": 20.0 if label == "far" else 3.0, "obstacle_m": 2.5}
            mission.step(snap(0.0, seq=3, total=7, **first), 0.0)
            self.assertEqual(mission.step(snap(30.0, total=7, **{"seq": 3, **later}), 30.0), [], label)

    def test_the_last_item_is_reported_not_skipped(self):
        mission = RescueMission(VEHICLE, RESCUE)
        mission.step(snap(0.0, seq=6, total=6, wp_dist=3.0, obstacle_m=2.0), 0.0)
        actions = mission.step(snap(30.0, seq=6, total=6, wp_dist=3.0, obstacle_m=2.0), 30.0)
        self.assertEqual(kinds(actions), [("report", "WP 6 BLOCKED")])

    def test_skips_at_once_a_waypoint_with_something_on_it(self):
        mission = RescueMission(VEHICLE, RESCUE)
        # Heading east (90), waypoint 5 m dead ahead, a LiDAR return 4.2 m dead ahead: 0.8 m from the waypoint.
        on_it = {"seq": 3, "total": 7, "wp_dist": 5.0, "wp_bearing": 90.0, "obstacle_m": 4.2}
        self.assertEqual(mission.step(snap(0.0, **on_it), 0.0), [])
        self.assertEqual(mission.step(snap(1.0, **on_it), 1.0), [])
        self.assertEqual(mission.step(snap(2.0, **on_it), 2.0), [])
        actions = mission.step(snap(4.5, **on_it), 4.5)                      # 3 s after it was first seen there
        self.assertEqual([(a.kind, a.seq or a.text) for a in actions], [("skip", 4), ("report", "WP 3 BLOCKED, SKIPPED")])
        # The same return with the waypoint 8 m ahead is 3.8 m short of it: not on it.
        mission = RescueMission(VEHICLE, RESCUE)
        clear = {**on_it, "wp_dist": 8.0}
        mission.step(snap(0.0, **clear), 0.0)
        self.assertEqual(mission.step(snap(5.0, **clear), 5.0), [])

    def test_time_spent_holding_next_to_a_person_does_not_count(self):
        mission = RescueMission(VEHICLE, RESCUE)
        near = {"seq": 1, "total": 2, "wp_dist": 3.0, "obstacle_m": 2.0}
        mission.step(snap(0.0, target=(0.0, 1.8), **near), 0.0)
        mission.step(snap(0.5, target=(0.0, 1.8), **near), 0.5)
        mission.step(snap(1.1, target=(0.0, 1.8), **near), 1.1)                     # takeover
        mission.step(snap(2.0, mode="GUIDED", target=(0.0, 1.8), **near), 2.0)      # already at the stand-off: hold
        mission.step(snap(23.0, mode="LOITER", **near), 23.0)                       # resume
        mission.step(snap(23.5, mode="AUTO", **near), 23.5)
        self.assertIs(mission.phase, Phase.SEARCH)
        self.assertEqual(mission.step(snap(30.0, **near), 30.0), [])
        self.assertEqual(mission.step(snap(40.0, **near), 40.0), [])                # 16.5 s since the search resumed

    def test_not_outside_the_search_mode(self):
        mission = RescueMission(VEHICLE, RESCUE)
        mission.step(snap(0.0, mode="HOLD", seq=3, total=7, wp_dist=3.0, obstacle_m=2.0), 0.0)
        self.assertEqual(mission.step(snap(30.0, mode="HOLD", seq=3, total=7, wp_dist=3.0, obstacle_m=2.0), 30.0), [])


if __name__ == "__main__":
    unittest.main()

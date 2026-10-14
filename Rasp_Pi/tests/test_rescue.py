import unittest

from modules.perception import Obstacle
from modules.rescue import Phase, RescueMission
from modules.state_store import GpsState, HumanTarget, ImuState, Snapshot, SysState, VehicleState
from utils.geometry import haversine_m

VEHICLE = {"target_standoff_m": 1.5}
RESCUE = {"search_mode": "AUTO", "hold_mode": "LOITER", "after_hold": "AUTO", "confirm_s": 1.0, "hold_s": 20.0,
          "lost_s": 5.0, "arrive_m": 0.5, "revisit_radius_m": 8.0, "goto_period_s": 1.0}
LAT, LON = 10.883438, 106.796019


def snap(now, mode="AUTO", armed=True, target=None, lat=LAT, lon=LON, yaw=90.0, fix=3, seq=1, total=0, wp_dist=-1.0, obstacle_m=None, wp_bearing=270.0, others=()):
    """others: more people in the same frame, as (bearing from the bow, range)."""
    human = HumanTarget(target[0], target[1], 0.8, now) if target else None
    everybody = tuple(([human] if human else []) + [HumanTarget(b, d, 0.7, now) for b, d in others])
    obstacles = (Obstacle(0.0, obstacle_m, 0.1),) if obstacle_m is not None else ()
    return Snapshot(ImuState(yaw_deg=yaw, imu_ok=True, timestamp=now), GpsState(lat_deg=lat, lon_deg=lon, fix=fix, timestamp=now),
                    SysState(timestamp=now), obstacles, now, None, human, VehicleState(mode, armed, seq, now, total, wp_dist, wp_bearing), everybody)


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


class SeveralPeopleTest(unittest.TestCase):
    # Boat heading east. Person A 6 m dead ahead; person B 6.7 m away, 26.6 deg to port: 3 m north of A.
    A, B = (0.0, 6.0), (-26.6, 6.71)

    def _seen_both(self):
        mission = RescueMission(VEHICLE, {**RESCUE, "revisit_radius_m": 2.0})
        for t in (0.0, 0.5):
            self.assertEqual(mission.step(snap(t, target=self.A, others=[self.B]), t), [])
        return mission

    def test_two_people_three_metres_apart_are_two_people(self):
        mission = self._seen_both()
        actions = mission.step(snap(1.1, target=self.A, others=[self.B]), 1.1)
        self.assertEqual(kinds(actions), [("mode", "GUIDED"), ("report", "VICTIM SEEN"), ("report", "PEOPLE 2 FOUND 0 ATTENDED")])
        self.assertEqual(len(mission.people), 2)
        self.assertAlmostEqual(haversine_m(mission.people[0].position[0], mission.people[0].position[1], *mission.people[1].position), 3.0, delta=0.1)
        # The nearer one first; the other is kept clear of on the way.
        self.assertAlmostEqual(haversine_m(LAT, LON, *mission.victim), 6.0, delta=0.05)
        self.assertEqual(len(mission.keep_outs()), 1)

    def test_the_other_person_in_the_frame_does_not_pull_the_boat_over(self):
        mission = self._seen_both()
        mission.step(snap(1.1, target=self.A, others=[self.B]), 1.1)
        going_to = mission.victim
        # The detector now lists B first (the taller box, say): the boat still goes to A.
        mission.step(snap(1.6, mode="GUIDED", target=self.B, others=[self.A]), 1.6)
        self.assertLess(haversine_m(*mission.victim, *going_to), 0.1)
        # And with only B in view for a moment, A stays where A was.
        mission.step(snap(2.1, mode="GUIDED", target=self.B), 2.1)
        self.assertLess(haversine_m(*mission.victim, *going_to), 0.1)

    def test_found_counts_confirmed_people_only(self):
        mission = RescueMission(VEHICLE, dict(RESCUE, revisit_radius_m=2.0))
        self.assertEqual(mission.found, 0)
        mission.step(snap(0.0, target=(-10.0, 5.0), others=((25.0, 5.5),)), 0.0)
        self.assertEqual(mission.found, 0)                      # one frame is nobody yet
        for t in (0.5, 1.0, 1.5):
            mission.step(snap(t, target=(-10.0, 5.0), others=((25.0, 5.5),)), t)
        self.assertEqual(mission.found, 2)

    def test_after_the_first_the_boat_goes_to_the_second(self):
        mission = self._seen_both()
        mission.step(snap(1.1, target=self.A, others=[self.B]), 1.1)
        first = mission.victim
        near_lon = first[1] - 1.8 / (111_320.0 * 0.982)                     # 1.8 m short of A
        actions = mission.step(snap(10.0, mode="GUIDED", target=(0.0, 1.8), lon=near_lon), 10.0)
        self.assertEqual(kinds(actions), [("mode", "LOITER"), ("report", "VICTIM REACHED"), ("report", "PEOPLE 2 FOUND 1 ATTENDED")])
        actions = mission.step(snap(30.5, mode="LOITER", lon=near_lon), 30.5)
        self.assertEqual(kinds(actions), [("mode", "AUTO"), ("report", "RESUMING AUTO")])
        mission.step(snap(31.0, mode="AUTO", lon=near_lon), 31.0)             # hand-back seen
        actions = mission.step(snap(31.2, mode="AUTO", lon=near_lon), 31.2)   # B was never forgotten
        self.assertEqual(kinds(actions), [("mode", "GUIDED"), ("report", "VICTIM SEEN")])
        self.assertAlmostEqual(haversine_m(*mission.victim, *first), 3.0, delta=0.1)
        self.assertEqual(len(mission.keep_outs()), 1)                         # A, now attended
        self.assertLess(haversine_m(*mission.keep_outs()[0], *first), 0.1)

    def test_a_range_that_reads_metres_differently_is_still_the_same_person(self):
        mission = RescueMission(VEHICLE, {**RESCUE, "revisit_radius_m": 2.0})
        for t, where in ((0.0, (0.0, 6.0)), (0.5, (1.0, 3.6)), (1.1, (-1.0, 6.2))):    # the box cut by the frame, then whole again
            mission.step(snap(t, target=where), t)
        self.assertEqual(len(mission.people), 1)

    def test_the_second_person_unseen_during_the_hold_is_not_lost_at_once(self):
        mission = self._seen_both()
        mission.step(snap(1.1, target=self.A, others=[self.B]), 1.1)
        near_lon = mission.victim[1] - 1.8 / (111_320.0 * 0.982)
        mission.step(snap(10.0, mode="GUIDED", target=(0.0, 1.8), lon=near_lon), 10.0)       # A reached; B out of the frame from here on
        mission.step(snap(30.5, mode="LOITER", lon=near_lon), 30.5)
        mission.step(snap(31.0, mode="AUTO", lon=near_lon), 31.0)
        mission.step(snap(31.2, mode="AUTO", lon=near_lon), 31.2)                            # takes B on
        self.assertEqual(mission.step(snap(32.0, mode="GUIDED", lon=near_lon), 32.0)[0].kind, "goto")
        # Still not seen 5 s into the approach: given up, kept on the map, not gone to again unseen.
        actions = mission.step(snap(36.5, mode="GUIDED", lon=near_lon), 36.5)
        self.assertEqual(kinds(actions), [("mode", "AUTO"), ("report", "VICTIM LOST")])
        mission.step(snap(37.0, mode="AUTO", lon=near_lon), 37.0)
        self.assertEqual(mission.step(snap(37.2, mode="AUTO", lon=near_lon), 37.2), [])
        self.assertEqual(len(mission.keep_outs()), 2)                                        # A and where B was

    def test_detections_a_metre_apart_are_one_person(self):
        mission = RescueMission(VEHICLE, {**RESCUE, "revisit_radius_m": 2.0})
        for t, where in ((0.0, (0.0, 6.0)), (0.5, (8.0, 6.3)), (1.1, (2.0, 5.8))):      # the same person, read a little differently
            mission.step(snap(t, target=where), t)
        self.assertEqual(len(mission.people), 1)
        self.assertIs(mission.phase, Phase.APPROACH)


class FacingTest(unittest.TestCase):
    """The bow follows the camera: turn to the person, then go; and hold with the bow on them."""
    CONFIG = {**RESCUE, "revisit_radius_m": 2.0, "hold_mode": "FACE", "turn_first": True}
    SIDE = (50.0, 6.0)             # 50 deg to starboard of a boat heading east: true bearing 140

    def _taken_over(self, where):
        mission = RescueMission(VEHICLE, self.CONFIG)
        for t in (0.0, 0.5, 1.1):
            mission.step(snap(t, target=where), t)
        self.assertIs(mission.phase, Phase.APPROACH)
        return mission

    def test_the_bow_is_turned_to_the_person_before_the_boat_goes_to_them(self):
        mission = self._taken_over(self.SIDE)
        actions = mission.step(snap(1.5, mode="GUIDED", target=self.SIDE), 1.5)
        self.assertEqual([a.kind for a in actions], ["face"])
        self.assertAlmostEqual(actions[0].heading, 140.0, delta=0.5)
        self.assertEqual(actions[0].speed, 0.3)
        self.assertEqual(mission.step(snap(1.8, mode="GUIDED", target=self.SIDE), 1.8), [])       # rate limited, like goto
        # The boat has turned to 136: they are 4 deg off the bow now. From here it goes to them.
        actions = mission.step(snap(2.6, mode="GUIDED", yaw=136.0, target=(4.0, 6.0)), 2.6)
        self.assertEqual([a.kind for a in actions], ["goto"])
        # And it does not go back to turning if the bow swings off again on the way.
        actions = mission.step(snap(3.7, mode="GUIDED", yaw=110.0, target=(30.0, 6.0)), 3.7)
        self.assertEqual([a.kind for a in actions], ["goto"])

    def test_turning_is_given_up_after_a_while(self):
        mission = self._taken_over(self.SIDE)
        self.assertEqual(mission.step(snap(5.0, mode="GUIDED", target=self.SIDE), 5.0)[0].kind, "face")
        self.assertEqual(mission.step(snap(9.5, mode="GUIDED", target=self.SIDE), 9.5)[0].kind, "goto")    # 8 s after the takeover

    def test_somebody_dead_ahead_is_gone_to_at_once(self):
        mission = self._taken_over((3.0, 6.0))
        self.assertEqual(mission.step(snap(1.5, mode="GUIDED", target=(3.0, 6.0)), 1.5)[0].kind, "goto")

    def _holding(self):
        mission = self._taken_over((0.0, 6.0))
        victim = mission.victim
        short = lambda metres: victim[1] - metres / (111_320.0 * 0.982)      # this far west of them, on their latitude
        actions = mission.step(snap(10.0, mode="GUIDED", target=(0.0, 1.8), lon=short(1.8)), 10.0)
        self.assertEqual(kinds(actions), [("face", ""), ("report", "VICTIM REACHED")])       # no mode change: the Pi holds in GUIDED
        self.assertEqual(actions[0].speed, 0.0)
        self.assertIs(mission.phase, Phase.HOLD)
        return mission, short

    def _hold_step(self, mission, t, **where):
        actions = mission.step(snap(t, mode="GUIDED", **where), t)
        self.assertEqual([a.kind for a in actions], ["face"])
        return actions[0]

    def test_holds_with_the_bow_on_the_person(self):
        mission, short = self._holding()
        self.assertEqual(mission.step(snap(10.4, mode="GUIDED", lon=short(1.8)), 10.4), [])       # rate limited
        # In place, bow on them: stand still.
        face = self._hold_step(mission, 11.5, lon=short(1.8))
        self.assertEqual((face.speed, round(face.heading)), (0.0, 90))
        # Drifted to 3 m: creep back, heading for them.
        self.assertEqual(self._hold_step(mission, 13.0, lon=short(3.0)).speed, 0.3)
        # In place but the bow 50 deg off them: creep round.
        face = self._hold_step(mission, 14.5, lon=short(1.8), yaw=40.0)
        self.assertEqual((face.speed, round(face.heading)), (0.3, 90))

    def test_the_hold_never_drives_the_boat_closer_than_the_standoff_or_onto_something(self):
        mission, short = self._holding()
        # 1.2 m from them (stand-off 1.5), bow well off: still no creeping.
        self.assertEqual(self._hold_step(mission, 11.5, lon=short(1.2), yaw=40.0).speed, 0.0)
        # Drifted out, but something 0.7 m from the LiDAR: stand still.
        self.assertEqual(self._hold_step(mission, 13.0, lon=short(3.0), obstacle_m=0.7).speed, 0.0)

    def test_the_hold_ends_like_any_other_and_the_operator_still_wins(self):
        mission, short = self._holding()
        self._hold_step(mission, 20.0, lon=short(1.8))
        actions = mission.step(snap(30.5, mode="GUIDED", lon=short(1.8)), 30.5)
        self.assertEqual(kinds(actions), [("mode", "AUTO"), ("report", "RESUMING AUTO")])
        mission, short = self._holding()
        self.assertEqual(mission.step(snap(15.0, mode="HOLD", lon=short(1.8)), 15.0), [])         # E-stop, RC switch, ground station
        self.assertIs(mission.phase, Phase.SEARCH)

    def test_by_default_neither_is_on(self):
        mission = RescueMission(VEHICLE, {**RESCUE, "revisit_radius_m": 2.0})
        for t in (0.0, 0.5, 1.1):
            mission.step(snap(t, target=self.SIDE), t)
        self.assertEqual(mission.step(snap(1.5, mode="GUIDED", target=self.SIDE), 1.5)[0].kind, "goto")    # no turning first
        self.assertFalse(mission.hold_facing)

    def test_with_loiter_the_autopilot_holds_as_before(self):
        mission = RescueMission(VEHICLE, {**self.CONFIG, "hold_mode": "LOITER"})
        for t in (0.0, 0.5, 1.1):
            mission.step(snap(t, target=(0.0, 6.0)), t)
        near_lon = mission.victim[1] - 1.8 / (111_320.0 * 0.982)
        actions = mission.step(snap(10.0, mode="GUIDED", target=(0.0, 1.8), lon=near_lon), 10.0)
        self.assertEqual(kinds(actions), [("mode", "LOITER"), ("report", "VICTIM REACHED")])
        self.assertEqual(mission.step(snap(20.0, mode="LOITER", lon=near_lon), 20.0), [])


class AlreadyAttendedTest(unittest.TestCase):
    """Nobody is gone to twice, and people standing together are one stop."""
    CONFIG = {**RESCUE, "revisit_radius_m": 2.0}

    def _attended(self, frame=((0.0, 6.0), ())):
        """A person 6 m east of the start reached, held next to, and the search resumed. frame: the
        followed person and the others in view while they are confirmed."""
        mission = RescueMission(VEHICLE, self.CONFIG)
        for t in (0.0, 0.5, 1.1):
            mission.step(snap(t, target=frame[0], others=frame[1]), t)
        self.assertIs(mission.phase, Phase.APPROACH)
        victim = mission.victim
        near_lon = victim[1] - 1.8 / (111_320.0 * 0.982)
        reached = mission.step(snap(10.0, mode="GUIDED", target=(0.0, 1.8), lon=near_lon), 10.0)
        mission.step(snap(30.5, mode="LOITER", lon=near_lon), 30.5)
        mission.step(snap(31.0, mode="AUTO", lon=near_lon), 31.0)
        self.assertIs(mission.phase, Phase.SEARCH)
        return mission, victim, reached

    def test_seen_again_from_afar_with_a_poor_range_is_not_somebody_new(self):
        mission, victim, _ = self._attended()
        # Back at the start, 6 m from them, looking straight at them. The box reads 3.5 m, then
        # 8.5, 10 and 9 m: each used to move their place, until the next one no longer fell on it.
        for t, read in ((40.0, 3.5), (40.5, 8.5), (41.0, 10.0), (41.5, 9.0), (42.0, 4.0), (42.5, 9.5)):
            self.assertEqual(mission.step(snap(t, target=(0.0, read)), t), [], read)
        self.assertEqual(len(mission.people), 1)
        self.assertIs(mission.phase, Phase.SEARCH)
        self.assertLess(haversine_m(*mission.people[0].position, *victim), 0.1)       # where they were reached, still
        self.assertEqual(mission.found, 1)

    def test_somebody_well_to_the_side_of_them_is_still_gone_to(self):
        mission, victim, _ = self._attended()
        # 45 deg to port at 8.5 m: 6 m north of the attended person.
        for t in (40.0, 40.5):
            self.assertEqual(mission.step(snap(t, target=(-45.0, 8.5)), t), [])
        actions = mission.step(snap(41.1, target=(-45.0, 8.5)), 41.1)
        self.assertEqual(kinds(actions), [("mode", "GUIDED"), ("report", "VICTIM SEEN"), ("report", "PEOPLE 2 FOUND 1 ATTENDED")])
        self.assertAlmostEqual(haversine_m(*mission.victim, *victim), 6.0, delta=0.2)

    def test_three_people_together_are_one_stop(self):
        # A dead ahead at 6 m, B and C 12 deg either side of A: 1.3 m from A, three people on the map.
        mission, victim, reached = self._attended(frame=((0.0, 6.0), ((-12.0, 6.1), (12.0, 6.1))))
        self.assertEqual(len(mission.people), 3)
        self.assertEqual(kinds(reached), [("mode", "LOITER"), ("report", "VICTIM REACHED"), ("report", "PEOPLE 3 FOUND 3 ATTENDED")])
        self.assertTrue(all(person.attended for person in mission.people))
        self.assertEqual(mission.step(snap(31.2, mode="AUTO"), 31.2), [])                 # nobody left to go to
        self.assertEqual(len(mission.keep_outs()), 3)
        # The three seen again on the way back: still nobody new.
        for t in (40.0, 40.5, 41.1, 41.6):
            self.assertEqual(mission.step(snap(t, target=(0.0, 7.5), others=((-12.0, 4.8), (12.0, 8.0))), t), [])
        self.assertEqual(mission.found, 3)

    def test_somebody_turning_up_next_to_them_later_is_of_their_group(self):
        mission, _, _ = self._attended()
        # 14 deg to starboard at 6.2 m: 1.5 m from the attended person.
        for t in (40.0, 40.5, 41.1, 41.6):
            self.assertEqual(mission.step(snap(t, target=(14.0, 6.2)), t), [])
        self.assertEqual(len(mission.people), 1)

    def test_somebody_who_turns_out_to_stand_next_to_them_is_not_a_second_stop(self):
        # B, seen with A from the start, read 9 m away at 40 deg to port: 6 m from A, somebody else.
        mission, victim, _ = self._attended(frame=((0.0, 6.0), ((-40.0, 9.0),)))
        self.assertEqual(len(mission.people), 2)
        near_lon = victim[1] - 1.8 / (111_320.0 * 0.982)
        actions = mission.step(snap(31.2, mode="AUTO", lon=near_lon), 31.2)
        self.assertEqual(kinds(actions), [("mode", "GUIDED"), ("report", "VICTIM SEEN")])
        # From here B is 65 deg to port. The range, 9 m at first, now reads 3.6 m: still apart from A.
        actions = mission.step(snap(32.5, mode="GUIDED", lon=near_lon, target=(-62.0, 3.6)), 32.5)
        self.assertIn(actions[0].kind, ("goto", "face"))
        # Then, the box whole, 2.4 m: that puts B two metres from A. One of A's group, no second stop.
        actions = mission.step(snap(33.6, mode="GUIDED", lon=near_lon, target=(-58.0, 2.4)), 33.6)
        self.assertEqual(kinds(actions), [("mode", "AUTO"), ("report", "SAME GROUP, RESUMING AUTO"), ("report", "PEOPLE 2 FOUND 2 ATTENDED")])
        mission.step(snap(34.0, mode="AUTO", lon=near_lon), 34.0)
        self.assertEqual(mission.step(snap(34.2, mode="AUTO", lon=near_lon), 34.2), [])
        self.assertTrue(all(person.attended for person in mission.people))

    def test_describes_what_it_is_doing_for_the_map(self):
        mission = RescueMission(VEHICLE, self.CONFIG)
        idle = mission.describe(snap(0.0, mode="MANUAL"), 0.0)
        self.assertTrue(idle["lines"][0].startswith("IDLE"))
        self.assertTrue(mission.describe(snap(0.0), 0.0)["lines"][0].startswith("SEARCH"))
        for t in (0.0, 0.5, 1.1):
            mission.step(snap(t, target=(0.0, 6.0), others=((60.0, 7.0),)), t)
        told = mission.describe(snap(1.5, mode="GUIDED", target=(0.0, 6.0)), 1.5)
        self.assertEqual(told["phase"], "approach")
        self.assertIn("APPROACH: person 6.0 m away at 090 deg", told["lines"][0])
        self.assertEqual(sorted(status for _lat, _lon, status in told["people"]), ["target", "waiting"])
        self.assertIn("2 found, 0 attended", told["lines"][-1])
        mission, _victim, _ = self._attended()
        self.assertEqual([status for _lat, _lon, status in mission.describe(snap(31.2), 31.2)["people"]], ["attended"])

    def test_two_people_three_metres_apart_stay_two_stops(self):
        mission, victim, reached = self._attended(frame=(SeveralPeopleTest.A, (SeveralPeopleTest.B,)))
        self.assertEqual(kinds(reached)[-1], ("report", "PEOPLE 2 FOUND 1 ATTENDED"))
        actions = mission.step(snap(31.2, mode="AUTO", lon=victim[1] - 1.8 / (111_320.0 * 0.982)), 31.2)
        self.assertEqual(kinds(actions), [("mode", "GUIDED"), ("report", "VICTIM SEEN")])


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
        for label, later in (("progress", {"wp_dist": 2.0, "obstacle_m": 2.5}), ("far, not for long", {"wp_dist": 20.0, "obstacle_m": 2.5}),
                             ("open water", {"wp_dist": 3.0}), ("next item", {"wp_dist": 3.0, "obstacle_m": 2.5, "seq": 4})):
            mission = RescueMission(VEHICLE, RESCUE)
            first = {"wp_dist": 20.0 if label.startswith("far") else 3.0, "obstacle_m": 2.5}
            mission.step(snap(0.0, seq=3, total=7, **first), 0.0)
            self.assertEqual(mission.step(snap(30.0, total=7, **{"seq": 3, **later}), 30.0), [], label)

    def test_a_blocked_last_item_stops_the_boat(self):
        mission = RescueMission(VEHICLE, RESCUE)
        mission.step(snap(0.0, seq=6, total=6, wp_dist=3.0, obstacle_m=2.0), 0.0)
        actions = mission.step(snap(30.0, seq=6, total=6, wp_dist=3.0, obstacle_m=2.0), 30.0)
        self.assertEqual(kinds(actions), [("mode", "HOLD"), ("report", "WP 6 BLOCKED, HOLD")])
        self.assertEqual(mission.step(snap(31.0, mode="HOLD", seq=6, total=6, wp_dist=3.0, obstacle_m=2.0), 31.0), [])

    def test_waiting_at_a_waypoint_is_not_being_blocked(self):
        # A waypoint with a Delay, next to a house: the boat sits 1.5 m from it for a minute.
        mission = RescueMission(VEHICLE, RESCUE)
        waiting = {"seq": 3, "total": 7, "wp_dist": 1.5, "obstacle_m": 2.0}
        mission.step(snap(0.0, **waiting), 0.0)
        self.assertEqual(mission.step(snap(30.0, **waiting), 30.0), [])
        self.assertEqual(mission.step(snap(60.0, **waiting), 60.0), [])

    def test_skips_a_far_waypoint_it_makes_no_progress_to(self):
        # 12 m short of a waypoint in the middle of a large building, nothing within 4 m of the LiDAR.
        mission = RescueMission(VEHICLE, RESCUE)
        far = {"seq": 3, "total": 7, "wp_dist": 12.0}
        mission.step(snap(0.0, **far), 0.0)
        self.assertEqual(mission.step(snap(110.0, **far), 110.0), [])
        actions = mission.step(snap(121.0, **far), 121.0)
        self.assertEqual([(a.kind, a.seq or a.text) for a in actions], [("skip", 4), ("report", "WP 3 BLOCKED, SKIPPED")])
        # Getting closer, however slowly, restarts the clock.
        mission = RescueMission(VEHICLE, RESCUE)
        mission.step(snap(0.0, **far), 0.0)
        mission.step(snap(100.0, seq=3, total=7, wp_dist=11.0), 100.0)
        self.assertEqual(mission.step(snap(200.0, seq=3, total=7, wp_dist=11.0), 200.0), [])

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

"""Rescue mission logic on top of the autopilot: search, approach, hold, report, resume.

Pure decisions: step() reads a Snapshot and returns the actions to take, the caller sends them
over MAVLink. The autopilot runs the search pattern (AUTO) on its own; the Pi takes over only
once a person has been seen for long enough, and gives the boat back afterwards.

The operator always wins: any mode change the Pi did not ask for (RC switch, ground station)
ends the takeover at once.

A search waypoint can land on something the operator could not see on the map: a tree, a car, a
roof. The autopilot's avoidance then keeps the boat nosing around in front of it for ever. Three
rules move the mission on to the next item, with a report to the shore: the LiDAR shows something
right at the waypoint (decided from a distance, before the boat closes in); the boat is near its
waypoint, gets no closer for a while and has something on the LiDAR close by; or, however far the
waypoint (the middle of a large building), the boat has not come closer than its best for a long
while. When the blocked waypoint is the last item there is nothing to move on to: the boat is put
in blocked_last_mode (HOLD) instead of circling.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from modules.state_store import HumanTarget, Snapshot
from utils.geometry import bearing_deg, haversine_m, offset_latlon, signed_angle_deg

MODE_GRACE_S = 2.0   # the autopilot needs a moment to report a mode the Pi has just asked for
STALL_PROGRESS_M = 0.3   # getting this much closer counts as progress on the approach


class Phase(Enum):
    SEARCH = "search"        # autopilot in the search mode, Pi watching
    APPROACH = "approach"    # GUIDED towards the stand-off point
    HOLD = "hold"            # holding next to the person
    HANDBACK = "handback"    # search mode (or RTL) requested, waiting for it to take effect


@dataclass(frozen=True)
class Action:
    kind: str                # "mode" | "goto" | "report" | "skip"
    mode: str = ""
    seq: int = 0             # skip: the mission item to continue from
    lat: float = 0.0
    lon: float = 0.0
    confidence: float = 0.0
    text: str = ""


@dataclass
class Person:
    """Somebody the detector has seen, kept as a place on the map. A detection in their direction
    and at about their range is this person again; anything else is somebody else."""
    position: tuple[float, float]
    first_seen: float
    last_seen: float = 0.0
    last_stamp: float = -1.0          # detector timestamp of the last frame they were in
    sightings: int = 0                # frames they were in
    confidence: float = 0.0           # best so far
    confirmed: bool = False           # seen often and long enough to go to
    attended_at: float | None = None  # when the boat reached them (or gave up getting closer)
    lost_at: float | None = None      # when an approach to them was given up for want of seeing them

    @property
    def attended(self) -> bool:
        return self.attended_at is not None


class RescueMission:
    def __init__(self, vehicle: dict, rescue: dict) -> None:
        self.standoff_m = float(vehicle["target_standoff_m"])
        self.search_mode = str(rescue["search_mode"]).upper()
        self.hold_mode = str(rescue["hold_mode"]).upper()
        self.after_hold = str(rescue["after_hold"]).upper()       # the search mode to resume, or RTL
        self.confirm_s = float(rescue["confirm_s"])
        self.confirm_gap_s = float(rescue.get("confirm_gap_s", 1.0))
        self.confirm_frames = int(rescue.get("confirm_frames", 3))
        self.hold_s = float(rescue["hold_s"])
        self.lost_s = float(rescue["lost_s"])
        self.arrive_m = float(rescue["arrive_m"])
        self.same_person_m = float(rescue["revisit_radius_m"])
        self.same_person_range_m = float(rescue.get("same_person_range_m", 3.0))
        self.goto_period_s = float(rescue["goto_period_s"])
        self.stall_s = float(rescue.get("stall_s", 10.0))
        self.hold_clear_m = float(rescue.get("hold_clear_m", 1.5))
        self.hold_clear_within_m = float(rescue.get("hold_clear_within_m", 3.0))
        self.hold_min_m = float(rescue.get("hold_min_m", 0.9))
        self.blocked_s = float(rescue.get("blocked_s", 20.0))
        self.blocked_within_m = float(rescue.get("blocked_within_m", 6.0))
        self.blocked_obstacle_m = float(rescue.get("blocked_obstacle_m", 4.0))
        self.blocked_clear_m = float(rescue.get("blocked_clear_m", 1.5))
        self.blocked_clear_s = float(rescue.get("blocked_clear_s", 3.0))
        self.blocked_far_s = float(rescue.get("blocked_far_s", 120.0))
        self.blocked_arrived_m = float(rescue.get("blocked_arrived_m", 2.5))
        self.blocked_last_mode = str(rescue.get("blocked_last_mode", "HOLD")).upper()
        self._wp_occupied_since: float | None = None
        self._wp = (-1, 0.0, 0.0)                                 # mission item, best distance to it, when
        self.phase = Phase.SEARCH
        self.people: list[Person] = []                            # everybody seen so far
        self.target: Person | None = None                         # the one the boat is going to or holding next to
        self._counts = (0, 0)                                     # people found, attended, as last told to the shore
        self._hold_since = 0.0
        self._approach_since = 0.0
        self._last_goto = 0.0
        self._best_range = 0.0
        self._progress_at = 0.0
        self._expected_mode = ""
        self._mode_asked_at = 0.0

    @property
    def victim(self) -> tuple[float, float] | None:
        """Best estimate of where the person being attended is."""
        return self.target.position if self.target else None

    @property
    def confidence(self) -> float:
        return self.target.confidence if self.target else 0.0

    @property
    def reported(self) -> list[tuple[float, float]]:
        """People already attended, in the order the boat got to them."""
        return [p.position for p in sorted((p for p in self.people if p.attended), key=lambda p: p.attended_at)]

    # A detection gives bearing from the bow and range; the boat's fix and heading place it on the map.
    @staticmethod
    def _locate(snap: Snapshot, target: HumanTarget) -> tuple[float, float]:
        return offset_latlon(snap.gps.lat_deg, snap.gps.lon_deg, snap.imu.yaw_deg + target.bearing_body_deg, target.distance_m)

    def _track(self, snap: Snapshot, now: float) -> None:
        """Put every person in the frame on the map: the same person as before, or a new one."""
        if snap.gps.fix < 3:
            return
        targets = snap.human_targets or ((snap.human_target,) if snap.human_target is not None else ())
        for target in targets:
            if now - target.timestamp > 1.0:
                continue
            position = self._locate(snap, target)
            person = self._same_person(snap, target)
            if person is None:
                person = Person(position, now)
                self.people.append(person)
            if target.timestamp == person.last_stamp:
                continue                                    # the frame this step has already counted
            person.position, person.last_stamp, person.last_seen = position, target.timestamp, now
            person.sightings += 1
            person.confidence = max(person.confidence, target.confidence)
            # One frame is not a person: they have to persist, over confirm_s and in confirm_frames frames.
            person.confirmed = person.confirmed or (person.sightings >= self.confirm_frames and now - person.first_seen >= self.confirm_s)
        # A detection near the threshold drops out for a frame or two, more so at the few frames per
        # second of the Pi: only a longer gap forgets somebody not yet confirmed.
        self.people = [p for p in self.people
                       if p.confirmed or p.attended or p is self.target or now - p.last_seen <= self.confirm_gap_s]

    def _same_person(self, snap: Snapshot, target: HumanTarget) -> Person | None:
        """The known person this detection is, if any. A detection's bearing is good to a degree or
        two; its range, read off the height of a box, can be metres out, and jumps when the box is
        cut by the frame's edge. So the match is narrow across the line of sight (half of
        same_person_m, or 6 deg) and wide along it (same_person_range_m). Matched on map distance
        alone, a person whose range read 2 m differently became a second person, and the boat
        going to the first "lost" them while looking straight at them."""
        best, best_off = None, float("inf")
        for person in self.people:
            distance = haversine_m(snap.gps.lat_deg, snap.gps.lon_deg, *person.position)
            bearing = signed_angle_deg(bearing_deg(snap.gps.lat_deg, snap.gps.lon_deg, *person.position) - snap.imu.yaw_deg)
            off_deg = abs(signed_angle_deg(target.bearing_body_deg - bearing))
            across = target.distance_m * math.sin(math.radians(min(off_deg, 90.0)))
            if (off_deg <= 6.0 or across <= self.same_person_m / 2.0) and abs(target.distance_m - distance) <= self.same_person_range_m \
                    and off_deg < best_off:
                best, best_off = person, off_deg
        return best

    def keep_outs(self) -> list[tuple[float, float]]:
        """People the boat must steer clear of: those already attended, and those seen but not yet
        gone to, so it does not run past them at arm's length on its way to somebody else. The one
        it is going to or holding next to is left out: as an obstacle they would make the autopilot
        stop short of them or back away."""
        return [p.position for p in self.people if (p.attended or p.confirmed) and p is not self.target]

    def attending(self) -> tuple[float, float] | None:
        """The person the boat is going to or holding next to: not an obstacle for now. The
        detector keeps the camera on them (rescue_main passes this to the state as the focus)."""
        return self.target.position if self.target and self.phase in (Phase.APPROACH, Phase.HOLD) else None

    def _attend(self, now: float) -> None:
        self.phase, self._hold_since = Phase.HOLD, now
        self.target.attended_at = now

    def _count_report(self) -> list[Action]:
        """With more than one person about, tell the shore how many there are and how many are done."""
        counts = (sum(p.confirmed or p.attended for p in self.people), sum(p.attended for p in self.people))
        if counts == self._counts or counts[0] < 2:
            return []
        self._counts = counts
        return [Action("report", text=f"PEOPLE {counts[0]} FOUND {counts[1]} ATTENDED")]

    def _ask_mode(self, mode: str, now: float) -> Action:
        self._expected_mode, self._mode_asked_at = mode, now
        return Action("mode", mode=mode)

    def _to_search(self) -> None:
        self.phase, self.target = Phase.SEARCH, None

    def _blocked_waypoint(self, snap: Snapshot, now: float) -> list[Action]:
        vehicle = snap.vehicle
        seq, best, since = self._wp
        if vehicle.wp_dist_m < 0.0 or seq != vehicle.mission_seq:
            self._wp, self._wp_occupied_since = (vehicle.mission_seq, vehicle.wp_dist_m, now), None
            return []
        if vehicle.wp_dist_m < best - 0.5:
            self._wp = (seq, vehicle.wp_dist_m, now)
        # Something right at the waypoint: LiDAR returns closer to it than the clearance the autopilot keeps.
        bearing, distance = vehicle.wp_bearing_deg - snap.imu.yaw_deg, vehicle.wp_dist_m
        occupied = any(math.sqrt(max(0.0, o.distance_m ** 2 + distance ** 2 - 2.0 * o.distance_m * distance
                                     * math.cos(math.radians(o.bearing_body_deg - bearing)))) < self.blocked_clear_m
                       for o in snap.obstacles)
        self._wp_occupied_since = (self._wp_occupied_since or now) if occupied else None
        nearest = min((o.distance_m for o in snap.obstacles), default=float("inf"))
        waited = now - self._wp[2]
        stuck = waited > self.blocked_s and distance <= self.blocked_within_m and nearest <= self.blocked_obstacle_m
        stuck = stuck or waited > self.blocked_far_s        # no closer than the best so far, wherever the waypoint is
        # Inside the waypoint radius the boat has arrived and may be waiting there (a Delay): not stuck.
        stuck = stuck and distance > self.blocked_arrived_m
        if not stuck and not (occupied and now - self._wp_occupied_since >= self.blocked_clear_s):
            return []
        self._wp, self._wp_occupied_since = (-1, 0.0, now), None
        # MISSION_CURRENT.total is the number of the last item (home, item 0, is not counted).
        if vehicle.mission_total and seq >= vehicle.mission_total:
            # The last item: nothing to move on to. Stop here rather than circle; the operator decides.
            return [Action("mode", mode=self.blocked_last_mode), Action("report", text=f"WP {seq} BLOCKED, {self.blocked_last_mode}")]
        return [Action("skip", seq=seq + 1), Action("report", text=f"WP {seq} BLOCKED, SKIPPED")]

    def step(self, snap: Snapshot, now: float) -> list[Action]:
        self._track(snap, now)
        return self._decide(snap, now) + self._count_report()

    def _decide(self, snap: Snapshot, now: float) -> list[Action]:
        vehicle = snap.vehicle

        if self.phase is not Phase.SEARCH or not (vehicle.armed and vehicle.mode == self.search_mode):
            # Holding next to a person is not being blocked: the clocks restart afterwards.
            self._wp, self._wp_occupied_since = (-1, 0.0, now), None

        if self.phase is Phase.SEARCH:
            searching = vehicle.armed and vehicle.mode == self.search_mode
            # Somebody an approach was given up on is only gone to again once seen again.
            waiting = [p for p in self.people if p.confirmed and not p.attended and (p.lost_at is None or p.last_seen > p.lost_at)]
            if not searching or not waiting:
                return self._blocked_waypoint(snap, now) if searching else []
            # Somebody confirmed and not yet gone to: the nearest of them first.
            self.target = min(waiting, key=lambda p: haversine_m(snap.gps.lat_deg, snap.gps.lon_deg, *p.position))
            self.phase, self._last_goto, self._approach_since = Phase.APPROACH, 0.0, now
            self._best_range, self._progress_at = float("inf"), now
            return [self._ask_mode("GUIDED", now),
                    Action("report", lat=self.victim[0], lon=self.victim[1], confidence=self.confidence, text="VICTIM SEEN")]

        # From here on the Pi has asked for a mode. Whoever changes it to something else takes over.
        if not vehicle.armed:
            self._to_search()
            return []
        if vehicle.mode != self._expected_mode:
            if now - self._mode_asked_at > MODE_GRACE_S:
                self._to_search()
            return []

        if self.phase is Phase.HANDBACK:
            self._to_search()
            return []

        if self.phase is Phase.APPROACH:
            # Only detections that fall on this person move their position (_track): somebody else
            # in the frame is somebody else on the map, and does not pull the boat over to them.
            # Out of sight for lost_s since the approach began (the second of two people may not
            # have been in view for a while when the boat turns to them): resume the search rather
            # than hold at a guess. They stay on the map, so the boat keeps clear of where they
            # were, and are gone to again once seen again.
            if now - max(self.target.last_seen, self._approach_since) > self.lost_s:
                self.target.lost_at = now
                self.phase, self.target = Phase.HANDBACK, None
                return [self._ask_mode(self.search_mode, now), Action("report", text="VICTIM LOST")]
            boat = (snap.gps.lat_deg, snap.gps.lon_deg)
            range_m = haversine_m(boat[0], boat[1], self.victim[0], self.victim[1])
            if range_m <= self.standoff_m + self.arrive_m:
                self._attend(now)
                return [self._ask_mode(self.hold_mode, now),
                        Action("report", lat=self.victim[0], lon=self.victim[1], confidence=self.confidence, text="VICTIM REACHED")]
            # The stand-off is measured from the person, not from what they stand on: coming in at
            # an angle, the stand-off point of somebody on a roof can lie against the wall. On the
            # last metres, something solid close ahead means this is as near as the boat should go.
            # And anywhere on the approach, anything on any side within hold_min_m of the LiDAR is
            # too close to the hulls to go on: the autopilot is not told about what stands right at
            # the person (obstacle_feed), so a position estimated inside a wall opens a gap in it.
            ahead = min((o.distance_m for o in snap.obstacles if abs(o.bearing_body_deg) <= 45.0), default=float("inf"))
            around = min((o.distance_m for o in snap.obstacles), default=float("inf"))
            if around < self.hold_min_m or (ahead < self.hold_clear_m and range_m <= self.standoff_m + self.hold_clear_within_m):
                self._attend(now)
                near = range_m <= self.standoff_m + self.hold_clear_within_m
                return [self._ask_mode(self.hold_mode, now),
                        Action("report", lat=self.victim[0], lon=self.victim[1], confidence=self.confidence,
                               text="VICTIM REACHED" if near else "VICTIM UNREACHABLE")]
            if range_m < self._best_range - STALL_PROGRESS_M:
                self._best_range, self._progress_at = range_m, now
            elif now - self._progress_at > self.stall_s:
                # No closer for a while: something is in the way (a person on a roof, behind debris).
                # Stay here and tell the shore where they are.
                self._attend(now)
                return [self._ask_mode(self.hold_mode, now),
                        Action("report", lat=self.victim[0], lon=self.victim[1], confidence=self.confidence, text="VICTIM UNREACHABLE")]
            if now - self._last_goto < self.goto_period_s:
                return []
            self._last_goto = now
            # Stop short of the person: aim standoff_m before them on the line from the boat.
            fraction = (range_m - self.standoff_m) / range_m
            return [Action("goto", lat=boat[0] + (self.victim[0] - boat[0]) * fraction,
                           lon=boat[1] + (self.victim[1] - boat[1]) * fraction)]

        # HOLD
        if now - self._hold_since < self.hold_s:
            return []
        self.phase, self.target = Phase.HANDBACK, None      # from now on they are an obstacle like any other
        return [self._ask_mode(self.after_hold, now), Action("report", text="RESUMING " + self.after_hold)]

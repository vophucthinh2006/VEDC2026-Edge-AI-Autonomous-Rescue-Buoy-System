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

from modules.state_store import Snapshot
from utils.geometry import haversine_m, offset_latlon

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
        self.revisit_m = float(rescue["revisit_radius_m"])
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
        self.blocked_far_s = float(rescue.get("blocked_far_s", 45.0))
        self.blocked_arrived_m = float(rescue.get("blocked_arrived_m", 2.5))
        self.blocked_last_mode = str(rescue.get("blocked_last_mode", "HOLD")).upper()
        self._wp_occupied_since: float | None = None
        self._wp = (-1, 0.0, 0.0)                                 # mission item, best distance to it, when
        self.phase = Phase.SEARCH
        self.victim: tuple[float, float] | None = None            # best estimate of the person's position
        self.confidence = 0.0
        self.reported: list[tuple[float, float]] = []             # people already attended
        self._seen_since: float | None = None
        self._sightings: set[float] = set()                       # detector timestamps since _seen_since
        self._last_located = 0.0
        self._last_seen = 0.0
        self._hold_since = 0.0
        self._last_goto = 0.0
        self._best_range = 0.0
        self._progress_at = 0.0
        self._expected_mode = ""
        self._mode_asked_at = 0.0

    # A detection gives bearing from the bow and range; the boat's fix and heading place it on the map.
    @staticmethod
    def _locate(snap: Snapshot) -> tuple[float, float]:
        target = snap.human_target
        return offset_latlon(snap.gps.lat_deg, snap.gps.lon_deg, snap.imu.yaw_deg + target.bearing_body_deg, target.distance_m)

    def keep_outs(self) -> list[tuple[float, float]]:
        """People the boat must now steer clear of. The one being held next to is left out until
        the boat leaves: as an obstacle it would make the autopilot back away from them."""
        return self.reported[:-1] if self.phase is Phase.HOLD else list(self.reported)

    def attending(self) -> tuple[float, float] | None:
        """The person the boat is going to or holding next to: not an obstacle for now."""
        if self.phase is Phase.APPROACH:
            return self.victim
        return self.reported[-1] if self.phase is Phase.HOLD and self.reported else None

    def _already_reported(self, position: tuple[float, float]) -> bool:
        return any(haversine_m(position[0], position[1], lat, lon) < self.revisit_m for lat, lon in self.reported)

    def _ask_mode(self, mode: str, now: float) -> Action:
        self._expected_mode, self._mode_asked_at = mode, now
        return Action("mode", mode=mode)

    def _to_search(self) -> None:
        self.phase, self.victim, self._seen_since = Phase.SEARCH, None, None

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
        vehicle, target = snap.vehicle, snap.human_target
        fresh = target is not None and now - target.timestamp <= 1.0
        located = self._locate(snap) if fresh and snap.gps.fix >= 3 else None

        if self.phase is not Phase.SEARCH or not (vehicle.armed and vehicle.mode == self.search_mode):
            # Holding next to a person is not being blocked: the clocks restart afterwards.
            self._wp, self._wp_occupied_since = (-1, 0.0, now), None

        if self.phase is Phase.SEARCH:
            searching = vehicle.armed and vehicle.mode == self.search_mode
            if not searching or located is None or self._already_reported(located):
                # A detection near the threshold drops out for a frame or two, more so at the few
                # frames per second of the Pi: a short gap does not restart the confirmation.
                if not searching or located is not None or now - self._last_located > self.confirm_gap_s:
                    self._seen_since = None
                return self._blocked_waypoint(snap, now) if searching else []
            # One frame is not a person: the detection has to persist, over confirm_s and in
            # at least confirm_frames frames.
            if self._seen_since is None:
                self._seen_since, self._sightings = now, set()
            self._last_located = now
            self._sightings.add(target.timestamp)
            if now - self._seen_since < self.confirm_s or len(self._sightings) < self.confirm_frames:
                return []
            self.phase, self.victim, self.confidence = Phase.APPROACH, located, target.confidence
            self._last_seen, self._last_goto = now, 0.0
            self._best_range, self._progress_at = float("inf"), now
            return [self._ask_mode("GUIDED", now),
                    Action("report", lat=located[0], lon=located[1], confidence=target.confidence, text="VICTIM SEEN")]

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
            if located is not None:
                self.victim, self.confidence, self._last_seen = located, max(self.confidence, target.confidence), now
            elif now - self._last_seen > self.lost_s:
                # Lost before reaching it: resume the search rather than hold at a guess.
                self.phase = Phase.HANDBACK
                return [self._ask_mode(self.search_mode, now), Action("report", text="VICTIM LOST")]
            boat = (snap.gps.lat_deg, snap.gps.lon_deg)
            range_m = haversine_m(boat[0], boat[1], self.victim[0], self.victim[1])
            if range_m <= self.standoff_m + self.arrive_m:
                self.phase, self._hold_since = Phase.HOLD, now
                self.reported.append(self.victim)
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
                self.phase, self._hold_since = Phase.HOLD, now
                self.reported.append(self.victim)
                near = range_m <= self.standoff_m + self.hold_clear_within_m
                return [self._ask_mode(self.hold_mode, now),
                        Action("report", lat=self.victim[0], lon=self.victim[1], confidence=self.confidence,
                               text="VICTIM REACHED" if near else "VICTIM UNREACHABLE")]
            if range_m < self._best_range - STALL_PROGRESS_M:
                self._best_range, self._progress_at = range_m, now
            elif now - self._progress_at > self.stall_s:
                # No closer for a while: something is in the way (a person on a roof, behind debris).
                # Stay here and tell the shore where they are.
                self.phase, self._hold_since = Phase.HOLD, now
                self.reported.append(self.victim)
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
        self.phase = Phase.HANDBACK
        return [self._ask_mode(self.after_hold, now), Action("report", text="RESUMING " + self.after_hold)]

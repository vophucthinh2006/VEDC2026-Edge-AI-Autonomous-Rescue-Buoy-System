"""Sailing in formation behind a leader (docs/fleet-control.md): where to steer and how fast.

The shore station puts the boat in a formation with $FOL (leader, right_m, back_m); every beacon of the
leader the F407 hears on the LoRa channel comes as $LDR (position, heading, speed). Beacons come a few
seconds apart, so the leader is reckoned forward from the last one at its speed. The boat's place is
right_m to the right of and back_m behind that point, in the leader's frame; the boat steers to a point
lookahead_m ahead of its place along the leader's heading, so it runs alongside instead of circling the
place, and adds to the leader's speed what it lags behind its place (gain per metre), up to its own
top speed. With the leader still and the boat at its place it stops. No beacon for stale_s: no target,
the boat stands still until the leader is heard again or the station changes the formation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from modules.state_store import LeaderState, Snapshot
from utils.geometry import bearing_deg, haversine_m, offset_latlon


@dataclass(frozen=True)
class FormationGoal:
    point: tuple[float, float]   # where to steer: ahead of the boat's place
    speed_mps: float
    place: tuple[float, float]   # the boat's place in the formation
    error_m: float               # how far the boat is from it


class FormationKeeper:
    """The side a follower keeps is that of the leader's lane, not of its bow at each moment. The leader's search
    route is a lawnmower: at the end of each lane it turns about, and its right becomes the other side of the
    water. Kept on the leader's right, the follower then had to change sides: around the outside of the turn, or
    across the leader's way (simulation 2026-11-12, wide_empty: followers 6.5 m off their place, two boats within
    3 m for 112 s). So right_m is measured square to the lane (the heading the leader settles on for settle_s),
    and when the leader settles on a lane the other way about (more than REVERSAL_DEG off it) the side flips
    with it: the follower stays on the same side of the water and the formation moves across as a whole.
    The leader turns about in two right angles with a short leg across between the lanes; a heading square to
    the lane becomes the lane only once the leader has held it longer than it held the lane (the leg from the
    start to the first lane, or a real bend of the route), so the short leg across changes nothing."""
    REVERSAL_DEG = 150.0   # a lawnmower's turn about is 180; the way from the start to the first lane wanders up to 135
    SETTLE_TOLERANCE_DEG = 20.0

    def __init__(self, config: dict, max_speed_mps: float) -> None:
        self.lookahead_m = float(config.get("lookahead_m", 6.0))
        self.gain = float(config.get("speed_gain_per_m", 0.08))
        self.stale_s = float(config.get("leader_stale_s", 15.0))
        self.stop_radius_m = float(config.get("stop_radius_m", 2.5))
        self.settle_s = float(config.get("settle_s", 5.0))
        # "ned": the station sends the place as metres north and east of the leader, fixed on the water (ArduPilot
        # AP_Follow FOLL_OFS_TYPE 0; the Worker's plan with offsets "ned", src/fleetplan.js nedOffsets); "bow"
        # (default): right_m and back_m of the leader's lane, as above
        self.ned = str(config.get("offsets", "bow")) == "ned"
        self.max_speed = max_speed_mps
        self._key: tuple | None = None       # (leader, right, back) the lane below belongs to
        self._lane: float | None = None      # the heading of the leader's lane, None till it settles on one
        self._lane_held = 0.0                # how long the leader has held its lane, s
        self._side = 1.0                     # -1 once the leader runs its lanes the other way about
        self._seen: tuple[float, float] | None = None   # (heading, since) the leader holds now

    def _lane_heading(self, leader: LeaderState, key: tuple, now: float) -> float:
        if key != self._key:
            self._key, self._lane, self._lane_held, self._side, self._seen = key, None, 0.0, 1.0, None
        heading = leader.heading_deg
        if leader.speed_mps < 0.1:
            self._seen = None
        elif self._seen is None or abs((heading - self._seen[0] + 180.0) % 360.0 - 180.0) > self.SETTLE_TOLERANCE_DEG:
            self._seen = (heading, now)
        elif (held := now - self._seen[1]) >= self.settle_s:
            turn = abs((heading - self._lane + 180.0) % 360.0 - 180.0) if self._lane is not None else 0.0
            # The lane stays the heading first settled on: followed as the heading creeps, a slow turn about
            # (east, south, west over half a minute) never looked like one.
            if self._lane is None:
                self._lane, self._lane_held = self._seen[0], held
            elif turn <= self.SETTLE_TOLERANCE_DEG:   # on the lane
                self._lane_held = max(self._lane_held, held)
            elif turn > self.REVERSAL_DEG:            # the next lane, the other way about
                self._lane, self._lane_held, self._side = self._seen[0], held, -self._side
            elif held > self._lane_held:              # off the lane, and held longer than it: the lane now
                self._lane, self._lane_held = self._seen[0], held
        return heading if self._lane is None else self._lane

    def goal(self, snap: Snapshot, leader: LeaderState | None, right_m: float, back_m: float, now: float) -> FormationGoal | None:
        if leader is None or now - leader.timestamp > self.stale_s or leader.heading_deg < 0.0:
            return None
        speed = max(leader.speed_mps, 0.0)
        heading = leader.heading_deg
        ahead = offset_latlon(leader.lat, leader.lon, heading, speed * (now - leader.timestamp))
        if self.ned:   # right_m, back_m carry metres north, east
            place = offset_latlon(*offset_latlon(*ahead, 0.0, right_m), 90.0, back_m)
        else:
            lane = self._lane_heading(leader, (leader.buoy, right_m, back_m), now)
            behind = offset_latlon(*ahead, (heading + 180.0) % 360.0, back_m) if back_m else ahead
            place = offset_latlon(*behind, (lane + 90.0) % 360.0, self._side * right_m) if right_m else behind
        here = (snap.gps.lat_deg, snap.gps.lon_deg)
        error = haversine_m(*here, *place)
        # Along the leader's heading: positive when the boat is behind its place
        along = error * math.cos(math.radians(bearing_deg(*here, *place) - heading)) if error > 0.01 else 0.0
        if speed < 0.1 and error <= self.stop_radius_m:
            return FormationGoal(place, 0.0, place, error)
        point = offset_latlon(*place, heading, self.lookahead_m) if speed >= 0.1 else place
        return FormationGoal(point, min(max(speed + self.gain * along, 0.0), self.max_speed), place, error)

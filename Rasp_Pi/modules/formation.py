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
    def __init__(self, config: dict, max_speed_mps: float) -> None:
        self.lookahead_m = float(config.get("lookahead_m", 6.0))
        self.gain = float(config.get("speed_gain_per_m", 0.08))
        self.stale_s = float(config.get("leader_stale_s", 15.0))
        self.stop_radius_m = float(config.get("stop_radius_m", 2.5))
        self.max_speed = max_speed_mps

    def goal(self, snap: Snapshot, leader: LeaderState | None, right_m: float, back_m: float, now: float) -> FormationGoal | None:
        if leader is None or now - leader.timestamp > self.stale_s or leader.heading_deg < 0.0:
            return None
        speed = max(leader.speed_mps, 0.0)
        heading = leader.heading_deg
        ahead = offset_latlon(leader.lat, leader.lon, heading, speed * (now - leader.timestamp))
        behind = offset_latlon(*ahead, (heading + 180.0) % 360.0, back_m) if back_m else ahead
        place = offset_latlon(*behind, (heading + 90.0) % 360.0, right_m) if right_m else behind
        here = (snap.gps.lat_deg, snap.gps.lon_deg)
        error = haversine_m(*here, *place)
        # Along the leader's heading: positive when the boat is behind its place
        along = error * math.cos(math.radians(bearing_deg(*here, *place) - heading)) if error > 0.01 else 0.0
        if speed < 0.1 and error <= self.stop_radius_m:
            return FormationGoal(place, 0.0, place, error)
        point = offset_latlon(*place, heading, self.lookahead_m) if speed >= 0.1 else place
        return FormationGoal(point, min(max(speed + self.gain * along, 0.0), self.max_speed), place, error)

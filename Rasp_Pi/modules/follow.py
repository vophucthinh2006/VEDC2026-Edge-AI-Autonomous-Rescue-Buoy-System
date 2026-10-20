"""Search the route, follow a person, back to the route: the STM32 boat's answer to the Pixhawk
boat's AUTO / GUIDED / LOITER (modules/rescue.py).

SEARCH  the navigator goes to the route's waypoints.
FOLLOW  somebody was seen in confirm_frames frames within confirm_s: the navigator goes to them
        along the camera's bearing (BapfNavigator's human_target branch: turn the bow, close in,
        hold at the stand-off). No GPS position of the person is used: in a pool the GPS wanders
        2 to 3 m, the camera's bearing and the compass do not.
        The detector looks about once every 0.8 s; between looks the last sighting is kept, as a
        compass bearing so it stays right while the boat turns, for up to lost_s.
SEARCH  again after lost_s without a sighting.

With follow.only_on_route (the STM32 boat with no RC transmitter) the boat moves only while it
has a route from the dashboard: with none, plan() stands by and nobody is followed.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace

from modules.state_store import HumanTarget, Snapshot
from utils.geometry import offset_latlon, signed_angle_deg

SEARCH, FOLLOW = "SEARCH", "FOLLOW"


@dataclass(frozen=True)
class Event:
    kind: str                 # "found" | "lost"
    lat: float | None = None  # found: where the person is reckoned to be (boat GPS + bearing + range);
    lon: float | None = None  # None without a GPS fix
    confidence: float = 0.0
    risk: int = 1             # found: the person's risk level (modules/edge_model.py)


class Follow:
    def __init__(self, follow: dict) -> None:
        self.confirm_frames = int(follow.get("confirm_frames", 3))
        self.confirm_s = float(follow.get("confirm_s", 2.5))
        self.lost_s = float(follow.get("lost_s", 4.0))
        self.phase = SEARCH
        self._sightings: deque[float] = deque()
        self._last: HumanTarget | None = None
        self._last_world_deg = 0.0          # compass bearing of the last sighting
        self._last_at = 0.0

    def _sighting(self, snap: Snapshot) -> HumanTarget | None:
        """The detector's target, if it is one this object has not counted yet."""
        target = snap.human_target
        if target is None or (self._last is not None and target.timestamp <= self._last.timestamp):
            return None
        return target

    def step(self, snap: Snapshot, now: float) -> tuple[Snapshot, list[Event]]:
        """The snapshot for the navigator (its human_target set while following, cleared while
        searching) and what happened this step."""
        events: list[Event] = []
        target = self._sighting(snap)
        if target is not None:
            self._last, self._last_at = target, now
            self._last_world_deg = (snap.imu.yaw_deg + target.bearing_body_deg) % 360.0
            self._sightings.append(now)
        while self._sightings and now - self._sightings[0] > self.confirm_s:
            self._sightings.popleft()
        if self.phase == SEARCH and len(self._sightings) >= self.confirm_frames and self._last is not None:
            self.phase = FOLLOW
            lat = lon = None
            if snap.gps.fix >= 2:
                lat, lon = offset_latlon(snap.gps.lat_deg, snap.gps.lon_deg, self._last_world_deg, self._last.distance_m)
            events.append(Event("found", lat, lon, self._last.confidence, self._last.risk))
        elif self.phase == FOLLOW and now - self._last_at > self.lost_s:
            self.phase = SEARCH
            self._sightings.clear()
            events.append(Event("lost"))
        if self.phase == SEARCH or self._last is None:
            return replace(snap, human_target=None), events
        held = HumanTarget(signed_angle_deg(self._last_world_deg - snap.imu.yaw_deg), self._last.distance_m,
                           self._last.confidence, self._last.timestamp, self._last.risk)
        return replace(snap, human_target=held), events

    @property
    def target(self) -> HumanTarget | None:
        """The last sighting of the person followed; None while searching."""
        return self._last if self.phase == FOLLOW else None

    @property
    def world_deg(self) -> float:
        """Compass bearing of the last sighting."""
        return self._last_world_deg

    def position(self, snap: Snapshot) -> tuple[float, float] | None:
        """Where the person last seen is reckoned to be (boat GPS + compass bearing + range); None
        without a sighting or a GPS fix."""
        if self._last is None or snap.gps.fix < 2:
            return None
        return offset_latlon(snap.gps.lat_deg, snap.gps.lon_deg, self._last_world_deg, self._last.distance_m)

    def plan(self, snap: Snapshot, waypoint: tuple[float, float] | None, now: float,
             only_on_route: bool) -> tuple[Snapshot, list[Event]]:
        """The snapshot for the navigator and what happened: step() toward the route's waypoint,
        or stand_by() when only_on_route holds and there is no waypoint to go to."""
        if only_on_route and waypoint is None:
            return self.stand_by(snap), []
        return self.step(replace(snap, waypoint=waypoint), now)

    def stand_by(self, snap: Snapshot) -> Snapshot:
        """No route to run (follow.only_on_route): forget any sighting and give the navigator
        neither a waypoint nor a person, so it stops (no_goal)."""
        self.phase = SEARCH
        self._sightings.clear()
        self._last = None
        return replace(snap, waypoint=None, human_target=None)

    def describe(self) -> str:
        return self.phase if self.phase == SEARCH else f"FOLLOW {self._last.distance_m:.1f} m" if self._last else FOLLOW

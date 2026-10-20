"""What the STM32 boat is doing (main.py): the mission state machine above the navigator.

IDLE       no route to run (none sent yet, an empty one sent, or the last waypoint reached):
           stand still. With follow.only_on_route false (the bench) people are followed anyway.
PATROL     the route's waypoints, the camera sweeping for people, BAPF avoiding obstacles.
FOLLOW     the person with the highest risk in view, nearest first (modules/follow.py: confirmed
           in a few frames, gone to along the camera's bearing, held at the stand-off). Back to
           PATROL when the person presses the confirm button on the buoy (they took the supplies:
           that person is ignored for follow.ignore_s) or after follow.lost_s without a sighting.
RETURN     from PATROL or FOLLOW when the F407 says the shore station has not been heard for a
           while ($LNK shore_ok 0): straight home, following nobody, still reporting people.
           Home is the shore station's GPS position ($LNK), brought return.fence_margin_m inside
           the geofence when the station stands outside it; with no home ever heard, where the
           boat was when the current route was accepted.
HOME_HOLD  within return.arrive_radius_m of home: stand still.
           When the shore link comes back, RETURN and HOME_HOLD go back to PATROL at the waypoint
           the boat was going to (the Route is kept), or to IDLE without one.

Nobody is followed while on a boat (risk SAFE). Reports to the shore ($TXD VICTIM_FOUND, Report):
everybody the camera confirms, once each (Reporter: SAFE people too, the same person not twice
within report_dedup_m); while following, a change of the person's risk, and every
follow.report_every_s their reckoned position if it moved more than report_moved_m.

The safety arbiter (modules/safety.py) stays on top of whatever this asks for.
"""
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field, replace

from modules.follow import FOLLOW as FOLLOWING, Follow
from modules.state_store import RISK_NAMES, SAFE, HumanTarget, Snapshot
from modules.waypoints import Route
from utils.geometry import haversine_m, offset_latlon, signed_angle_deg

IDLE, PATROL, FOLLOW, RETURN, HOME_HOLD = "IDLE", "PATROL", "FOLLOW", "RETURN", "HOME_HOLD"


@dataclass(frozen=True)
class Report:
    """One $TXD,seq,VICTIM_FOUND,lat,lon,conf,risk to the shore."""
    lat: float               # 0, 0 without a GPS fix
    lon: float
    confidence: float
    risk: int
    why: str                 # "found" | "risk" (the followed person's changed) | "moved" (the periodic one)


@dataclass(frozen=True)
class Step:
    planned: Snapshot        # for the navigator: its waypoint and human_target set by the state
    reports: list[Report]
    supplies_taken: bool     # the confirm button was pressed since the last step


@dataclass
class _Track:
    """Somebody the Reporter is seeing, by compass bearing and range."""
    world_deg: float
    distance_m: float
    times: deque = field(default_factory=deque)
    confidence: float = 0.0
    risk: int = 1
    reported: bool = False


def _reckon(snap: Snapshot, world_deg: float, distance_m: float) -> tuple[float, float] | None:
    if snap.gps.fix < 2:
        return None
    return offset_latlon(snap.gps.lat_deg, snap.gps.lon_deg, world_deg, distance_m)


def _same(snap: Snapshot, position: tuple[float, float] | None, world_deg: float,
          other_position: tuple[float, float] | None, other_world_deg: float, radius_m: float, gate_deg: float) -> bool:
    """Same person: within radius_m by reckoned position when both have one, else by compass bearing."""
    if position is not None and other_position is not None:
        return haversine_m(*position, *other_position) <= radius_m
    return abs(signed_angle_deg(world_deg - other_world_deg)) <= gate_deg


class Reporter:
    """Everybody the camera sees, followed or not, is reported once: seen in confirm_frames
    detector passes within confirm_s (as Follow confirms), and not within dedup_m of somebody
    already reported. Without a GPS fix people are told apart by compass bearing only, and such a
    report is forgotten after forget_s (the boat may have moved since)."""

    def __init__(self, confirm_frames: int, confirm_s: float, dedup_m: float = 5.0, gate_deg: float = 10.0,
                 forget_s: float = 60.0) -> None:
        self.confirm_frames, self.confirm_s = confirm_frames, confirm_s
        self.dedup_m, self.gate_deg, self.forget_s = dedup_m, gate_deg, forget_s
        self._tracks: list[_Track] = []
        self._reported: list[tuple[tuple[float, float] | None, float, float]] = []   # position, compass bearing, when
        self._frame = float("-inf")   # timestamp of the last detector pass taken

    def observe(self, snap: Snapshot, now: float) -> list[Report]:
        fresh = [t for t in snap.human_targets if t.timestamp > self._frame]
        if fresh:
            self._frame = max(t.timestamp for t in fresh)
        for target in fresh:
            world = (snap.imu.yaw_deg + target.bearing_body_deg) % 360.0
            track = next((k for k in self._tracks if abs(signed_angle_deg(world - k.world_deg)) <= self.gate_deg
                          and abs(target.distance_m - k.distance_m) <= max(2.0, 0.35 * target.distance_m)), None)
            if track is None:
                track = _Track(world, target.distance_m)
                self._tracks.append(track)
            track.world_deg, track.distance_m = world, target.distance_m
            track.confidence, track.risk = target.confidence, target.risk
            track.times.append(now)
        for track in self._tracks:
            while track.times and now - track.times[0] > self.confirm_s:
                track.times.popleft()
        self._tracks = [k for k in self._tracks if k.times]
        self._reported = [r for r in self._reported if r[0] is not None or now - r[2] <= self.forget_s]
        reports = []
        for track in self._tracks:
            if track.reported or len(track.times) < self.confirm_frames:
                continue
            track.reported = True
            position = _reckon(snap, track.world_deg, track.distance_m)
            if any(_same(snap, position, track.world_deg, p, w, self.dedup_m, self.gate_deg) for p, w, _ in self._reported):
                continue
            self._reported.append((position, track.world_deg, now))
            lat, lon = position if position is not None else (0.0, 0.0)
            reports.append(Report(lat, lon, track.confidence, track.risk, "found"))
        return reports


class Mission:
    def __init__(self, config: dict, route: Route, only_on_route: bool) -> None:
        follow, back = config.get("follow", {}), config.get("return", {}) or {}
        self.follow = Follow(follow)
        self.reporter = Reporter(self.follow.confirm_frames, self.follow.confirm_s, float(follow.get("report_dedup_m", 5.0)))
        self.route = route
        self.only_on_route = only_on_route
        self.report_every_s = float(follow.get("report_every_s", 20.0))
        self.report_moved_m = float(follow.get("report_moved_m", 3.0))
        self.risk_frames = int(follow.get("risk_frames", 2))
        self.ignore_s = float(follow.get("ignore_s", 60.0))
        self.ignore_m = float(follow.get("ignore_radius_m", 5.0))
        self.arrive_m = float(back.get("arrive_radius_m", 3.0))
        self.margin_m = float(back.get("fence_margin_m", 2.0))
        self.log = logging.getLogger(__name__)
        self.state = IDLE
        self.home: tuple[float, float] | None = None          # the shore station, from $LNK
        self.route_start: tuple[float, float] | None = None   # the boat when the route was accepted
        self._start_pending = True
        self._presses = -1
        self._ignored: list[tuple[float, tuple[float, float] | None, float]] = []   # until, position, compass bearing
        self._risk = 0                     # the followed person's risk last reported
        self._risk_new, self._risk_count, self._risk_seen = 0, 0, 0.0
        self._report_at = 0.0
        self._report_pos: tuple[float, float] | None = None

    # -- route and home ---------------------------------------------------------------------
    def set_route(self, route: Route) -> None:
        """A new route from the dashboard. Where the boat is at the next GPS fix becomes the way
        home when no shore station position was ever heard."""
        self.route = route
        self._start_pending = True

    def home_goal(self) -> tuple[float, float] | None:
        home = self.home or self.route_start
        return None if home is None else self.route.clamp_inside_fence(*home, self.margin_m)

    @property
    def sweeping(self) -> bool:
        """Whether the camera should sweep for people: not while standing still for want of a route."""
        return not (self.state == IDLE and self.only_on_route)

    @property
    def detecting(self) -> bool:
        """Whether the person detector should run: not while standing still for want of a route
        (nobody is followed or reported then, and the Pi stays cool). On again with the route."""
        return self.sweeping

    def _base(self) -> str:
        return PATROL if self.route.current is not None else IDLE

    def _enter(self, state: str, why: str) -> None:
        if state != self.state:
            self.log.warning("%s -> %s: %s", self.state, state, why)
            self.state = state

    # -- the step -----------------------------------------------------------------------------
    def step(self, snap: Snapshot, now: float) -> Step:
        fix = snap.gps.fix >= 2
        reports = self.reporter.observe(snap, now)
        if snap.link.home is not None:
            self.home = snap.link.home
        if self._start_pending and fix and self.route.waypoints:
            self.route_start, self._start_pending = (snap.gps.lat_deg, snap.gps.lon_deg), False
        supplies = self._pressed(snap.system.confirm_presses)
        self._ignored = [i for i in self._ignored if i[0] > now]

        if self.state in (PATROL, FOLLOW) and not snap.link.shore_ok:
            self.follow.stand_by(snap)
            heard = f"{snap.link.heard_ago_s:.0f} s ago" if snap.link.heard_ago_s >= 0 else "never"
            self._enter(RETURN, f"shore station not heard ({heard}): going home")
        elif self.state in (RETURN, HOME_HOLD) and snap.link.shore_ok:
            self._enter(self._base(), f"shore link back: {self.route.describe()}")
        if self.state in (RETURN, HOME_HOLD):
            goal = self.home_goal()
            if self.state == RETURN and goal is not None and fix and haversine_m(snap.gps.lat_deg, snap.gps.lon_deg, *goal) <= self.arrive_m:
                self._enter(HOME_HOLD, "home reached: holding")
            planned = replace(snap, waypoint=goal if self.state == RETURN else None, human_target=None)
            return Step(planned, reports, supplies)

        if fix and self.route.update(snap.gps.lat_deg, snap.gps.lon_deg):
            self.log.info("waypoint reached, now %s", self.route.describe())
        if supplies and self.state == FOLLOW:
            self._ignored.append((now + self.ignore_s, self.follow.position(snap), self.follow.world_deg))
            self.follow.stand_by(snap)
            self._enter(self._base(), f"supplies taken: that person is left alone for {self.ignore_s:.0f} s")
        waypoint = self.route.current
        if self.only_on_route and waypoint is None:
            self._enter(IDLE, self.route.describe())
            return Step(self.follow.stand_by(snap), reports, supplies)
        planned, events = self.follow.step(replace(snap, waypoint=waypoint, human_target=self._pick(snap)), now)
        for event in events:
            if event.kind == "found":
                self._risk = self._risk_new = event.risk
                self._risk_count, self._risk_seen = 0, self.follow.target.timestamp
                self._report_at, self._report_pos = now, self.follow.position(snap)
        if self.follow.phase == FOLLOWING:
            self._enter(FOLLOW, f"following a {RISK_NAMES[self._risk]} person")
        else:
            self._enter(self._base(), ("person lost: " if self.state == FOLLOW else "") + self.route.describe())
        if self.state == FOLLOW:
            reports += self._follow_reports(snap, now)
        return Step(planned, reports, supplies)

    def _pressed(self, presses: int) -> bool:
        """The confirm button count rose. The first report (or one lower than before: the F407
        restarted) is only the starting count."""
        if presses < 0:
            return False
        rose = presses > self._presses >= 0
        self._presses = presses
        return rose

    def _ignored_target(self, snap: Snapshot, target: HumanTarget) -> bool:
        world = (snap.imu.yaw_deg + target.bearing_body_deg) % 360.0
        position = _reckon(snap, world, target.distance_m)
        return any(_same(snap, position, world, p, w, self.ignore_m, 15.0) for _, p, w in self._ignored)

    def _pick(self, snap: Snapshot) -> HumanTarget | None:
        """Who to follow: never somebody on a boat or left alone after taking the supplies; the
        highest risk, then the one followed already (by compass bearing), else the nearest."""
        candidates = [t for t in snap.human_targets if t.risk != SAFE and not self._ignored_target(snap, t)]
        if not candidates:
            return None
        if self.follow.target is not None:
            followed = self.follow.world_deg
            return min(candidates, key=lambda t: (-t.risk, abs(signed_angle_deg(snap.imu.yaw_deg + t.bearing_body_deg - followed))))
        return min(candidates, key=lambda t: (-t.risk, t.distance_m))

    def _follow_reports(self, snap: Snapshot, now: float) -> list[Report]:
        """While following: the person's new risk once seen in risk_frames sightings in a row, and
        every report_every_s their position if it moved more than report_moved_m. A higher risk
        goes out at once; a lower one waits for report_every_s since the last report, so a pose
        that flickers between two levels (legs half under water) is not a report every second."""
        target, position = self.follow.target, self.follow.position(snap)
        lat, lon = position if position is not None else (0.0, 0.0)
        if target is not None and target.timestamp > self._risk_seen:
            self._risk_seen = target.timestamp
            if target.risk == self._risk:
                self._risk_count = 0
            else:
                self._risk_count = self._risk_count + 1 if target.risk == self._risk_new else 1
                self._risk_new = target.risk
                due = target.risk > self._risk or now - self._report_at >= self.report_every_s
                if self._risk_count >= self.risk_frames and due:
                    self.log.warning("followed person now %s (was %s)", RISK_NAMES[target.risk], RISK_NAMES[self._risk])
                    self._risk, self._risk_count = target.risk, 0
                    self._report_at, self._report_pos = now, position
                    return [Report(lat, lon, target.confidence, target.risk, "risk")]
        if target is None or now - self._report_at < self.report_every_s:
            return []
        self._report_at = now
        if position is None or (self._report_pos is not None and haversine_m(*position, *self._report_pos) <= self.report_moved_m):
            return []
        self._report_pos = position
        return [Report(lat, lon, target.confidence, self._risk, "moved")]

    # -- words for the log and the viewer ---------------------------------------------------
    def describe(self) -> str:
        if self.state == FOLLOW and self.follow.target is not None:
            target = self.follow.target
            return f"FOLLOW {RISK_NAMES[target.risk]} {target.distance_m:.1f} m"
        return f"{self.state} {self.route.describe()}"

    def lines(self, snap: Snapshot) -> list[str]:
        """The viewer's first two status lines: what the boat is doing and why."""
        if self.state == IDLE:
            return [f"IDLE: {self.route.describe()}", "send a route from the dashboard to start (detector off)" if self.only_on_route
                    else "no route: following anybody in view"]
        if self.state == PATROL:
            return [f"PATROL: {self.route.describe()}", "the camera sweeps for people"]
        if self.state == FOLLOW:
            target = self.follow.target
            what = f"{RISK_NAMES[target.risk]} person {target.distance_m:.1f} m away" if target else "person"
            return [f"FOLLOW: {what}", "going to them along the camera's bearing"]
        heard = f"{snap.link.heard_ago_s:.0f} s ago" if snap.link.heard_ago_s >= 0 else "never"
        goal = self.home_goal()
        if self.state == RETURN:
            where = (f"{haversine_m(snap.gps.lat_deg, snap.gps.lon_deg, *goal):.0f} m to home" if goal and snap.gps.fix >= 2
                     else "home unknown: standing still" if goal is None else "home known, no GPS fix")
            return [f"RETURN: shore heard {heard}", where]
        return [f"HOME_HOLD: shore heard {heard}", "holding at home until the shore link is back"]

"""Waypoint guidance with obstacle avoidance computed on the Pi.

BapfNavigator is the starboard-biased artificial potential field of the STM32 runtime (main.py).

PaperBapfNavigator and VoNavigator follow Jo, Kim, Kim and Park, "Comparison of Velocity Obstacle
and Artificial Potential Field Methods for Collision Avoidance in Swarm Operation of Unmanned
Surface Vehicles", J. Mar. Sci. Eng. 2022, 10, 2036 (papers/jmse-10-02036-v2.pdf): the biased
potential field of section 3.2, the velocity obstacle of section 3.3 and the closest-point-of-
approach gate of section 3.4. The paper avoids other boats whose position and velocity are known;
here the obstacles are LiDAR returns and do not move, so their velocity is zero. Both are
experiments (pi_steer.py, sim/tools/avoid_compare.sh): the Pixhawk boat avoids with the
autopilot's own BendyRuler.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, log, radians, sin
from time import monotonic

from modules.perception import Obstacle
from modules.state_store import Snapshot
from utils.geometry import bearing_deg, body_to_world, clamp, haversine_m, signed_angle_deg, world_to_heading


@dataclass(frozen=True)
class NavCommand:
    mode: str
    speed_mps: float
    heading_deg: float
    reason: str


CREEP = 0.3   # share of max_speed_mps kept near an obstacle the commanded course leads away from
FACE_INSIDE_M = 0.3   # this far inside the stand-off the boat no longer creeps to turn its bow to the person
# Hysteresis, so that a reading jittering round a threshold does not flip the command every 100 ms
# (each flip from a speed to STOP used to brake with the front motor):
STOP_RELEASE_M = 0.20       # stopped for an obstacle: going on once it is this much past the stop distance
STANDOFF_RELEASE_M = 0.50   # holding at the stand-off: closing in again only this much beyond it (a box range is +-30 %)
STANDOFF_ARRIVE_M = 0.15    # within stand-off + this the boat stops next to the person
COURSE_MIN_MPS = 0.3        # the GPS course is the boat's way over the ground only above this speed
# Bow blocked, the boat still swings away at a creep (the F407 steers only with some throttle) when
# the turn is at least this big and every obstacle on its way lies this far off the bow on the other side.
TURN_AWAY_MIN_DEG = 30.0
TURN_AWAY_OFF_BOW_DEG = 5.0
TURN_AWAY_ABEAM_DEG = 30.0  # this far off the bow an obstacle may already be inside the inflated hull
ESCAPE_STEP_DEG = 15.0      # stopped by an obstacle, the courses tried for a way out, this far apart
# No way out ahead: the boat backs off for BACK_S with the front motor alone (NAV BACK, App/control.c),
# BACK_THROTTLE its throttle astern, while the stern stays clear within the stop distance.
BACK_THROTTLE = 0.30
BACK_S = 3.0


class BapfNavigator:
    def __init__(self, vehicle: dict, planner: dict) -> None:
        self.v = vehicle
        self.p = planner
        self._blocked = False      # the last plan stopped for an obstacle on the course
        self._at_standoff = False  # the last plan held next to the person
        self._bow_blocked = False  # the last plan stopped for an obstacle on the way the boat moves
        self._coasting = False     # the last plan cut the throttle for the boat running too fast
        self._back_until = 0.0     # backing off an obstacle until then (monotonic s)

    @property
    def standoff_m(self) -> float:
        """The range to a person at which the boat holds: the water kept off the bow (person_gap_m;
        target_standoff_m where it is not set) plus the bow ahead of the LiDAR and camera that
        measure that range (bow_ahead_m)."""
        gap = self.v.get("person_gap_m", self.v["target_standoff_m"])
        return float(gap) + float(self.v.get("bow_ahead_m", 0.0))

    def braking_m(self, snapshot: Snapshot) -> float:
        """How far the boat still goes once the Pi cuts the throttle, at the speed the GPS measures:
        a NAV round trip (reaction_s), then the coast. Water drag at these speeds grows about with
        the speed, so the boat coasts about v * coast_s. The F407 turns the speed into a throttle
        open-loop, so the boat may run several times the speed it was given."""
        v = max(float(snapshot.gps.speed_mps), 0.0)
        return v * (float(self.v.get("reaction_s", 0.0)) + float(self.v.get("coast_s", 0.0)))

    def _governed(self, speed: float, gps_speed: float) -> bool:
        """True when the throttle goes off for the boat running faster than it was told: above the
        speed plus overspeed_mps, back on below the speed plus half of it (0 or unset: off). The margin
        shrinks with the speed below max_speed_mps: creeping off an obstacle at 0.09 m/s, the simulated
        boat ran 0.4 m/s within the full margin and coasted 3 m on into the wrecked boat (2026-10-23)."""
        over = float(self.v.get("overspeed_mps", 0.0)) * min(1.0, speed / max(float(self.v["max_speed_mps"]), 1e-6))
        if over <= 0.0 or speed <= 0.0:
            self._coasting = False
            return False
        self._coasting = gps_speed > speed + (over / 2.0 if self._coasting else over)
        return self._coasting

    @staticmethod
    def _ways_ahead(snapshot: Snapshot) -> tuple[float, ...]:
        """Body bearings the boat moves along whatever course it is given: its bow, and its course
        over the ground once it is fast enough for the GPS course to mean something."""
        gps = snapshot.gps
        if gps.speed_mps >= COURSE_MIN_MPS:
            return 0.0, signed_angle_deg(gps.course_deg - snapshot.imu.yaw_deg)
        return (0.0,)

    def _course_blocked(self, obstacles: tuple[Obstacle, ...], heading_body_deg: float, limit_m: float) -> bool:
        """True when an obstacle whose clearance is inside limit_m lies on the course: held, the
        course would bring the boat's inflated hull onto it (closest point of approach, as in Jo et
        al. 2022, eq. 16)."""
        return bool(self._blocking(obstacles, heading_body_deg, limit_m))

    def _blocking(self, obstacles: tuple[Obstacle, ...], heading_body_deg: float, limit_m: float) -> list[Obstacle]:
        """The obstacles _course_blocked finds on the course."""
        inflated_radius = float(self.v["boat_radius_m"]) + float(self.v["safety_margin_m"])
        v_fwd, v_right = cos(radians(heading_body_deg)), sin(radians(heading_body_deg))
        out = []
        for obstacle in obstacles:
            if obstacle.distance_m - inflated_radius - obstacle.width_m / 2 > limit_m:
                continue
            angle = radians(obstacle.bearing_body_deg)
            tcpa, dcpa = cpa(obstacle.distance_m * cos(angle), obstacle.distance_m * sin(angle), v_fwd, v_right)
            if tcpa >= 0.0 and dcpa <= inflated_radius + obstacle.width_m / 2:
                out.append(obstacle)
        return out

    def _can_turn_away(self, blocking: list[Obstacle], turn_deg: float) -> bool:
        """True when the boat, its bow blocked, may swing away at a creep: a big enough turn, and every
        obstacle on its way off the bow on the side the bow leaves, clear of the inflated hull unless it
        lies abeam. Seen in the central scene (2026-10-23): the end of the wrecked boat 1.2 m ahead to
        starboard (or its side 0.4 m off the starboard hull), the next waypoint 50 to 150 deg to port,
        and the boat stopped there for good."""
        if abs(turn_deg) < TURN_AWAY_MIN_DEG:
            return False
        inflated_radius = float(self.v["boat_radius_m"]) + float(self.v["safety_margin_m"])
        return all(o.bearing_body_deg * turn_deg < 0.0 and abs(o.bearing_body_deg) >= TURN_AWAY_OFF_BOW_DEG
                   and (o.distance_m - inflated_radius - o.width_m / 2 > 0.0 or abs(o.bearing_body_deg) >= TURN_AWAY_ABEAM_DEG)
                   for o in blocking)

    def _escape_turn(self, obstacles: tuple[Obstacle, ...], blocking: list[Obstacle], wanted_deg: float, limit_m: float) -> float | None:
        """The turn from the bow, nearest to wanted_deg in steps of ESCAPE_STEP_DEG, whose course is clear
        within limit_m and which the bow can take (free, or swinging away from what blocks it); None if
        there is none: boxed in, the boat stays stopped."""
        turns = [wanted_deg] + [signed_angle_deg(wanted_deg + side * ESCAPE_STEP_DEG * k)
                                for k in range(1, int(180 // ESCAPE_STEP_DEG) + 1) for side in (1, -1)]
        for turn in turns:
            if self._course_blocked(obstacles, turn, limit_m):
                continue
            if not blocking or self._can_turn_away(blocking, turn):
                return turn
        return None

    def _repulsion_body(self, obstacles: tuple[Obstacle, ...]) -> tuple[float, float, float]:
        """Return forward/right forces and nearest clearance after inflating the hull."""
        forward = right = 0.0
        nearest = float("inf")
        inflated_radius = float(self.v["boat_radius_m"]) + float(self.v["safety_margin_m"])
        d0 = float(self.v["influence_distance_m"])
        for obstacle in obstacles:
            clearance = obstacle.distance_m - inflated_radius - obstacle.width_m / 2
            nearest = min(nearest, clearance)
            if clearance <= 0.01:
                # Strong force directly away from an occupied safety hull.
                magnitude = 30.0
            elif clearance >= d0:
                continue
            else:
                magnitude = float(self.p["repulsive_gain"]) * (1 / clearance - 1 / d0) / (clearance * clearance)
            angle = radians(obstacle.bearing_body_deg)
            forward -= magnitude * cos(angle)
            right -= magnitude * sin(angle)
        return forward, right, nearest

    def plan(self, snapshot: Snapshot) -> NavCommand:
        gps, imu = snapshot.gps, snapshot.imu
        goal = snapshot.waypoint
        target = snapshot.human_target
        brake = self.braking_m(snapshot)
        if target is not None:
            # A person is approached along the detected bearing, never closer than standoff. The
            # throttle goes off the braking distance early, so the boat coasts to the stand-off.
            desired_body = target.bearing_body_deg
            remaining = target.distance_m - self.standoff_m - brake
            desired_heading = (imu.yaw_deg + desired_body) % 360.0
            self._at_standoff = remaining <= (STANDOFF_RELEASE_M if self._at_standoff else STANDOFF_ARRIVE_M)
            desired_speed = 0.0 if self._at_standoff else float(self.v["approach_speed_mps"])
            reason = "human_standoff" if desired_speed == 0 else "approach_human"
            # The bow follows the camera: turn to the person first, then go to them, and keep the
            # bow on them at the stand-off. The STM32 only steers while it is given a speed, so the
            # turn is made at a creep; never once the boat is inside the stand-off.
            if abs(desired_body) > float(self.v.get("face_tolerance_deg", 10.0)) and remaining > -FACE_INSIDE_M:
                desired_speed = min(float(self.v["approach_speed_mps"]), float(self.v.get("face_speed_mps", 0.15)))
                reason = "face_human"
        elif goal is not None and gps.fix < 2 and not self.v.get("gps_required", True):
            # No fix and vehicle.gps_required off (indoors, the boat on its stand for the judges): the
            # route's speed along the bow, the LiDAR still turning it off obstacles, instead of a
            # bearing reckoned from 0, 0. It never arrives anywhere.
            self._at_standoff = False
            desired_heading = imu.yaw_deg
            desired_speed = float(self.v["max_speed_mps"])
            reason = "bow_no_gps"
        elif goal is not None:
            self._at_standoff = False
            desired_heading = bearing_deg(gps.lat_deg, gps.lon_deg, goal[0], goal[1])
            remaining = haversine_m(gps.lat_deg, gps.lon_deg, goal[0], goal[1])
            desired_speed = float(self.v["max_speed_mps"])
            reason = "waypoint"
            if remaining <= float(self.v["arrive_radius_m"]):
                return NavCommand("STOP", 0.0, imu.yaw_deg, "arrived")
        else:
            self._at_standoff = self._blocked = self._bow_blocked = False
            return NavCommand("STOP", 0.0, imu.yaw_deg, "no_goal")

        # Desired global vector, then add obstacle forces transformed from body to world.
        desired_east, desired_north = body_to_world(cos(radians(signed_angle_deg(desired_heading - imu.yaw_deg))), sin(radians(signed_angle_deg(desired_heading - imu.yaw_deg))), imu.yaw_deg)
        # vehicle.lidar_avoidance false (the boat on its stand for the judges, config/stand_demo.yaml):
        # the LiDAR is read and shown but steers nothing, slows nothing and stops nothing.
        obstacles = snapshot.obstacles if self.v.get("lidar_avoidance", True) else ()
        rep_fwd, rep_right, clearance = self._repulsion_body(obstacles)  # type: ignore[arg-type]
        # Bias is a small consistent starboard tangent. It is applied only when avoiding.
        if rep_fwd or rep_right:
            bias = float(self.p["bias_gain"])
            rep_right += bias * hypot(rep_fwd, rep_right)
        rep_east, rep_north = body_to_world(rep_fwd, rep_right, imu.yaw_deg)
        heading = world_to_heading(float(self.p["attractive_gain"]) * desired_east + rep_east, float(self.p["attractive_gain"]) * desired_north + rep_north)
        turn_error = abs(signed_angle_deg(heading - imu.yaw_deg))
        speed = desired_speed * clamp(1.0 - turn_error / 120.0, 0.25, 1.0)
        # Near an obstacle the boat slows down, and stops if the course it is given leads onto it.
        # It used to stop for anything inside the stop distance, whatever the course: stopped, it
        # could not turn, so it never left the obstacle again. A course that leads away from it is
        # now followed at a creep.
        creep = min(desired_speed, CREEP * float(self.v["max_speed_mps"]))
        # Both distances grow by the way the boat still coasts at the speed it really goes.
        stop_m = float(self.v["stop_distance_m"]) + brake
        slow_m = float(self.v["slow_distance_m"]) + brake
        # Stopped for an obstacle, the boat goes on only once the course is clear by STOP_RELEASE_M more.
        release = STOP_RELEASE_M if self._blocked else 0.0
        stop_limit = stop_m + release
        self._blocked = clearance < slow_m + release and \
            self._course_blocked(obstacles, signed_angle_deg(heading - imu.yaw_deg), stop_limit)  # type: ignore[arg-type]
        # The boat cannot turn on the spot: given a speed to swing away from an obstacle, it first
        # goes on along its bow (and its course over the ground) while it turns. With an obstacle on
        # that way inside turn_room_m plus the braking distance, it stays stopped: wedged next to the
        # obstacle, not driven onto it.
        release = STOP_RELEASE_M if self._bow_blocked else 0.0
        room = float(self.v.get("turn_room_m", 0.0)) + brake + release
        ways = self._ways_ahead(snapshot)
        blocking = [o for way in ways for o in self._blocking(obstacles, way, room)]  # type: ignore[arg-type]
        self._bow_blocked = speed > 0.0 and bool(blocking)
        now = monotonic()
        stern_clear = not self._course_blocked(obstacles, 180.0, stop_limit)  # type: ignore[arg-type]
        if now < self._back_until and stern_clear and (self._blocked or self._bow_blocked):
            return NavCommand("BACK", BACK_THROTTLE, imu.yaw_deg, "backing_off")
        self._back_until = 0.0
        if self._blocked or self._bow_blocked:
            # Stopped there the boat would wait for good (the F407 steers only with some throttle, and a
            # waypoint behind an obstacle keeps the field pointing at it): from rest it creeps along the
            # nearest course to the field's that is clear and that the bow can swing to (_escape_turn).
            # Not while it slides on (ways has the GPS course): a creep would not stop it, STOP brakes
            # with the front motor.
            at_rest = len(ways) == 1
            turn = self._escape_turn(obstacles, blocking, signed_angle_deg(heading - imu.yaw_deg), stop_limit) if at_rest else None  # type: ignore[arg-type]
            if turn is None:
                # No way out ahead (wedged in a doorway, against a wreck): back off, the stern clear.
                if at_rest and stern_clear:
                    self._back_until = now + BACK_S
                    return NavCommand("BACK", BACK_THROTTLE, imu.yaw_deg, "backing_off")
                return NavCommand("STOP", 0.0, heading, "obstacle_too_close" if self._blocked else "bow_blocked")
            heading, speed, reason = (imu.yaw_deg + turn) % 360.0, creep, "turning_away"
        elif clearance < slow_m:
            scale = clamp((clearance - stop_m) / (slow_m - stop_m), 0.0, 1.0)
            speed = max(speed * scale, creep)
            if clearance <= stop_m:
                reason = "leaving_obstacle"
        # Speed 0 in AUTO: the F407 rests the motors (and the front one brakes) and the boat coasts
        # down to the speed it was given, then the throttle comes back.
        if self._governed(speed, gps.speed_mps):
            return NavCommand("AUTO", 0.0, heading, "overspeed")
        return NavCommand("AUTO", speed, heading, reason)


def cpa(forward_m: float, right_m: float, v_forward: float, v_right: float) -> tuple[float, float]:
    """Time to and distance at the closest point of approach to a fixed point (paper, eq. 16 with
    the other vessel at rest). Position and own velocity in the same frame."""
    speed_sq = v_forward * v_forward + v_right * v_right
    if speed_sq < 1e-6:
        return 0.0, hypot(forward_m, right_m)
    tcpa = (forward_m * v_forward + right_m * v_right) / speed_sq
    return tcpa, hypot(forward_m - v_forward * tcpa, right_m - v_right * tcpa)


class _GuidedNavigator:
    """What the two paper navigators share: the goal, the boat's velocity, the gate, the speed law."""

    def __init__(self, vehicle: dict, planner: dict) -> None:
        self.v, self.p = vehicle, planner

    def _goal(self, snapshot: Snapshot) -> tuple[float, float] | None:
        """Bearing of the waypoint from the bow (degrees, clockwise) and its distance; None once there."""
        goal, gps = snapshot.waypoint, snapshot.gps
        if goal is None:
            return None
        distance = haversine_m(gps.lat_deg, gps.lon_deg, goal[0], goal[1])
        if distance <= float(self.v["arrive_radius_m"]):
            return None
        return signed_angle_deg(bearing_deg(gps.lat_deg, gps.lon_deg, goal[0], goal[1]) - snapshot.imu.yaw_deg), distance

    def _velocity_body(self, snapshot: Snapshot) -> tuple[float, float]:
        """Own velocity, forward and to the right. Nearly stopped, the boat is taken to be about to
        move towards its waypoint at cruise speed: the gate needs a direction, and the bow's, on a
        boat turning on the spot, sweeps over every obstacle around it in turn."""
        speed = snapshot.gps.speed_mps
        if speed < 0.2:
            goal = self._goal(snapshot)
            bearing = radians(goal[0]) if goal else 0.0
            return float(self.v["max_speed_mps"]) * cos(bearing), float(self.v["max_speed_mps"]) * sin(bearing)
        course = radians(signed_angle_deg(snapshot.gps.course_deg - snapshot.imu.yaw_deg))
        return speed * cos(course), speed * sin(course)

    def _threats(self, snapshot: Snapshot, v_forward: float, v_right: float) -> list[tuple[float, float]]:
        """Obstacles (forward, right in m) that pass the gate 0 <= TCPA <= TCPA_max and DCPA <= DCPA_min."""
        out = []
        for obstacle in snapshot.obstacles:
            angle = radians(obstacle.bearing_body_deg)  # type: ignore[attr-defined]
            fwd, right = obstacle.distance_m * cos(angle), obstacle.distance_m * sin(angle)  # type: ignore[attr-defined]
            tcpa, dcpa = cpa(fwd, right, v_forward, v_right)
            if 0.0 <= tcpa <= float(self.p["tcpa_max_s"]) and dcpa <= float(self.p["dcpa_min_m"]):
                out.append((fwd, right))
        return out

    def _command(self, snapshot: Snapshot, heading_body_deg: float, speed_scale: float, reason: str) -> NavCommand:
        heading = (snapshot.imu.yaw_deg + heading_body_deg) % 360.0
        speed = float(self.v["max_speed_mps"]) * speed_scale * clamp(1.0 - abs(heading_body_deg) / 120.0, 0.25, 1.0)
        return NavCommand("AUTO", speed, heading, reason)


class PaperBapfNavigator(_GuidedNavigator):
    """Biased artificial potential field (paper, section 3.2).

    Attraction grows with the distance to the goal, capped at att_max. Each obstacle that passes
    the gate repels with -a ln(b d), which is zero beyond the safety boundary d = 1 / b, and with
    a second, weaker field whose source sits bias_offset_m from the obstacle, 60 deg clockwise of
    the line from the obstacle to the boat: on the boat's port side of the obstacle, so the boat
    yields to starboard and two equal pushes never cancel.
    """

    def _repel(self, a: float, b: float, d: float) -> float:
        return clamp(-a * log(b * max(d, 0.05)), 0.0, float(self.p["control_max"]))

    def plan(self, snapshot: Snapshot) -> NavCommand:
        goal = self._goal(snapshot)
        if goal is None:
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "arrived" if snapshot.waypoint else "no_goal")
        goal_bearing, goal_distance = goal
        attract = min(goal_distance, float(self.p["att_max"]))
        fwd, right = attract * cos(radians(goal_bearing)), attract * sin(radians(goal_bearing))
        hull = float(self.v["boat_radius_m"])
        threats = self._threats(snapshot, *self._velocity_body(snapshot))
        for o_fwd, o_right in threats:
            distance = hypot(o_fwd, o_right)
            away_f, away_r = -o_fwd / distance, -o_right / distance
            push = self._repel(float(self.p["a"]), float(self.p["b"]), distance - hull)
            fwd, right = fwd + push * away_f, right + push * away_r
            # The biased source: from the obstacle, 60 deg clockwise of the direction to the boat.
            # (forward, right) rotated clockwise by t is (f cos t - r sin t, f sin t + r cos t).
            t = radians(60.0)
            offset = float(self.p["bias_offset_m"])
            b_fwd = o_fwd + offset * (away_f * cos(t) - away_r * sin(t))
            b_right = o_right + offset * (away_f * sin(t) + away_r * cos(t))
            b_distance = max(hypot(b_fwd, b_right), 0.05)
            push = self._repel(float(self.p["bias_a"]), float(self.p["bias_b"]), b_distance - hull)
            fwd, right = fwd - push * b_fwd / b_distance, right - push * b_right / b_distance
        return self._command(snapshot, degrees(atan2(right, fwd)), 1.0, "avoid" if threats else "waypoint")


class VoNavigator(_GuidedNavigator):
    """Velocity obstacle (paper, section 3.3) for obstacles at rest.

    Candidate velocities: cruise speed along every heading_step_deg around the boat. (Half speed
    was tried as a second set: it always "misses" within the time horizon, so the boat crept
    straight at the obstacle instead of turning.) A candidate is inside the velocity obstacle of an obstacle when, held, it brings the boat
    within radius_m of it in the next tcpa_max_s. The candidate nearest to the goal's bearing that
    is outside all of them is taken; a change of side has to be worth switch_deg, so the choice
    does not flip between two equally good headings.
    """

    def __init__(self, vehicle: dict, planner: dict) -> None:
        super().__init__(vehicle, planner)
        self._last_world_deg: float | None = None

    def plan(self, snapshot: Snapshot) -> NavCommand:
        goal = self._goal(snapshot)
        if goal is None:
            self._last_world_deg = None
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "arrived" if snapshot.waypoint else "no_goal")
        goal_bearing, goal_distance = goal
        cruise, radius = float(self.v["max_speed_mps"]), float(self.p["radius_m"])
        # No further ahead than the waypoint: what stands behind it is not in the way of reaching it.
        # (Without this a waypoint 2 m in front of a wall was never approached.)
        horizon = min(float(self.p["tcpa_max_s"]), goal_distance / cruise)
        points = [(o.distance_m * cos(radians(o.bearing_body_deg)), o.distance_m * sin(radians(o.bearing_body_deg)))  # type: ignore[attr-defined]
                  for o in snapshot.obstacles]
        last = None if self._last_world_deg is None else signed_angle_deg(self._last_world_deg - snapshot.imu.yaw_deg)
        step = float(self.p["heading_step_deg"])
        best: tuple[float, float] | None = None              # cost, heading from the bow
        fallback: tuple[float, float] = (-1.0, 0.0)          # largest miss distance, heading
        for index in range(int(360.0 / step)):
            heading = signed_angle_deg(goal_bearing + (index + 1) // 2 * step * (1 if index % 2 else -1))
            v_f, v_r = cruise * cos(radians(heading)), cruise * sin(radians(heading))
            miss = float("inf")
            for fwd, right in points:
                tcpa, dcpa = cpa(fwd, right, v_f, v_r)
                if 0.0 <= tcpa <= horizon:
                    miss = min(miss, dcpa)
                elif tcpa > horizon:
                    # Not reached within the horizon: where the boat is at its end is what counts.
                    miss = min(miss, hypot(fwd - v_f * horizon, right - v_r * horizon))
            if miss > fallback[0]:
                fallback = (miss, heading)
            if miss <= radius:
                continue
            cost = abs(signed_angle_deg(heading - goal_bearing))
            if last is not None and abs(signed_angle_deg(heading - last)) > step:
                cost += float(self.p["switch_deg"])
            if best is None or cost < best[0]:
                best = (cost, heading)
        if best is None:
            # Every velocity leads into something: creep along the one that misses by most.
            self._last_world_deg = (snapshot.imu.yaw_deg + fallback[1]) % 360.0
            return self._command(snapshot, fallback[1], 0.3, "boxed_in")
        self._last_world_deg = (snapshot.imu.yaw_deg + best[1]) % 360.0
        return self._command(snapshot, best[1], 1.0, "waypoint" if abs(signed_angle_deg(best[1] - goal_bearing)) < step else "avoid")

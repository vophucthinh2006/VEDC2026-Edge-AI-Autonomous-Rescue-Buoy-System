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


class BapfNavigator:
    def __init__(self, vehicle: dict, planner: dict) -> None:
        self.v = vehicle
        self.p = planner

    def _course_blocked(self, obstacles: tuple[Obstacle, ...], heading_body_deg: float) -> bool:
        """True when an obstacle inside the stop distance lies on the commanded course: held, the
        course would bring the boat's inflated hull onto it (closest point of approach, as in
        Jo et al. 2022, eq. 16)."""
        inflated_radius = float(self.v["boat_radius_m"]) + float(self.v["safety_margin_m"])
        v_fwd, v_right = cos(radians(heading_body_deg)), sin(radians(heading_body_deg))
        for obstacle in obstacles:
            if obstacle.distance_m - inflated_radius - obstacle.width_m / 2 > float(self.v["stop_distance_m"]):
                continue
            angle = radians(obstacle.bearing_body_deg)
            tcpa, dcpa = cpa(obstacle.distance_m * cos(angle), obstacle.distance_m * sin(angle), v_fwd, v_right)
            if tcpa >= 0.0 and dcpa <= inflated_radius + obstacle.width_m / 2:
                return True
        return False

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
        if target is not None:
            # A person is approached along the detected bearing, never closer than standoff.
            desired_body = target.bearing_body_deg
            remaining = target.distance_m - float(self.v["target_standoff_m"])
            desired_heading = (imu.yaw_deg + desired_body) % 360.0
            desired_speed = float(self.v["approach_speed_mps"]) if remaining > 0.15 else 0.0
            reason = "human_standoff" if desired_speed == 0 else "approach_human"
        elif goal is not None:
            desired_heading = bearing_deg(gps.lat_deg, gps.lon_deg, goal[0], goal[1])
            remaining = haversine_m(gps.lat_deg, gps.lon_deg, goal[0], goal[1])
            desired_speed = float(self.v["max_speed_mps"])
            reason = "waypoint"
            if remaining <= float(self.v["arrive_radius_m"]):
                return NavCommand("STOP", 0.0, imu.yaw_deg, "arrived")
        else:
            return NavCommand("STOP", 0.0, imu.yaw_deg, "no_goal")

        # Desired global vector, then add obstacle forces transformed from body to world.
        desired_east, desired_north = body_to_world(cos(radians(signed_angle_deg(desired_heading - imu.yaw_deg))), sin(radians(signed_angle_deg(desired_heading - imu.yaw_deg))), imu.yaw_deg)
        rep_fwd, rep_right, clearance = self._repulsion_body(snapshot.obstacles)  # type: ignore[arg-type]
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
        if clearance < float(self.v["slow_distance_m"]):
            if self._course_blocked(snapshot.obstacles, signed_angle_deg(heading - imu.yaw_deg)):  # type: ignore[arg-type]
                return NavCommand("STOP", 0.0, heading, "obstacle_too_close")
            scale = clamp((clearance - float(self.v["stop_distance_m"])) / (float(self.v["slow_distance_m"]) - float(self.v["stop_distance_m"])), 0.0, 1.0)
            speed = max(speed * scale, creep)
            if clearance <= float(self.v["stop_distance_m"]):
                reason = "leaving_obstacle"
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

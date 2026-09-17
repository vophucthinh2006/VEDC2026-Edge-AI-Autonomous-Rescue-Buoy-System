"""Waypoint guidance and starboard-biased artificial potential field."""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, radians, sin

from modules.perception import Obstacle
from modules.state_store import Snapshot
from utils.geometry import bearing_deg, body_to_world, clamp, haversine_m, signed_angle_deg, world_to_heading


@dataclass(frozen=True)
class NavCommand:
    mode: str
    speed_mps: float
    heading_deg: float
    reason: str


class BapfNavigator:
    def __init__(self, vehicle: dict, planner: dict) -> None:
        self.v = vehicle
        self.p = planner

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
        if clearance <= float(self.v["stop_distance_m"]):
            return NavCommand("STOP", 0.0, heading, "obstacle_too_close")
        if clearance < float(self.v["slow_distance_m"]):
            speed *= clamp((clearance - float(self.v["stop_distance_m"])) / (float(self.v["slow_distance_m"]) - float(self.v["stop_distance_m"])), 0.0, 1.0)
        return NavCommand("AUTO", speed, heading, reason)

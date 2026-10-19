"""Pi-side safety arbiter. STM32 independently enforces all motor stop conditions."""
from __future__ import annotations

from time import monotonic
from typing import Callable

from modules.navigation import NavCommand
from modules.state_store import Snapshot


class SafetyArbiter:
    def __init__(self, vehicle: dict, inside_fence: Callable[[float, float], bool] | None = None) -> None:
        """inside_fence(lat, lon): the mission's geofence (modules/waypoints.py Route.inside_fence)."""
        self.v = vehicle
        self.inside_fence = inside_fence

    def check(self, snapshot: Snapshot, proposed: NavCommand) -> NavCommand:
        now = monotonic()
        if snapshot.system.estop or snapshot.system.motor_fault:
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "stm32_fault_or_estop")
        if snapshot.imu.overturned or abs(snapshot.imu.pitch_deg) > self.v["max_pitch_deg"] or abs(snapshot.imu.roll_deg) > self.v["max_roll_deg"]:
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "attitude_fault")
        # vehicle.imu_required: false only for the boat on its stand (config/bench_demo.yaml), with
        # the STM32F407 built with BENCH_IGNORE_IMU: the heading is then the last one the IMU read.
        if self.v.get("imu_required", True) and (not snapshot.imu.imu_ok or now - snapshot.imu.timestamp > self.v["imu_stale_s"]):
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "imu_stale")
        if proposed.mode == "AUTO" and self.v.get("lidar_required", True) and now - snapshot.scan_timestamp > self.v["lidar_stale_s"]:
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "lidar_stale")
        if proposed.reason == "waypoint" and (snapshot.gps.fix < 2 or now - snapshot.gps.timestamp > self.v["gps_stale_s"]):
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "gps_stale_or_no_fix")
        # Outside the fence the boat stops where it is: without a LiDAR, a way back could lead onto the pool wall.
        if (proposed.mode == "AUTO" and self.inside_fence is not None and snapshot.gps.fix >= 2
                and not self.inside_fence(snapshot.gps.lat_deg, snapshot.gps.lon_deg)):
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "outside_geofence")
        return proposed

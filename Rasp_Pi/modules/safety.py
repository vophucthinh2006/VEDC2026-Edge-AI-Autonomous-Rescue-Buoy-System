"""Pi-side safety arbiter. STM32 independently enforces all motor stop conditions."""
from __future__ import annotations

from time import monotonic

from modules.navigation import NavCommand
from modules.state_store import Snapshot


class SafetyArbiter:
    def __init__(self, vehicle: dict) -> None:
        self.v = vehicle

    def check(self, snapshot: Snapshot, proposed: NavCommand) -> NavCommand:
        now = monotonic()
        if snapshot.system.estop or snapshot.system.motor_fault:
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "stm32_fault_or_estop")
        if snapshot.imu.overturned or abs(snapshot.imu.pitch_deg) > self.v["max_pitch_deg"] or abs(snapshot.imu.roll_deg) > self.v["max_roll_deg"]:
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "attitude_fault")
        if not snapshot.imu.imu_ok or now - snapshot.imu.timestamp > self.v["imu_stale_s"]:
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "imu_stale")
        if proposed.mode == "AUTO" and now - snapshot.scan_timestamp > self.v["lidar_stale_s"]:
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "lidar_stale")
        if proposed.reason == "waypoint" and (snapshot.gps.fix < 2 or now - snapshot.gps.timestamp > self.v["gps_stale_s"]):
            return NavCommand("STOP", 0.0, snapshot.imu.yaw_deg, "gps_stale_or_no_fix")
        return proposed

import unittest
from time import monotonic

from modules.navigation import NavCommand
from modules.safety import SafetyArbiter
from modules.state_store import GpsState, HumanTarget, ImuState, Snapshot, SysState

VEHICLE = {"max_pitch_deg": 35.0, "max_roll_deg": 35.0, "imu_stale_s": 0.3, "lidar_required": False,
           "lidar_stale_s": 0.5, "gps_stale_s": 2.0}
WAYPOINT = (10.8835, 106.7962)


def snap(fix=0, gps_age_s=0.0, person=False):
    now = monotonic()
    target = HumanTarget(5.0, 4.0, 0.7, now) if person else None
    return Snapshot(ImuState(yaw_deg=90.0, imu_ok=True, timestamp=now), GpsState(fix=fix, timestamp=now - gps_age_s),
                    SysState(timestamp=now), (), now, WAYPOINT, target)


class GpsGateTest(unittest.TestCase):
    """Driven to a position (a waypoint, home) the boat needs a GPS fix, whatever the reason the
    navigator gives; driven to a person, the camera's bearing is enough."""

    def setUp(self):
        self.safety = SafetyArbiter(VEHICLE)

    def test_no_fix_stops_every_reason_on_the_way_to_a_position(self):
        # On the stand (2026-10-22, GPS fix 0) the boat crept on with leaving_obstacle, the course
        # reckoned from 0, 0, because only reason "waypoint" was held to the fix.
        for reason, speed in (("waypoint", 0.3), ("leaving_obstacle", 0.09), ("overspeed", 0.0)):
            command = self.safety.check(snap(fix=0), NavCommand("AUTO", speed, 55.0, reason))
            self.assertEqual((command.mode, command.reason), ("STOP", "gps_stale_or_no_fix"), reason)

    def test_a_stale_fix_is_no_fix(self):
        command = self.safety.check(snap(fix=3, gps_age_s=3.0), NavCommand("AUTO", 0.09, 55.0, "leaving_obstacle"))
        self.assertEqual(command.reason, "gps_stale_or_no_fix")

    def test_with_a_fix_the_navigator_has_its_way(self):
        go = NavCommand("AUTO", 0.09, 55.0, "leaving_obstacle")
        self.assertIs(self.safety.check(snap(fix=3), go), go)

    def test_a_person_is_followed_without_a_fix(self):
        for reason in ("approach_human", "face_human", "leaving_obstacle"):
            go = NavCommand("AUTO", 0.15, 95.0, reason)
            self.assertIs(self.safety.check(snap(fix=0, person=True), go), go, reason)

    def test_gps_required_off_lets_the_navigator_drive_without_a_fix(self):
        safety = SafetyArbiter({**VEHICLE, "gps_required": False})
        go = NavCommand("AUTO", 0.3, 90.0, "bow_no_gps")
        self.assertIs(safety.check(snap(fix=0), go), go)

    def test_backing_off_needs_no_fix_but_a_fresh_lidar(self):
        back = NavCommand("BACK", 0.3, 90.0, "backing_off")
        self.assertIs(self.safety.check(snap(fix=0), back), back)
        fresh = snap(fix=3)
        stale = Snapshot(fresh.imu, fresh.gps, fresh.system, (), monotonic() - 5.0, WAYPOINT, None)
        self.assertEqual(SafetyArbiter({**VEHICLE, "lidar_required": True}).check(stale, back).reason, "lidar_stale")

    def test_a_stop_stays_a_stop(self):
        stop = NavCommand("STOP", 0.0, 90.0, "bow_blocked")
        self.assertIs(self.safety.check(snap(fix=0), stop), stop)


if __name__ == "__main__":
    unittest.main()

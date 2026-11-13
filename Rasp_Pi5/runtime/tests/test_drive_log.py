import unittest

from modules.drive_log import SILENT_S, DriveLog
from modules.navigation import NavCommand
from modules.state_store import GpsState, ImuState, Snapshot, SysState, VehicleState

NOW = 1000.0
AUTO = NavCommand("AUTO", 0.30, 90.0, "waypoint")


def snap(imu_ok=True, armed=True, sys_at=NOW, overturned=False, hold=False):
    from modules.state_store import LinkState
    return Snapshot(ImuState(imu_ok=imu_ok, overturned=overturned, timestamp=NOW - 0.05),
                    GpsState(fix=3, timestamp=NOW - 0.5), SysState(battery_v=11.6, link_ok=True, timestamp=sys_at), (), NOW,
                    None, None, VehicleState("AUTO", armed, 0, sys_at), link=LinkState(hold=hold))


class DriveLogTest(unittest.TestCase):
    def test_a_stop_says_why_and_what_the_pi_knew(self) -> None:
        log = DriveLog()
        self.assertEqual(log.update(AUTO, snap(), NOW), ["boat driving: waypoint 0.30 m/s heading 90"])
        self.assertEqual(log.update(AUTO, snap(), NOW), [])          # nothing changed: nothing said
        lines = log.update(NavCommand("STOP", 0.0, 90.0, "imu_stale"), snap(imu_ok=False), NOW)
        self.assertEqual(len(lines), 1)
        self.assertIn("boat stopped: STOP imu_stale", lines[0])
        self.assertIn("IMU NOT ok", lines[0])
        self.assertIn("GPS fix 3", lines[0])
        self.assertEqual(log.update(NavCommand("STOP", 0.0, 90.0, "imu_stale"), snap(imu_ok=False), NOW), [])

    def test_a_speed_of_zero_is_a_stop_too(self) -> None:
        log = DriveLog()
        log.update(AUTO, snap(), NOW)
        lines = log.update(NavCommand("AUTO", 0.0, 90.0, "human_standoff"), snap(), NOW)
        self.assertIn("boat stopped: AUTO human_standoff", lines[0])

    def test_coasting_down_to_the_speed_given_is_not_a_stop(self) -> None:
        # The throttle off and on again every few seconds (overspeed) is not a line each time.
        log = DriveLog()
        log.update(AUTO, snap(), NOW)
        self.assertEqual(log.update(NavCommand("AUTO", 0.0, 90.0, "overspeed"), snap(), NOW), [])
        self.assertEqual(log.update(AUTO, snap(), NOW), [])

    def test_the_f407_disarming_with_its_likely_cause(self) -> None:
        log = DriveLog()
        log.update(AUTO, snap(), NOW)
        lines = log.update(AUTO, snap(imu_ok=False, armed=False), NOW)
        self.assertTrue(any("F407 DISARMED (IMU lost)" in line for line in lines), lines)
        self.assertEqual(log.update(AUTO, snap(armed=True), NOW), ["F407 armed"])
        lines = log.update(AUTO, snap(armed=False, overturned=True), NOW)
        self.assertTrue(any("F407 DISARMED (overturned)" in line for line in lines), lines)

    def test_the_first_sys_is_not_a_change(self) -> None:
        log = DriveLog()
        self.assertEqual(log.update(NavCommand("STOP", 0.0, 0.0, "no_goal"), snap(armed=False), NOW), [])

    def test_the_f407_going_silent_and_coming_back(self) -> None:
        log = DriveLog()
        log.update(AUTO, snap(), NOW)
        lines = log.update(NavCommand("STOP", 0.0, 90.0, "imu_stale"), snap(sys_at=NOW - SILENT_S - 1.0), NOW)
        self.assertTrue(lines[0].startswith("F407 silent: no $SYS for 2.0 s"), lines)
        self.assertIn("boat stopped", lines[1])
        self.assertEqual(log.update(AUTO, snap(), NOW)[0], "F407 heard again")

    def test_the_dashboard_hold_shows(self) -> None:
        log = DriveLog()
        log.update(AUTO, snap(), NOW)
        lines = log.update(NavCommand("STOP", 0.0, 90.0, "dashboard_hold"), snap(hold=True), NOW)
        self.assertIn("HOLD from the dashboard", lines[0])


if __name__ == "__main__":
    unittest.main()

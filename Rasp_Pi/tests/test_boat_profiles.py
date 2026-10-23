import tempfile
import threading
import unittest
from pathlib import Path
from time import monotonic

from modules.lidar_ld14 import LidarPoint
from modules.lidar_serial import drop_self_returns, in_sector, to_boat_frame
from modules.obstacle_feed import lidar_fresh
from modules.helper_board import HelperBoard, position_fields
from modules.perception import ray_obstacles
from modules.state_store import GpsState, ImuState, Snapshot, SysState, VehicleState
from utils.config import load_config
from utils.nmea_packet import encode

CONFIG = Path(__file__).resolve().parent.parent / "config"


class ConfigTest(unittest.TestCase):
    def test_overlay_values_win_and_the_rest_is_kept(self):
        with tempfile.TemporaryDirectory() as folder:
            base, over, empty = Path(folder, "base.yaml"), Path(folder, "over.yaml"), Path(folder, "empty.yaml")
            base.write_text("a: {x: 1, y: 2}\nb: 3\n", encoding="utf-8")
            over.write_text("a: {y: 5}\nc: 6\n", encoding="utf-8")
            empty.write_text("# nothing differs yet\n", encoding="utf-8")
            self.assertEqual(load_config(base, [over, empty]), {"a": {"x": 1, "y": 5}, "b": 3, "c": 6})

    def test_the_pixhawk_boat_reads_its_lidar_from_the_serial_port(self):
        self.assertEqual(load_config(CONFIG / "settings.yaml")["lidar"]["source"], "none")
        boat = load_config(CONFIG / "settings.yaml", [CONFIG / "boat_pixhawk.yaml"])
        self.assertEqual(boat["lidar"]["source"], "serial")
        self.assertIn("pan_output", boat["camera"])
        self.assertEqual(boat["camera"]["horizontal_fov_deg"], 60.0)      # settings.yaml still underneath

    def test_the_simulator_is_not_touched_by_the_boat_files(self):
        self.assertEqual(load_config(CONFIG / "settings.yaml", [CONFIG / "sim.yaml"])["lidar"]["source"], "gazebo")

    def test_the_stm32_boat_file_loads_and_the_model_is_the_one_in_the_repository(self):
        boat = load_config(CONFIG / "settings.yaml", [CONFIG / "boat_stm32.yaml"])
        self.assertTrue((CONFIG.parent / boat["camera"]["model_path"]).is_file())


class StandDemoTest(unittest.TestCase):
    """config/stand_demo.yaml (run_all.bat stand): the boat on its stand in a small room. Everything as
    on the water but the obstacle avoidance: walls and people close round the stand never stop it."""

    def setUp(self):
        from modules.navigation import BapfNavigator
        from modules.safety import SafetyArbiter
        self.water = load_config(CONFIG / "settings.yaml", [CONFIG / "boat_stm32.yaml"])
        self.config = load_config(CONFIG / "settings.yaml", [CONFIG / "boat_stm32.yaml", CONFIG / "stand_demo.yaml"])
        self.navigator = BapfNavigator(self.config["vehicle"], self.config["planner"])
        self.safety = SafetyArbiter(self.config["vehicle"])

    def drive(self, person=None):
        from modules.state_store import HumanTarget
        from modules.perception import Obstacle
        now = monotonic()
        # No GPS fix, a route from the dashboard, the table and the stand all round within 0.3-0.8 m.
        ring = tuple(Obstacle(float(b), 0.3 + 0.5 * (b % 60 == 0), 0.3) for b in range(-180, 180, 30))
        target = HumanTarget(person[0], person[1], 0.7, now) if person else None
        snap = Snapshot(ImuState(yaw_deg=40.0, imu_ok=True, timestamp=now), GpsState(fix=0, timestamp=now),
                        SysState(timestamp=now), ring, now, (10.8834335, 106.7962386), target)
        return self.safety.check(snap, self.navigator.plan(snap))

    def test_only_the_obstacle_avoidance_differs_from_the_water(self):
        stand = {**self.config, "vehicle": {k: v for k, v in self.config["vehicle"].items() if k != "lidar_avoidance"}}
        self.assertEqual(stand, self.water)
        self.assertFalse(self.config["vehicle"]["lidar_avoidance"])
        self.assertTrue(self.water["vehicle"].get("lidar_avoidance", True))

    def test_the_route_runs_whatever_the_lidar_sees(self):
        command = self.drive()
        self.assertEqual((command.mode, command.reason), ("AUTO", "bow_no_gps"))
        self.assertGreater(command.speed_mps, 0.0)
        self.assertAlmostEqual(command.heading_deg, 40.0)

    def test_a_person_is_gone_to_with_the_stand_boxed_in(self):
        # 4 m away, 25 deg to starboard: the bow turns to them and the motors run.
        command = self.drive((25.0, 4.0))
        self.assertEqual(command.mode, "AUTO")
        self.assertGreater(command.speed_mps, 0.0)
        self.assertAlmostEqual(command.heading_deg, 65.0)
        # The bow 2 m off the person, as on the water: a range of 2.25 m from the LiDAR and camera.
        self.assertAlmostEqual(self.navigator.standoff_m, 2.25)
        self.assertEqual(self.drive((2.0, 2.2)).reason, "human_standoff")
        self.assertEqual(self.drive((2.0, 2.8)).reason, "approach_human")

    def test_the_water_holds_2_m_off_the_bow_too(self):
        from modules.navigation import BapfNavigator
        water = BapfNavigator(self.water["vehicle"], self.water["planner"])
        self.assertAlmostEqual(water.standoff_m - self.water["vehicle"]["bow_ahead_m"], 2.0)


class SerialLidarTest(unittest.TestCase):
    def test_angles_turn_to_the_boat_frame(self):
        scan = (LidarPoint(10.0, 2.0, 50), LidarPoint(350.0, 3.0, 50))
        self.assertEqual([p.angle_deg for p in to_boat_frame(scan, 0.0, False)], [10.0, 350.0])
        self.assertEqual([p.angle_deg for p in to_boat_frame(scan, 90.0, False)], [100.0, 80.0])
        # Anticlockwise sensor: 10 deg on the sensor is 10 deg to port of the bow.
        self.assertEqual([p.angle_deg for p in to_boat_frame(scan, 0.0, True)], [350.0, 10.0])
        self.assertEqual(to_boat_frame(scan, 0.0, True)[0].distance_m, 2.0)

    def test_the_boats_own_parts_are_dropped(self):
        scan = (LidarPoint(172.0, 0.10, 90), LidarPoint(10.0, 0.25, 90), LidarPoint(10.0, 2.0, 90),
                LidarPoint(235.0, 0.65, 90), LidarPoint(235.0, 1.5, 90), LidarPoint(355.0, 0.5, 90),
                LidarPoint(90.0, 0.0, 0))
        kept = drop_self_returns(scan, 0.30, [[228, 250, 0.9], [350, 5, 0.6]])
        self.assertEqual(len(kept), len(scan))   # rays are kept, as "nothing seen"
        self.assertEqual([(p.angle_deg, p.distance_m) for p in kept if p.distance_m > 0], [(10.0, 2.0), (235.0, 1.5)])
        self.assertTrue(all(p.intensity == 0 for p in kept if p.distance_m == 0.0))
        self.assertEqual(drop_self_returns(scan, 0.0, []), scan)

    def test_a_mask_sector_may_run_across_zero(self):
        self.assertTrue(in_sector(355.0, 350, 5) and in_sector(3.0, 350, 5) and in_sector(350.0, 350, 5))
        self.assertFalse(in_sector(10.0, 350, 5) or in_sector(180.0, 350, 5))
        self.assertTrue(in_sector(240.0, 228, 250) and not in_sector(251.0, 228, 250))

    def test_a_wall_ahead_becomes_one_obstacle_per_ray_and_a_lone_return_is_dropped(self):
        scan = tuple(LidarPoint(float(a % 360), 3.0, 80) for a in range(-5, 6)) + (LidarPoint(120.0, 2.0, 80),)
        obstacles = ray_obstacles([(p.angle_deg, p.distance_m) for p in to_boat_frame(scan, 0.0, False) if p.intensity > 0], 0.15, 8.0)
        self.assertEqual(len(obstacles), 11)
        self.assertTrue(all(abs(o.bearing_body_deg) <= 5.0 and o.distance_m == 3.0 for o in obstacles))


class ObstacleFeedFreshnessTest(unittest.TestCase):
    def test_without_a_lidar_the_keep_outs_are_always_sent(self):
        self.assertTrue(lidar_fresh(0.0, 1000.0, None))

    def test_with_a_lidar_an_old_scan_stops_the_feed(self):
        self.assertTrue(lidar_fresh(99.5, 100.0, 1.0))
        self.assertFalse(lidar_fresh(98.0, 100.0, 1.0))
        self.assertFalse(lidar_fresh(0.0, 100.0, 1.0))        # no scan yet since the start


class HelperBoardTest(unittest.TestCase):
    """The POS packet for the STM32F103: the board forwards these texts into the LoRa frame as they
    are (STM32F103/Tests/helper_test.c feeds it the same lines)."""

    @staticmethod
    def snap(now, lat=10.762622, lon=106.660172, fix=3, yaw=278.5, roll=2.1, pitch=-1.4, mode="GUIDED", armed=True, imu_age=0.0):
        return Snapshot(ImuState(pitch, roll, yaw, True, False, now - imu_age), GpsState(lat, lon, fix, 1.0, 0.0, 0.0, now),
                        SysState(), (), 0.0, None, None, VehicleState(mode, armed, 0, now))

    def test_fields_as_the_board_and_the_station_expect_them(self):
        self.assertEqual(position_fields(self.snap(50.0), 50.0), ("1", "10.762622", "106.660172", "2.1", "-1.4", "278.5", "-1.0", "A", "1", "3"))

    def test_no_fix_sends_no_position_and_yaw_never_reads_360(self):
        fields = position_fields(self.snap(50.0, fix=1, yaw=359.97), 50.0)
        self.assertEqual(fields[:3], ("0", "0.000000", "0.000000"))
        self.assertEqual((fields[5], fields[9]), ("0.0", "0"))

    def test_mode_letter_follows_the_autopilot(self):
        self.assertEqual(position_fields(self.snap(1.0, mode="MANUAL"), 1.0)[7], "M")
        self.assertEqual(position_fields(self.snap(1.0, mode="HOLD"), 1.0)[7], "S")
        self.assertEqual(position_fields(self.snap(1.0, mode="AUTO", armed=False), 1.0)[7], "S")

    def test_a_stale_attitude_is_flagged_and_the_longest_values_keep_their_width(self):
        fields = position_fields(self.snap(50.0, lat=-10.762622, lon=-106.660172, roll=-179.94, pitch=-89.96, imu_age=5.0), 50.0)
        self.assertEqual(fields[8], "0")
        self.assertEqual(fields[1:5], ("-10.762622", "-106.660172", "-179.9", "-90.0"))
        self.assertTrue(all(len(text) <= limit for text, limit in zip(fields[1:7], (11, 11, 6, 5, 5, 5))))   # is_number() limits in pi_link.c

    def test_the_board_report_is_read_and_goes_stale(self):
        board = HelperBoard("none", 115200, threading.Event())
        self.assertFalse(board.online(monotonic()))
        board._handle(encode("AUX", 4, 12340, -250, 1, 1, 0))
        self.assertEqual((board.state.battery_v, board.state.current_a, board.state.estop, board.state.lora_ok, board.state.alarm),
                         (12.34, -0.25, True, True, False))
        self.assertTrue(board.online(monotonic()))
        self.assertFalse(board.online(monotonic() + 10.0))
        board._handle(b"$AUX,5,x,0,0,0,0*00\n")                   # bad checksum: ignored
        self.assertTrue(board.state.estop)


class RunLogTest(unittest.TestCase):
    def test_a_run_is_written_one_line_per_call_with_the_board_when_it_answers(self):
        from modules.helper_board import BoardState
        from utils.run_log import COLUMNS, RunLog
        import csv
        with tempfile.TemporaryDirectory() as folder:
            run = RunLog(Path(folder))
            now = monotonic()
            snap = HelperBoardTest.snap(now)
            run.write(snap, now, "search", 0)
            run.write(snap, now + 1.0, "approach", 1, BoardState(12.34, 1.5, True, True, True, now), True)
            run.write(snap, now + 2.0, "approach", 1, BoardState(12.34, 1.5, True, True, True, now), False)
            run.close()
            with run.path.open(encoding="utf-8") as written:
                rows = list(csv.DictReader(written))
        self.assertEqual(tuple(rows[0].keys()), COLUMNS)
        self.assertEqual((rows[0]["mode"], rows[0]["phase"], rows[0]["lat"], rows[0]["board"], rows[0]["battery_v"]), ("GUIDED", "search", "10.7626220", "", ""))
        self.assertEqual((rows[1]["people_found"], rows[1]["board"], rows[1]["battery_v"], rows[1]["estop"]), ("1", "1", "12.34", "1"))
        self.assertEqual((rows[2]["board"], rows[2]["battery_v"]), ("0", ""))

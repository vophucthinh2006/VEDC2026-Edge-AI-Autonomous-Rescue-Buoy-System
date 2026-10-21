import threading
import unittest
from time import monotonic, sleep

try:
    import serial
    from modules.uart_comm import Stm32Uart
except ImportError:          # pyserial missing
    Stm32Uart = None

from modules.state_store import StateStore
from utils.nmea_packet import decode, encode


@unittest.skipIf(Stm32Uart is None, "pyserial not installed")
class Stm32LinesTest(unittest.TestCase):
    """Lines from the STM32F407, parsed into the state store (STM32F4/README.md section 11)."""

    def setUp(self):
        self.state = StateStore()
        self.uart = Stm32Uart("unused", 115200, self.state, threading.Event())   # never started: no port opened

    def feed(self, command, *fields):
        self.uart._handle(decode(encode(command, *fields)))

    def test_without_lnk_the_shore_link_counts_as_up_with_no_home(self):
        link = self.state.snapshot().link
        self.assertTrue(link.shore_ok)
        self.assertIsNone(link.home)
        self.assertEqual(link.heard_ago_s, -1.0)

    def test_lnk_with_a_home(self):
        self.feed("LNK", 4, 1, "2.5", 1, "10.762600", "106.660100")
        link = self.state.snapshot().link
        self.assertTrue(link.shore_ok)
        self.assertAlmostEqual(link.heard_ago_s, 2.5)
        self.assertEqual(link.home, (10.7626, 106.6601))

    def test_lnk_lost_without_a_home(self):
        self.feed("LNK", 5, 0, "-1", 0, "", "")
        link = self.state.snapshot().link
        self.assertFalse(link.shore_ok)
        self.assertEqual(link.heard_ago_s, -1.0)
        self.assertIsNone(link.home)

    def test_lnk_hold_from_the_dashboard(self):
        self.feed("LNK", 7, 1, "1.0", 0, "", "", 1)
        self.assertTrue(self.state.snapshot().link.hold)
        self.feed("LNK", 8, 1, "1.0", 0, "", "", 0)
        self.assertFalse(self.state.snapshot().link.hold)
        self.feed("LNK", 9, 1, "1.0", 1, "10.762600", "106.660100")   # older firmware: no hold field
        self.assertFalse(self.state.snapshot().link.hold)

    def test_lnk_home_valid_0_ignores_any_position(self):
        self.feed("LNK", 6, 0, "75.0", 0, "10.762600", "106.660100")
        self.assertIsNone(self.state.snapshot().link.home)

    def test_sys_with_the_battery_percent(self):
        self.feed("SYS", 7, "11.84", 0, 0, 1, 1, 1, 3, 72)
        system = self.state.snapshot().system
        self.assertAlmostEqual(system.battery_v, 11.84)
        self.assertEqual(system.confirm_presses, 3)
        self.assertEqual(system.battery_pct, 72)

    def test_sys_of_older_firmware(self):
        self.feed("SYS", 8, "11.84", 0, 0, 1, 1, 1, 2)
        self.assertEqual(self.state.snapshot().system.battery_pct, -1)
        self.assertEqual(self.state.snapshot().system.confirm_presses, 2)
        self.feed("SYS", 9, "11.84", 0, 0, 1)
        self.assertEqual(self.state.snapshot().system.confirm_presses, -1)

    def test_gps_fix_quality_1_is_a_fix_the_boat_may_drive_on(self):
        # A plain GPS fix (GGA quality 1), what the boat's receiver reports, was read as "no fix":
        # safety.py stopped every waypoint with gps_stale_or_no_fix.
        from modules.navigation import NavCommand
        from modules.safety import SafetyArbiter
        vehicle = {"max_pitch_deg": 35.0, "max_roll_deg": 35.0, "imu_stale_s": 0.3, "lidar_required": False,
                   "lidar_stale_s": 0.5, "gps_stale_s": 2.0}
        safety = SafetyArbiter(vehicle)
        go = NavCommand("AUTO", 0.3, 90.0, "waypoint")
        self.feed("IMU", 1, "0.00", "0.00", "90.00", 1, 0)
        for quality, fix_type, allowed in ((1, 3, True), (2, 3, True), (4, 3, True), (0, 0, False), (6, 0, False)):
            self.feed("GPS", 2, "10.883438", "106.796019", quality, "0.9", "0.20", "90.0")
            snap = self.state.snapshot()
            self.assertEqual(snap.gps.fix, fix_type, quality)
            self.assertEqual(safety.check(snap, go).reason, "waypoint" if allowed else "gps_stale_or_no_fix", quality)

    def test_a_bad_lnk_counts_as_an_error(self):
        self.feed("LNK", 10, 1, "x", 0)
        self.assertEqual(self.uart.rx_errors, 1)


class FakePort:
    """A serial port that plays a script: bytes are lines read, an exception is raised once."""

    def __init__(self, script):
        self.script, self.written = list(script), []

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return False

    def readline(self):
        if self.script:
            item = self.script.pop(0)
            if isinstance(item, Exception):
                raise item
            return item
        sleep(0.002)
        return b""

    def write(self, data):
        self.written.append(data)


@unittest.skipIf(Stm32Uart is None, "pyserial not installed")
class Stm32ReconnectTest(unittest.TestCase):
    """The port failing no longer stops main.py: it is opened again."""

    def run_uart(self, opens, until, timeout=3.0):
        """opens: what each open returns (a FakePort) or raises. Runs the thread until until(uart)."""
        stop, opened = threading.Event(), list(opens)

        def open_port():
            item = opened.pop(0) if opened else FakePort([])
            if isinstance(item, Exception):
                raise item
            return item
        uart = Stm32Uart("fake", 115200, StateStore(), stop, open_port=open_port, reopen_s=0.01)
        uart.start()
        deadline = monotonic() + timeout
        while not until(uart) and monotonic() < deadline:
            sleep(0.005)
        stop.set()
        uart.join(timeout=1.0)
        self.assertFalse(uart.is_alive())
        return uart, stop

    def test_a_port_that_drops_is_opened_again_and_read_on(self):
        imu = encode("IMU", 1, "1.00", "2.00", "90.00", 1, 0)
        first = FakePort([imu, serial.SerialException("device reports readiness to read but returned no data")])
        second = FakePort([encode("IMU", 2, "3.00", "4.00", "180.00", 1, 0)])
        uart, _ = self.run_uart([first, second], lambda u: u.drops == 1 and u._state.snapshot().imu.yaw_deg == 180.0)
        self.assertEqual(uart.drops, 1)
        self.assertEqual(uart._state.snapshot().imu.yaw_deg, 180.0)

    def test_a_port_missing_at_start_is_waited_for(self):
        missing = serial.SerialException("could not open port")
        port = FakePort([encode("IMU", 1, "0.00", "0.00", "45.00", 1, 0)])
        uart, stop = self.run_uart([missing, missing, OSError(5, "Input/output error"), port],
                                   lambda u: u._state.snapshot().imu.yaw_deg == 45.0)
        self.assertEqual(uart.drops, 0)            # never open before: not a drop
        self.assertEqual(uart._state.snapshot().imu.yaw_deg, 45.0)

    def test_nothing_queued_while_down_goes_out_late(self):
        port = FakePort([])
        stop = threading.Event()
        opens = [serial.SerialException("gone"), port]

        def open_port():
            item = opens.pop(0) if opens else FakePort([])
            if isinstance(item, Exception):
                raise item
            return item
        uart = Stm32Uart("fake", 115200, StateStore(), stop, open_port=open_port, reopen_s=0.05)
        uart.send("NAV", 1, "AUTO", "0.30", "90.0", 300)   # not open yet: dropped
        uart.start()
        deadline = monotonic() + 2.0
        while not uart.connected and monotonic() < deadline:
            sleep(0.002)
        self.assertTrue(uart.connected)
        uart.send("NAV", 2, "STOP", "0.00", "90.0", 300)
        deadline = monotonic() + 2.0
        while not port.written and monotonic() < deadline:
            sleep(0.002)
        stop.set()
        uart.join(timeout=1.0)
        self.assertEqual([decode(line).fields[0] for line in port.written], ["2"])

    def test_a_bug_in_a_handler_does_not_end_the_link(self):
        class Flaky(Stm32Uart):
            calls = 0

            def _handle(self, packet):
                Flaky.calls += 1
                if Flaky.calls == 1:
                    raise RuntimeError("bug")
                super()._handle(packet)
        stop = threading.Event()
        ports = [FakePort([encode("IMU", 1, "1.00", "2.00", "90.00", 1, 0)]), FakePort([encode("IMU", 2, "1.00", "2.00", "270.00", 1, 0)])]
        uart = Flaky("fake", 115200, StateStore(), stop, open_port=lambda: ports.pop(0) if ports else FakePort([]), reopen_s=0.01)
        with self.assertLogs("modules.uart_comm", level="ERROR"):
            uart.start()
            deadline = monotonic() + 2.0
            while uart._state.snapshot().imu.yaw_deg != 270.0 and monotonic() < deadline:
                sleep(0.005)
            stop.set()
            uart.join(timeout=1.0)
        self.assertEqual(uart._state.snapshot().imu.yaw_deg, 270.0)
        self.assertFalse(stop.is_set() and uart.is_alive())


if __name__ == "__main__":
    unittest.main()

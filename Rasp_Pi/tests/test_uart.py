import threading
import unittest

try:
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

    def test_a_bad_lnk_counts_as_an_error(self):
        self.feed("LNK", 10, 1, "x", 0)
        self.assertEqual(self.uart.rx_errors, 1)


if __name__ == "__main__":
    unittest.main()

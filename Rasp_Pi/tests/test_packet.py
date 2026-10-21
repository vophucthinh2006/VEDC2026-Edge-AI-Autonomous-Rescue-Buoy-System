import unittest

from utils.nmea_packet import FIX_TYPE_3D, decode, encode, gga_fix_type


class GgaFixTest(unittest.TestCase):
    def test_measured_positions_are_a_fix_estimates_are_not(self) -> None:
        self.assertEqual([gga_fix_type(q) for q in range(10)], [0, 3, 3, 3, 3, 3, 0, 0, 0, 0])
        self.assertEqual(gga_fix_type(1), FIX_TYPE_3D)   # a plain GPS fix, the usual one


class PacketTest(unittest.TestCase):
    def test_round_trip(self) -> None:
        packet = decode(encode("NAV", 7, "AUTO", "0.20", "90.0", 300))
        self.assertIsNotNone(packet)
        assert packet is not None
        self.assertEqual(packet.command, "NAV")
        self.assertEqual(packet.fields[0], "7")

    def test_reject_bad_checksum(self) -> None:
        self.assertIsNone(decode(b"$NAV,1*00\n"))

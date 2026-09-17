import unittest

from utils.nmea_packet import decode, encode


class PacketTest(unittest.TestCase):
    def test_round_trip(self) -> None:
        packet = decode(encode("NAV", 7, "AUTO", "0.20", "90.0", 300))
        self.assertIsNotNone(packet)
        assert packet is not None
        self.assertEqual(packet.command, "NAV")
        self.assertEqual(packet.fields[0], "7")

    def test_reject_bad_checksum(self) -> None:
        self.assertIsNone(decode(b"$NAV,1*00\n"))

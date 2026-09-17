import unittest

from modules.lidar_ld14 import HEADER, LD14Decoder, PACKET_SIZE, crc8


class LD14Test(unittest.TestCase):
    def test_valid_packet_decodes_twelve_points(self) -> None:
        packet = bytearray(PACKET_SIZE)
        packet[:2] = HEADER
        packet[2:4] = (360).to_bytes(2, "little")
        packet[4:6] = (1000).to_bytes(2, "little")
        for i in range(12):
            offset = 6 + 3 * i
            packet[offset:offset + 2] = (2000).to_bytes(2, "little")
            packet[offset + 2] = 100
        packet[42:44] = (2100).to_bytes(2, "little")
        packet[-1] = crc8(packet[:-1])
        points = LD14Decoder().feed(bytes(packet))
        self.assertEqual(len(points), 12)
        self.assertAlmostEqual(points[0].distance_m, 2.0)
        self.assertAlmostEqual(points[-1].angle_deg, 21.0)

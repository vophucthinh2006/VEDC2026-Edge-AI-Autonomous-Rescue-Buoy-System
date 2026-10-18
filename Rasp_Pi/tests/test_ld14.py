import unittest

from modules.lidar_ld14 import HEADER, LD14Decoder, PACKET_SIZE, ScanAssembler, crc8, encode_scan


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

    def test_an_encoded_revolution_decodes_back(self) -> None:
        # 384 samples a turn (2300 Hz / 6 Hz), a wall at 2 m from 90 to 135 deg, nothing elsewhere.
        distances = [2.0 if 90.0 <= i * 360.0 / 384 <= 135.0 else None for i in range(384)]
        stream = encode_scan(distances) * 2
        self.assertEqual(len(stream), 2 * 32 * PACKET_SIZE)
        assembler, decoder, scan = ScanAssembler(), LD14Decoder(), None
        for start in range(0, len(stream), 100):        # the serial port hands it over in pieces
            scan = assembler.add(decoder.feed(stream[start:start + 100])) or scan
        self.assertIsNotNone(scan)
        self.assertEqual(len(scan), 384)
        wall = [p for p in scan if p.intensity > 0]
        self.assertTrue(all(abs(p.distance_m - 2.0) < 0.001 and 89.0 <= p.angle_deg <= 136.0 for p in wall))
        self.assertEqual(len(wall), sum(d is not None for d in distances))

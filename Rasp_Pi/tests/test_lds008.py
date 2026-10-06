import unittest

from modules.lidar_lds008 import LDS008Decoder, PACKET_SIZE, checksum


def make_packet(index: int, distances_mm: list[int], invalid: tuple[int, ...] = ()) -> bytes:
    packet = bytearray(PACKET_SIZE)
    packet[0], packet[1] = 0xFA, 0xA0 + index
    packet[2:4] = (300 * 64).to_bytes(2, "little")
    for i, distance in enumerate(distances_mm):
        offset = 4 + 4 * i
        packet[offset:offset + 2] = distance.to_bytes(2, "little")
        if i in invalid:
            packet[offset + 1] |= 0x80
        packet[offset + 2:offset + 4] = (120).to_bytes(2, "little")
    packet[20:22] = checksum(packet).to_bytes(2, "little")
    return bytes(packet)


class LDS008Test(unittest.TestCase):
    def test_valid_packet_decodes_four_points(self) -> None:
        points = LDS008Decoder().feed(make_packet(5, [1000, 1500, 2000, 2500]))
        self.assertEqual([point.angle_deg for point in points], [20.0, 21.0, 22.0, 23.0])
        self.assertAlmostEqual(points[1].distance_m, 1.5)
        self.assertEqual(points[0].intensity, 120)

    def test_invalid_and_zero_readings_are_dropped(self) -> None:
        points = LDS008Decoder().feed(make_packet(0, [1000, 53, 0, 2500], invalid=(1,)))
        self.assertEqual([point.angle_deg for point in points], [0.0, 3.0])

    def test_resynchronises_after_noise_and_split_input(self) -> None:
        decoder = LDS008Decoder()
        stream = b"\x00\xfa\x11" + make_packet(1, [800] * 4) + make_packet(2, [900] * 4)
        points = decoder.feed(stream[:17]) + decoder.feed(stream[17:])
        self.assertEqual(len(points), 8)
        self.assertEqual(points[4].angle_deg, 8.0)

    def test_bad_checksum_is_rejected(self) -> None:
        packet = bytearray(make_packet(3, [1000] * 4))
        packet[5] ^= 0x01
        decoder = LDS008Decoder()
        self.assertEqual(decoder.feed(bytes(packet)), [])
        self.assertGreater(decoder.bad_checksums, 0)

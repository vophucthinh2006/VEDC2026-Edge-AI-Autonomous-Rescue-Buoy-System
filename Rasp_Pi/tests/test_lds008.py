import unittest

from modules.lidar_lds008 import LDS008Decoder, PACKET_SIZE, checksum, encode_packet, encode_scan


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

    def test_invalid_and_zero_readings_come_out_as_nothing_seen(self) -> None:
        points = LDS008Decoder().feed(make_packet(0, [1000, 53, 0, 2500], invalid=(1,)))
        self.assertEqual([point.angle_deg for point in points], [0.0, 1.0, 2.0, 3.0])
        self.assertEqual([(point.distance_m, point.intensity) for point in points[1:3]], [(0.0, 0), (0.0, 0)])
        self.assertEqual([point.distance_m for point in points[::3]], [1.0, 2.5])

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


class LDS008EncodeTest(unittest.TestCase):
    """The simulator's emulator sends what encode_scan builds; the decoder must read it back."""

    def test_packet_matches_the_hand_built_one(self) -> None:
        self.assertEqual(encode_packet(5, [1.0, 1.5, 2.0, 2.5]), make_packet(5, [1000, 1500, 2000, 2500]))

    def test_a_revolution_survives_the_serial_path(self) -> None:
        distances: list[float | None] = [None] * 360
        for degree in range(350, 360):
            distances[degree] = 3.0
        for degree in range(0, 11):
            distances[degree] = 3.0
        distances[90] = 7.999
        points = [point for point in LDS008Decoder().feed(encode_scan(distances)) if point.intensity > 0]
        self.assertEqual(len(points), 22)
        self.assertEqual({point.angle_deg for point in points}, {float(d) for d in list(range(350, 360)) + list(range(0, 11)) + [90]})
        self.assertAlmostEqual(next(p.distance_m for p in points if p.angle_deg == 90.0), 7.999)

    def test_two_revolutions_make_one_finished_scan(self) -> None:
        from modules.lidar_ld14 import ScanAssembler
        decoder, assembler = LDS008Decoder(), ScanAssembler()
        stream = encode_scan([2.0] * 360) * 2
        done = [scan for chunk in (stream[i:i + 256] for i in range(0, len(stream), 256)) if (scan := assembler.add(decoder.feed(chunk)))]
        self.assertEqual(len(done), 1)
        self.assertEqual(len(done[0]), 360)

    def test_open_water_still_gives_revolutions_with_no_obstacle_in_them(self) -> None:
        """Nothing in range all around: the scans must keep coming (the obstacle feed stops, and the
        autopilot reports its proximity sensor failed, when they do not), and hold no obstacle."""
        from modules.lidar_ld14 import ScanAssembler
        from modules.lidar_serial import to_boat_frame
        from modules.perception import cluster_scan, ray_obstacles
        decoder, assembler = LDS008Decoder(), ScanAssembler()
        scans = [scan for _ in range(3) if (scan := assembler.add(decoder.feed(encode_scan([None] * 360))))]
        self.assertEqual(len(scans), 2)
        scan = to_boat_frame(scans[0], 0.0, False)
        self.assertEqual(ray_obstacles([(p.angle_deg, p.distance_m) for p in scan if p.intensity > 0], 0.15, 8.0), ())
        self.assertEqual(cluster_scan(scan), ())

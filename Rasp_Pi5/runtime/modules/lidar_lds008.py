"""LDS-008 serial packet decoder, written from the LDS-006 / Neato XV-11 format.

There is no datasheet for the LDS-008. Its sibling LDS-006 sends 22-byte packets at 115200 baud:
0xFA, index 0xA0..0xF9 (90 packets, 4 degrees each), motor speed, four readings, checksum; and
only spins after `startlds$`. Check a capture from the real module with tools/lidar_capture.py
before trusting this.
"""
from __future__ import annotations

from modules.lidar_ld14 import LidarPoint

PACKET_SIZE = 22
START = 0xFA
INDEX_FIRST, INDEX_LAST = 0xA0, 0xF9
START_COMMAND = b"startlds$"
STOP_COMMAND = b"stoplds$"


def checksum(data: bytes) -> int:
    """XV-11 checksum over the first 20 bytes, taken as ten little-endian words."""
    value = 0
    for i in range(0, 20, 2):
        value = (value << 1) + int.from_bytes(data[i:i + 2], "little")
    return ((value & 0x7FFF) + (value >> 15)) & 0x7FFF


class LDS008Decoder:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_checksums = 0

    def feed(self, data: bytes) -> list[LidarPoint]:
        self._buffer.extend(data)
        points: list[LidarPoint] = []
        while True:
            start = self._buffer.find(START)
            if start < 0:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < PACKET_SIZE:
                break
            packet = bytes(self._buffer[:PACKET_SIZE])
            if not INDEX_FIRST <= packet[1] <= INDEX_LAST or checksum(packet) != int.from_bytes(packet[20:22], "little"):
                # 0xFA also turns up inside a packet: step one byte and look for the next start.
                self.bad_checksums += 1
                del self._buffer[:1]
                continue
            del self._buffer[:PACKET_SIZE]
            base_deg = (packet[1] - INDEX_FIRST) * 4
            for i in range(4):
                reading = packet[4 + 4 * i:8 + 4 * i]
                # The invalid-data flag (the distance bytes then hold an error code) and a zero
                # distance are both "nothing seen at this angle". They are passed on as distance 0,
                # intensity 0 rather than dropped: over open water every reading is like that, and
                # a revolution with no points at all could not be told from a LiDAR that has stopped.
                distance_mm = 0 if reading[1] & 0x80 else reading[0] | (reading[1] & 0x3F) << 8
                intensity = min(255, int.from_bytes(reading[2:4], "little")) if distance_mm else 0
                points.append(LidarPoint(float(base_deg + i), distance_mm / 1000.0, intensity))
        return points


def encode_packet(index: int, distances_m: list[float | None], rpm: float = 300.0, intensity: int = 120) -> bytes:
    """One packet as the module sends it: four readings from 4 * index degrees on. A distance of
    None is sent with the invalid-data flag. For the simulator (sim/tools/lds008_emulator.py) and the tests."""
    packet = bytearray(PACKET_SIZE)
    packet[0], packet[1] = START, INDEX_FIRST + index
    packet[2:4] = int(rpm * 64).to_bytes(2, "little")
    for i, distance in enumerate(distances_m):
        offset = 4 + 4 * i
        if distance is None:
            packet[offset + 1] = 0x80
            continue
        packet[offset:offset + 2] = max(1, min(0x3FFF, round(distance * 1000.0))).to_bytes(2, "little")
        packet[offset + 2:offset + 4] = intensity.to_bytes(2, "little")
    packet[20:22] = checksum(packet).to_bytes(2, "little")
    return bytes(packet)


def encode_scan(distances_m: list[float | None]) -> bytes:
    """A whole revolution: 360 distances, one per degree from the module's zero, as 90 packets."""
    return b"".join(encode_packet(index, distances_m[4 * index:4 * index + 4]) for index in range(INDEX_LAST - INDEX_FIRST + 1))

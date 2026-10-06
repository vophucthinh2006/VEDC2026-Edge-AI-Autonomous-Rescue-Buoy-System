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
                if reading[1] & 0x80:   # invalid-data flag; the distance bytes then hold an error code
                    continue
                distance_mm = reading[0] | (reading[1] & 0x3F) << 8
                if distance_mm == 0:
                    continue
                points.append(LidarPoint(float(base_deg + i), distance_mm / 1000.0, min(255, int.from_bytes(reading[2:4], "little"))))
        return points

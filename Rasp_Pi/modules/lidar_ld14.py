"""LDROBOT LD14 serial packet decoder (0x54, 0x2C, 12 points per packet)."""
from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

PACKET_SIZE = 47
HEADER = b"\x54\x2c"


@dataclass(frozen=True)
class LidarPoint:
    angle_deg: float  # LD14 angle: clockwise from its mechanical zero
    distance_m: float
    intensity: int


def crc8(data: bytes) -> int:
    """LDROBOT CRC-8, polynomial 0x4D, initial value 0."""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x4D) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


class LD14Decoder:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[LidarPoint]:
        self._buffer.extend(data)
        points: list[LidarPoint] = []
        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                del self._buffer[:-1]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < PACKET_SIZE:
                break
            packet = bytes(self._buffer[:PACKET_SIZE])
            del self._buffer[:PACKET_SIZE]
            if crc8(packet[:-1]) != packet[-1]:
                continue
            start_angle = int.from_bytes(packet[4:6], "little") / 100.0
            end_angle = int.from_bytes(packet[42:44], "little") / 100.0
            delta = (end_angle - start_angle) % 360.0 / 11.0
            for i in range(12):
                offset = 6 + 3 * i
                distance = int.from_bytes(packet[offset:offset + 2], "little") / 1000.0
                points.append(LidarPoint((start_angle + i * delta) % 360.0, distance, packet[offset + 2]))
        return points


class ScanAssembler:
    """Accumulates one revolution; a wrap in angle marks a finished scan."""
    def __init__(self) -> None:
        self._points: list[LidarPoint] = []
        self._last_angle: float | None = None

    def add(self, incoming: list[LidarPoint]) -> tuple[LidarPoint, ...] | None:
        completed = None
        for point in incoming:
            if self._last_angle is not None and point.angle_deg + 180.0 < self._last_angle and self._points:
                completed = tuple(self._points)
                self._points = []
            self._points.append(point)
            self._last_angle = point.angle_deg
        return completed

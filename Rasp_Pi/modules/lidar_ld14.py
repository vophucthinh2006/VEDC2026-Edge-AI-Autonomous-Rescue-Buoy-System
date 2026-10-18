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


POINTS_PER_PACKET = 12


def encode_packet(start_deg: float, end_deg: float, points: list[tuple[float, int]], speed_dps: int = 2160,
                  timestamp_ms: int = 0) -> bytes:
    """One 47-byte packet: header, speed (deg/s), start angle, 12 x (distance mm, intensity), end
    angle (both 0.01 deg, clockwise), timestamp (ms), CRC-8. For the simulator's stand-in and tests."""
    assert len(points) == POINTS_PER_PACKET
    body = bytearray(HEADER)
    body += int(speed_dps).to_bytes(2, "little")
    body += (round(start_deg * 100) % 36000).to_bytes(2, "little")
    for distance_m, intensity in points:
        body += min(0xFFFF, max(0, round(distance_m * 1000))).to_bytes(2, "little") + bytes([intensity & 0xFF])
    body += (round(end_deg * 100) % 36000).to_bytes(2, "little")
    body += (timestamp_ms % 30000).to_bytes(2, "little")
    return bytes(body) + bytes([crc8(body)])


def encode_scan(distances_m: list[float | None], scan_hz: float = 6.0, timestamp_ms: int = 0) -> bytes:
    """A whole revolution, len(distances_m) evenly spaced samples clockwise from the module's zero
    (a multiple of 12; the LD14 makes about 2300 / 6 = 384), as packets. None: no return, sent as 0 mm."""
    count = len(distances_m)
    step = 360.0 / count
    packets = []
    for first in range(0, count - count % POINTS_PER_PACKET, POINTS_PER_PACKET):
        chunk = [(d if d is not None else 0.0, 200 if d is not None else 0) for d in distances_m[first:first + POINTS_PER_PACKET]]
        packets.append(encode_packet(first * step, (first + POINTS_PER_PACKET - 1) * step, chunk,
                                     round(360.0 * scan_hz), timestamp_ms))
    return b"".join(packets)


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

"""Bounded NMEA-like packet codec used only on the Pi<->STM32 UART."""
from __future__ import annotations

from dataclasses import dataclass

MAX_LINE_BYTES = 256

# The STM32F407's $GPS carries the GGA fix quality as the receiver sends it (Modules/GPS/gps_nmea.c):
# 0 none, 1 GPS, 2 DGPS/SBAS, 3 PPS, 4 RTK fixed, 5 RTK float, 6 dead reckoning, 7 manual, 8 simulator.
# The rest of the Pi code reads GpsState.fix as MAVLink's fix type (0/1 none, 2 2D, 3 3D), so a plain
# GPS fix (quality 1) was taken for "no fix": safety.py stopped every waypoint (gps_stale_or_no_fix)
# and only a receiver catching SBAS (quality 2) let the boat move.
GGA_POSITION_QUALITIES = range(1, 6)   # 6..8 are estimates or typed in, not a position the receiver measured
FIX_TYPE_3D = 3


def gga_fix_type(quality: int) -> int:
    """GGA fix quality -> MAVLink fix type: a measured position counts as a 3D fix (GGA does not
    say 2D or 3D), anything else as none."""
    return FIX_TYPE_3D if quality in GGA_POSITION_QUALITIES else 0


@dataclass(frozen=True)
class Packet:
    command: str
    fields: tuple[str, ...]


def checksum(payload: str) -> str:
    value = 0
    for char in payload:
        value ^= ord(char)
    return f"{value:02X}"


def encode(command: str, *fields: object) -> bytes:
    command = command.upper()
    if not command.isalnum():
        raise ValueError("command must be alphanumeric")
    safe_fields = [str(field).replace(",", ";").replace("*", "").replace("\n", "") for field in fields]
    payload = ",".join((command, *safe_fields))
    raw = f"${payload}*{checksum(payload)}\n".encode("ascii")
    if len(raw) > MAX_LINE_BYTES:
        raise ValueError("packet exceeds UART line limit")
    return raw


def decode(raw: bytes | str) -> Packet | None:
    if isinstance(raw, bytes):
        if len(raw) > MAX_LINE_BYTES:
            return None
        try:
            text = raw.decode("ascii", "strict")
        except UnicodeDecodeError:
            return None
    else:
        text = raw
    text = text.strip()
    if not text.startswith("$") or text.count("*") != 1:
        return None
    body, received = text[1:].split("*", 1)
    if len(received) != 2 or checksum(body) != received.upper():
        return None
    parts = body.split(",")
    if not parts or not parts[0].isalnum():
        return None
    return Packet(parts[0].upper(), tuple(parts[1:]))

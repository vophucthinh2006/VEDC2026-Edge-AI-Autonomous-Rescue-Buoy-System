"""Bounded NMEA-like packet codec used only on the Pi<->STM32 UART."""
from __future__ import annotations

from dataclasses import dataclass

MAX_LINE_BYTES = 256


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

"""The sole owner of the Pi<->STM32 serial port."""
from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass
from time import monotonic

import serial

from modules.state_store import StateStore
from utils.nmea_packet import Packet, decode, encode


@dataclass(frozen=True)
class Outbound:
    command: str
    fields: tuple[object, ...]


class Stm32Uart(threading.Thread):
    def __init__(self, port: str, baud: int, state: StateStore, stop_event: threading.Event) -> None:
        super().__init__(name="stm32-uart", daemon=True)
        self._port, self._baud, self._state, self._stop = port, baud, state, stop_event
        self.outbound: queue.Queue[Outbound] = queue.Queue(maxsize=100)
        self.log = logging.getLogger(__name__)
        self.rx_errors = 0

    def send(self, command: str, *fields: object) -> None:
        try:
            self.outbound.put_nowait(Outbound(command, fields))
        except queue.Full:
            self.log.warning("UART transmit queue full; dropping %s", command)

    def _handle(self, packet: Packet) -> None:
        try:
            if packet.command == "IMU" and len(packet.fields) >= 6:
                _, pitch, roll, yaw, ok, overturned = packet.fields[:6]
                self._state.update_imu(float(pitch), float(roll), float(yaw), bool(int(ok)), bool(int(overturned)))
            elif packet.command == "GPS" and len(packet.fields) >= 7:
                _, lat, lon, fix, hdop, speed, course = packet.fields[:7]
                self._state.update_gps(float(lat), float(lon), int(fix), float(hdop), float(speed), float(course))
            elif packet.command == "SYS" and len(packet.fields) >= 5:
                _, battery, motor_fault, estop, link_ok = packet.fields[:5]
                self._state.update_system(float(battery), bool(int(motor_fault)), bool(int(estop)), bool(int(link_ok)))
            elif packet.command == "LRA" and len(packet.fields) >= 3 and packet.fields[1] == "WAYPOINT":
                # Format: LRA,seq,WAYPOINT,lat;lon - semicolon prevents CSV ambiguity.
                lat, lon = packet.fields[2].split(";")
                self._state.set_waypoint(float(lat), float(lon))
                self.log.info("received waypoint %.6f, %.6f", float(lat), float(lon))
        except (ValueError, IndexError) as exc:
            self.rx_errors += 1
            self.log.warning("invalid %s packet: %s", packet.command, exc)

    def run(self) -> None:
        try:
            with serial.Serial(self._port, self._baud, timeout=0.05, write_timeout=0.2) as ser:
                self.log.info("STM32 UART open: %s @ %d", self._port, self._baud)
                while not self._stop.is_set():
                    raw = ser.readline()
                    if raw:
                        packet = decode(raw)
                        if packet is None:
                            self.rx_errors += 1
                        else:
                            self._handle(packet)
                    try:
                        item = self.outbound.get_nowait()
                    except queue.Empty:
                        continue
                    ser.write(encode(item.command, *item.fields))
        except serial.SerialException as exc:
            self.log.error("STM32 UART unavailable: %s", exc)
            self._stop.set()

"""The sole owner of the Pi<->STM32 serial port.

A port that fails (the USB-to-TTL adapter dropping off the bus when the motors start, a write that
times out) is closed and opened again every REOPEN_S; it used to stop the whole program, and the
boat stood still until main.py was started again by hand. Meanwhile nothing is sent: commands
queued while the port is down would arrive stale, and the F407 stops the motors by itself once
$NAV and $HBT go silent."""
from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass
from time import monotonic
from typing import Callable

import serial

REOPEN_S = 1.0

from modules.lora_route import LoraRoute, RouteAssembler
from modules.state_store import StateStore
from utils.nmea_packet import Packet, decode, encode, gga_fix_type


@dataclass(frozen=True)
class Outbound:
    command: str
    fields: tuple[object, ...]


class Stm32Uart(threading.Thread):
    def __init__(self, port: str, baud: int, state: StateStore, stop_event: threading.Event,
                 open_port: Callable[[], object] | None = None, reopen_s: float = REOPEN_S) -> None:
        """open_port: returns the opened port (a context manager with readline and write); the
        tests pass a stand-in."""
        super().__init__(name="stm32-uart", daemon=True)
        self._port, self._baud, self._state, self._stop_event = port, baud, state, stop_event
        self._open = open_port or (lambda: serial.Serial(self._port, self._baud, timeout=0.02, write_timeout=0.2))
        self._reopen_s = reopen_s
        self.outbound: queue.Queue[Outbound] = queue.Queue(maxsize=100)
        self.log = logging.getLogger(__name__)
        self.rx_errors = 0
        self.connected = False
        self.drops = 0            # times the port failed after it had been open
        # Route sent from the dashboard over LoRa (modules/lora_route.py): the last complete one,
        # for the main loop to take; it reports lora_route.rev back in $HBT.
        self._routes = RouteAssembler()
        self.lora_route: LoraRoute | None = None

    def send(self, command: str, *fields: object) -> None:
        if not self.connected:
            return   # stale by the time the port is back; the F407 has stopped the motors meanwhile
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
                _, lat, lon, quality, hdop, speed, course = packet.fields[:7]
                # GGA fix quality from the F407, as the fix type the rest of the Pi code expects.
                self._state.update_gps(float(lat), float(lon), gga_fix_type(int(quality)), float(hdop), float(speed), float(course))
            elif packet.command == "SYS" and len(packet.fields) >= 5:
                _, battery, motor_fault, estop, link_ok = packet.fields[:5]
                # Firmware from 2026-10-17 adds the presses of the victim's confirm button (PE4),
                # later firmware the battery's charge in percent (-1 unknown); battery is then volts.
                presses = int(packet.fields[7]) if len(packet.fields) >= 8 else -1
                percent = int(packet.fields[8]) if len(packet.fields) >= 9 and packet.fields[8] != "" else -1
                self._state.update_system(float(battery), bool(int(motor_fault)), bool(int(estop)), bool(int(link_ok)), presses, percent)
                if len(packet.fields) >= 7:
                    # Newer firmware adds armed and the RC mode switch; the camera's search sweep
                    # (camera.scan.modes) runs on them as it does on the autopilot's mode.
                    armed, auto = bool(int(packet.fields[5])), bool(int(packet.fields[6]))
                    self._state.update_vehicle("AUTO" if auto else "MANUAL", armed, 0)
            elif packet.command == "LNK" and len(packet.fields) >= 4:
                # $LNK,seq,shore_ok,heard_ago_s,home_valid,home_lat,home_lon[,hold], once a second: the
                # shore station's link as the F407 hears it over LoRa, the station's position (home) and
                # whether the admin stopped the boat from the dashboard (the F407 then keeps the motors off).
                _, shore_ok, heard_ago, home_valid = packet.fields[:4]
                home = None
                if int(home_valid) and len(packet.fields) >= 6 and packet.fields[4] and packet.fields[5]:
                    home = (float(packet.fields[4]), float(packet.fields[5]))
                hold = len(packet.fields) >= 7 and packet.fields[6] == "1"
                self._state.update_link(bool(int(shore_ok)), float(heard_ago) if heard_ago else -1.0, home, hold)
            elif packet.command == "RTE":
                route = self._routes.feed(packet.fields)
                if route is not None:
                    self.lora_route = route
                    self.log.warning("route rev %d from the dashboard: %d points%s", route.rev, len(route.points), ", loop" if route.loop else "")
            elif packet.command == "LRA" and len(packet.fields) >= 3 and packet.fields[1] == "WAYPOINT":
                # Format: LRA,seq,WAYPOINT,lat;lon - semicolon prevents CSV ambiguity.
                lat, lon = packet.fields[2].split(";")
                self._state.set_waypoint(float(lat), float(lon))
                self.log.info("received waypoint %.6f, %.6f", float(lat), float(lon))
        except (ValueError, IndexError) as exc:
            self.rx_errors += 1
            self.log.warning("invalid %s packet: %s", packet.command, exc)

    def _discard_outbound(self) -> None:
        while True:
            try:
                self.outbound.get_nowait()
            except queue.Empty:
                return

    def _serve(self, ser: object) -> None:
        while not self._stop_event.is_set():
            raw = ser.readline()
            if raw:
                packet = decode(raw)
                if packet is None:
                    self.rx_errors += 1
                else:
                    self._handle(packet)
            # Drain everything queued: one packet per loop cannot keep up with NAV+HBT+CAM.
            while True:
                try:
                    item = self.outbound.get_nowait()
                except queue.Empty:
                    break
                ser.write(encode(item.command, *item.fields))

    def run(self) -> None:
        lost_at: float | None = None      # when the port went, None while it works
        opened = False
        while not self._stop_event.is_set():
            try:
                with self._open() as ser:
                    if lost_at is None:
                        self.log.info("STM32 UART open: %s @ %d", self._port, self._baud)
                    else:
                        self.log.warning("STM32 UART back after %.1f s: %s", monotonic() - lost_at, self._port)
                    lost_at, opened = None, True
                    self._discard_outbound()
                    self.connected = True
                    self._serve(ser)
            except (serial.SerialException, OSError) as exc:
                self._lost(exc, opened, lost_at)
            except Exception as exc:  # noqa: BLE001 - a bug in a packet handler must not end the link for good
                self.log.exception("STM32 UART thread failed")
                self._lost(exc, opened, lost_at)
            finally:
                self.connected = False
            if lost_at is None and not self._stop_event.is_set():
                lost_at = monotonic()
            self._discard_outbound()
            self._stop_event.wait(self._reopen_s)

    def _lost(self, exc: Exception, opened: bool, lost_at: float | None) -> None:
        if lost_at is not None:
            return   # still down: said once already
        if opened:
            self.drops += 1
        self.log.error("STM32 UART %s: %s; trying again every %.0f s, the F407 stops the motors meanwhile",
                       "lost" if opened else "unavailable", exc, self._reopen_s)

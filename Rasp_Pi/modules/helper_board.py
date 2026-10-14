"""The STM32F103 helper board of the Pixhawk boat, on a USB-to-serial adapter (/dev/ttyUSB0).

The autopilot drives the boat; this board does what hangs off the Pi: the camera servo, the alarm
relays, the E-stop button, the battery monitor and the LoRa beacon to the shore station. Same line
framing as the STM32F407 boat (utils/nmea_packet.py); the packets are listed in
STM32F103/Modules/PiLink/pi_link.h.

The board is an extra: without it, or with the adapter pulled out, the rescue program runs on. The
port is opened again every few seconds until it is there.
"""
from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass
from time import monotonic

import serial

from modules.state_store import Snapshot
from utils.nmea_packet import decode, encode

AUTO_MODES = {"AUTO", "GUIDED", "RTL", "SMART_RTL", "LOITER", "FOLLOW", "DOCK", "CIRCLE"}   # as esp32-lora-station/tools/mavlink_bridge.py
STOP_MODES = {"HOLD", "INITIALISING"}
REOPEN_S = 3.0
STALE_S = 2.0


@dataclass(frozen=True)
class BoardState:
    battery_v: float = 0.0
    current_a: float = 0.0
    estop: bool = False
    lora_ok: bool = False
    alarm: bool = False
    timestamp: float = 0.0       # 0: nothing heard yet


def position_fields(snap: Snapshot, now: float) -> tuple[str, ...]:
    """What the LoRa beacon carries, as the POS packet's fields: fix, lat, lon, roll, pitch, yaw,
    target yaw, mode, imu_ok, calib. The board forwards the numbers as the text written here, so
    the widths matter: one decimal for angles, six for the position, yaw never "360.0"."""
    vehicle, imu, gps = snap.vehicle, snap.imu, snap.gps
    fix = gps.fix >= 3 and now - gps.timestamp <= STALE_S and abs(gps.lat_deg) <= 90.0 and abs(gps.lon_deg) <= 180.0
    imu_ok = imu.imu_ok and now - imu.timestamp <= STALE_S
    steering = vehicle.armed and vehicle.mode in AUTO_MODES
    mode = "A" if steering else "S" if not vehicle.armed or vehicle.mode in STOP_MODES else "M"
    yaw = round(imu.yaw_deg * 10.0) % 3600 / 10.0
    # ATTITUDE gives roll and pitch to +-180 and +-90; clamp what a glitch could push past the frame's width.
    roll, pitch = max(-180.0, min(180.0, imu.roll_deg)), max(-90.0, min(90.0, imu.pitch_deg))
    return ("1" if fix else "0", f"{gps.lat_deg if fix else 0.0:.6f}", f"{gps.lon_deg if fix else 0.0:.6f}", f"{roll:.1f}", f"{pitch:.1f}",
            f"{yaw:.1f}", "-1.0",                     # the autopilot's target heading is not read here
            mode, "1" if imu_ok else "0",
            "3" if fix and imu_ok else "0")           # no BNO055 here: 3 means attitude and position are both there


class HelperBoard(threading.Thread):
    def __init__(self, port: str, baud: int, stop_event: threading.Event) -> None:
        super().__init__(name="helper", daemon=True)
        self._port, self._baud, self._stop_event = port, baud, stop_event
        self._outbound: queue.Queue[bytes] = queue.Queue(maxsize=50)
        self._sequence = 0
        self._state = BoardState()
        self.log = logging.getLogger(__name__)

    @property
    def state(self) -> BoardState:
        return self._state

    def online(self, now: float) -> bool:
        return self._state.timestamp > 0.0 and now - self._state.timestamp <= STALE_S

    def _send(self, command: str, *fields: object) -> None:
        self._sequence = (self._sequence + 1) & 0xFFFF
        try:
            self._outbound.put_nowait(encode(command, self._sequence, *fields))
        except queue.Full:
            pass            # the port is away: what is queued is stale anyway, the next second sends it all again

    def send_pan(self, pan_deg: float) -> None:
        """Camera servo angle from the bow, positive right."""
        self._send("CAM", f"{max(-90.0, min(90.0, pan_deg)):.1f}")

    def report(self, snap: Snapshot, now: float, victims: int, phase: str) -> None:
        """Once a second: the Pi is alive, how many people it has found, where the boat is."""
        self._send("HBT", phase)
        self._send("VIC", max(0, min(255, victims)))
        self._send("POS", *position_fields(snap, now))

    def _handle(self, raw: bytes) -> None:
        packet = decode(raw)
        if packet is None or packet.command != "AUX" or len(packet.fields) < 6:
            return
        try:
            _, battery_mv, current_ma, estop, lora_ok, alarm = packet.fields[:6]
            self._state = BoardState(int(battery_mv) / 1000.0, int(current_ma) / 1000.0, bool(int(estop)), bool(int(lora_ok)),
                                     bool(int(alarm)), monotonic())
        except ValueError:
            self.log.warning("invalid AUX packet: %r", raw)

    def run(self) -> None:
        told = False
        while not self._stop_event.is_set():
            try:
                with serial.Serial(self._port, self._baud, timeout=0.02, write_timeout=0.2) as ser:
                    self.log.info("helper board UART open: %s @ %d", self._port, self._baud)
                    told = False
                    while not self._stop_event.is_set():
                        raw = ser.readline()
                        if raw:
                            self._handle(raw)
                        while True:
                            try:
                                ser.write(self._outbound.get_nowait())
                            except queue.Empty:
                                break
            except (serial.SerialException, OSError) as exc:
                if not told:
                    told = True
                    self.log.warning("helper board unavailable on %s, running without it: %s", self._port, exc)
                self._stop_event.wait(REOPEN_S)

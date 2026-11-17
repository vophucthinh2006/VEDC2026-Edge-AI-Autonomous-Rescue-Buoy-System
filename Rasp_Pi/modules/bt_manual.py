"""Manual control from an Android phone over Bluetooth Classic SPP (RFCOMM), one character a command:

  V / v   into MANUAL / back to AUTO        F  forward, all three motors at the throttle level
  B       astern, the front motor alone     S  the three motors stop (the rudders stay where they are)
  L / R   rudders one notch left / right    W / w  camera sweeps back and forth / stops where it is
  U / u   lights on / off                   0..9   throttle level, 9 = full range (no 0.5 cap as in AUTO)

Anything else is ignored. In MANUAL the phone's silence (silence_s) or a lost connection stops the
motors and the boat stays in MANUAL: only v gives it back to the mission. The F407 stops by itself
once the Pi's $MAN goes silent (its TTL).

The Pi needs an SPP record for the phone to find channel 1: bluetoothd started with --compat and
`sdptool add --channel=1 SP`, the phone paired once with bluetoothctl."""
from __future__ import annotations

import logging
import socket
import threading
from dataclasses import dataclass, replace
from time import monotonic

STEER_NOTCH = 0.2        # 5 notches a side
NOTCH_GAP_S = 0.15       # an app that repeats L while the button is held turns one notch per gap
LEVELS = 9


@dataclass(frozen=True)
class ManualState:
    active: bool = False
    motion: str = "S"         # F forward, B astern, S stopped
    level: int = 0            # 0..9
    steer: float = 0.0        # -1..1, positive turns the bow right
    sweep: bool = False
    lights: int = 0           # relay mask, 3 = both
    last_rx: float = 0.0
    last_notch: float = float("-inf")

    @property
    def rear(self) -> float:
        return self.level / LEVELS if self.motion == "F" else 0.0

    @property
    def front(self) -> float:
        return {"F": 1.0, "B": -1.0}.get(self.motion, 0.0) * self.level / LEVELS


def apply(state: ManualState, ch: str, now: float) -> ManualState:
    """The state after one character from the phone."""
    state = replace(state, last_rx=now)
    if ch == "V":
        return replace(state, active=True, motion="S", steer=0.0, sweep=False)
    if ch == "v":
        return replace(state, active=False, motion="S", steer=0.0, sweep=False)
    if ch.isdigit():
        return replace(state, level=int(ch))
    if not state.active:
        return state      # in AUTO the phone only switches mode and sets the level
    if ch in "FBS":
        return replace(state, motion=ch)
    if ch in "LR":
        if now - state.last_notch < NOTCH_GAP_S:
            return state
        steer = state.steer + (STEER_NOTCH if ch == "R" else -STEER_NOTCH)
        return replace(state, steer=round(max(-1.0, min(1.0, steer)), 2), last_notch=now)
    if ch in "Ww":
        return replace(state, sweep=ch == "W")
    if ch in "Uu":
        return replace(state, lights=3 if ch == "U" else 0)
    return state


def silenced(state: ManualState, now: float, silence_s: float) -> ManualState:
    """Motors stopped once the phone has said nothing for silence_s; still MANUAL."""
    if state.active and state.motion != "S" and now - state.last_rx > silence_s:
        return replace(state, motion="S")
    return state


class BluetoothManual(threading.Thread):
    def __init__(self, config: dict, stop_event: threading.Event) -> None:
        super().__init__(name="bt-manual", daemon=True)
        self._channel = int(config.get("channel", 1))
        self._silence_s = float(config.get("silence_s", 1.0))
        self._stop_event = stop_event
        self._lock = threading.Lock()
        self._state = ManualState()
        self.log = logging.getLogger(__name__)

    def state(self, now: float | None = None) -> ManualState:
        now = monotonic() if now is None else now
        with self._lock:
            stopped = silenced(self._state, now, self._silence_s)
            if stopped is not self._state:
                self.log.warning("phone silent %.1f s in MANUAL: motors stopped", self._silence_s)
                self._state = stopped
            return self._state

    def _feed(self, data: bytes) -> None:
        now = monotonic()
        with self._lock:
            for ch in data.decode("ascii", "replace"):
                before = self._state
                self._state = apply(before, ch, now)
                if before.active != self._state.active:
                    self.log.warning("phone: %s", "MANUAL" if self._state.active else "back to AUTO")

    def _dropped(self) -> None:
        with self._lock:
            if self._state.active:
                self._state = replace(self._state, motion="S")
                self.log.warning("phone disconnected in MANUAL: motors stopped, waiting for it")

    def run(self) -> None:
        if not hasattr(socket, "AF_BLUETOOTH"):
            self.log.error("no Bluetooth sockets on this system: manual control off")
            return
        try:
            server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            server.bind((getattr(socket, "BDADDR_ANY", "00:00:00:00:00:00"), self._channel))
            server.listen(1)
            server.settimeout(1.0)
        except OSError as exc:
            self.log.error("Bluetooth RFCOMM channel %d: %s; manual control off", self._channel, exc)
            return
        self.log.info("waiting for the phone on RFCOMM channel %d", self._channel)
        with server:
            while not self._stop_event.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    self.log.error("Bluetooth accept: %s", exc)
                    self._stop_event.wait(1.0)
                    continue
                self.log.warning("phone connected: %s", address[0])
                client.settimeout(0.2)
                with client:
                    while not self._stop_event.is_set():
                        try:
                            data = client.recv(64)
                        except socket.timeout:
                            continue
                        except OSError:
                            break
                        if not data:
                            break
                        self._feed(data)
                self._dropped()

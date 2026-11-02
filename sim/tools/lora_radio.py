"""The LoRa link between the simulated STM32F407 (stm32_board_sim.py) and the simulated shore
station (shore_station_sim.py): UDP on 127.0.0.1 standing in for the 433 MHz radio, with the
air time of each frame, and the firmware's own route code (librouteradio.so, built by
tools/hw_standins.sh from STM32F4/App/route_link.c and esp32-lora-station/main/route_downlink.c).

A datagram leaves when the frame has finished on air, which is when the real receiver gets it.
The buoy only takes what arrives while it listens (its 1.5 s window after each beacon), as the
SX1278 hears nothing outside receive mode.

Several boats share the one channel (docs/fleet-control.md): Air sends every frame to every radio on
the channel, the station on BEACON_PORT and boat n on BOAT_PORT_BASE + n - 1, and each receiver drops
what it could not have heard: a frame that overlaps one it was already hearing (two boats talking at
once: the first one is caught, the second lost), and a frame that arrived while it was itself sending
(the radio is half duplex). Each datagram carries the moment its frame left the air, so a receiver that
reads it late still knows when it was on the air.
"""
from __future__ import annotations

import ctypes
import math
import socket
import time
from pathlib import Path

BEACON_PORT = 47101         # buoy -> station
DOWNLINK_PORT = 47102       # station -> buoy (PHAO-01)
BOAT_PORT_BASE = 47102      # boat n listens on BOAT_PORT_BASE + n - 1
MAX_BOATS = 8
AIR_PORTS = (BEACON_PORT, *range(BOAT_PORT_BASE, BOAT_PORT_BASE + MAX_BOATS))
# Radio settings of both ends (STM32F4/App/app_config.h, esp32-lora-station/main/main.c)
SF, BW_HZ, CR, PREAMBLE = 9, 125000, 5, 8


def airtime_s(length: int) -> float:
    """Semtech AN1200.13 time on air: explicit header, CRC on, no low data rate optimisation."""
    symbol = (1 << SF) / BW_HZ
    payload = 8 + max(math.ceil((8 * length - 4 * SF + 28 + 16) / (4 * SF)) * CR, 0)
    return (PREAMBLE + 4.25) * symbol + payload * symbol


def boat_port(buoy_id: str) -> int:
    """The port boat buoy_id listens on: PHAO-03 -> BOAT_PORT_BASE + 2."""
    tail = ""
    for c in reversed(buoy_id):
        if not c.isdigit():
            break
        tail = c + tail
    number = int(tail) if tail else 1
    return BOAT_PORT_BASE + (max(number, 1) - 1) % MAX_BOATS


class Air:
    """One radio on the shared channel: send() puts a frame on it once it is off the air, receive()
    returns what this radio heard, without the frames lost to a collision or to its own sending.
    collisions False: every frame is heard (the channel of a single boat, as before)."""

    def __init__(self, port: int, collisions: bool = True) -> None:
        self.port, self.collisions = port, collisions
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", port))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.others = [p for p in AIR_PORTS if p != port]
        self.heard: list[tuple[float, float]] = []      # (on air from, to) of frames being received
        self.sent: list[tuple[float, float]] = []       # this radio's own transmissions
        self.received = self.collided = self.deaf = 0   # counters for the comparison (sim/tools/fleet_compare.py)

    def send(self, frame: bytes, started: float | None = None) -> None:
        """frame has just left the air (started: when it began, default now minus its air time)."""
        end = time.monotonic()
        start = end - airtime_s(len(frame)) if started is None else started
        self.sent.append((start, end))
        packet = f"{end:.6f}|".encode() + frame
        for port in self.others:
            try:
                self.tx.sendto(packet, ("127.0.0.1", port))
            except OSError:
                pass

    def receive(self, timeout: float | None = 0.0) -> list[bytes]:
        """What reached this radio since the last call; timeout None waits for the first one."""
        self.rx.settimeout(timeout)
        out: list[bytes] = []
        try:
            while True:
                packet = self.rx.recv(512)
                self.rx.settimeout(0.0)
                # "<end>|<frame>"; a sender without the stamp (tools/beacon_buoy_sim.py) sends the bare frame
                stamp, sep, frame = packet.partition(b"|")
                try:
                    end = float(stamp) if sep else time.monotonic()
                except ValueError:
                    end, sep = time.monotonic(), b""
                if not sep:
                    frame = packet
                start = end - airtime_s(len(frame))
                self.received += 1
                if self.collisions and any(s < end and start < e for s, e in self.sent):
                    self.deaf += 1                # it was sending then: half duplex
                elif self.collisions and any(s < end and start < e for s, e in self.heard):
                    self.collided += 1            # another frame was on the air: lost
                else:
                    out.append(frame)
                self.heard.append((start, end))
        except (BlockingIOError, socket.timeout):
            pass
        horizon = time.monotonic() - 5.0
        self.heard = [i for i in self.heard if i[1] > horizon]
        self.sent = [i for i in self.sent if i[1] > horizon]
        return out

    def stats(self) -> dict:
        return {"port": self.port, "received": self.received, "collided": self.collided, "deaf": self.deaf}


class _Lib:
    def __init__(self, path: Path) -> None:
        lib = ctypes.CDLL(str(path))
        lib.sim_route_link_size.restype = ctypes.c_size_t
        lib.sim_route_downlink_size.restype = ctypes.c_size_t
        for name in ("sim_route_link_rev", "sim_route_link_have", "sim_route_link_total"):
            getattr(lib, name).restype = ctypes.c_uint16
        lib.RouteLink_OnFrame.restype = ctypes.c_bool
        lib.RouteLink_OnFrame.argtypes = (ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)
        lib.RouteLink_BeaconField.restype = ctypes.c_size_t
        lib.RouteLink_BeaconField.argtypes = (ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t)
        lib.RouteLink_SetPiRev.argtypes = (ctypes.c_void_p, ctypes.c_uint16)
        lib.RouteLink_NextPiLine.restype = ctypes.c_bool
        lib.RouteLink_NextPiLine.argtypes = (ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint32, ctypes.c_char_p, ctypes.c_size_t)
        lib.RouteDownlink_Frame.restype = ctypes.c_size_t
        lib.RouteDownlink_Frame.argtypes = (ctypes.c_void_p, ctypes.c_char_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t)
        lib.RouteDownlink_ParseAck.restype = ctypes.c_bool
        lib.RouteDownlink_ParseAck.argtypes = (ctypes.c_char_p, ctypes.c_void_p)
        lib.sim_route_downlink_set.restype = ctypes.c_bool
        lib.sim_route_downlink_set.argtypes = (ctypes.c_void_p, ctypes.c_uint16, ctypes.c_char_p, ctypes.c_bool, ctypes.c_uint16,
                                               ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double))
        # App/shore_link.c, App/event_queue.c, the station's ACK and event parser
        lib.sim_shore_link_size.restype = ctypes.c_size_t
        lib.sim_event_queue_size.restype = ctypes.c_size_t
        lib.sim_shore_lights.restype = ctypes.c_uint8
        lib.sim_shore_lights.argtypes = (ctypes.c_void_p,)
        lib.sim_shore_hold.restype = ctypes.c_bool
        lib.sim_shore_hold.argtypes = (ctypes.c_void_p,)
        lib.sim_shore_home.restype = ctypes.c_bool
        lib.sim_shore_home.argtypes = (ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double))
        lib.sim_event_count.restype = ctypes.c_uint8
        lib.sim_event_count.argtypes = (ctypes.c_void_p,)
        lib.sim_event_json.restype = ctypes.c_size_t
        lib.sim_event_json.argtypes = (ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t)
        lib.ShoreLink_Heard.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
        lib.ShoreLink_OnFrame.restype = ctypes.c_bool
        lib.ShoreLink_OnFrame.argtypes = (ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint16))
        lib.ShoreLink_Ok.restype = ctypes.c_bool
        lib.ShoreLink_Ok.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
        lib.ShoreLink_PiLine.restype = ctypes.c_size_t
        lib.ShoreLink_PiLine.argtypes = (ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint32, ctypes.c_char_p, ctypes.c_size_t)
        lib.EventQueue_Init.argtypes = (ctypes.c_void_p,)
        lib.EventQueue_PushVictim.restype = ctypes.c_uint16
        lib.EventQueue_PushVictim.argtypes = (ctypes.c_void_p, ctypes.c_bool, ctypes.c_int32, ctypes.c_int32, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16)
        lib.EventQueue_PushTilt.restype = ctypes.c_uint16
        lib.EventQueue_PushTilt.argtypes = (ctypes.c_void_p, ctypes.c_bool, ctypes.c_int32, ctypes.c_int32, ctypes.c_float, ctypes.c_float)
        lib.EventQueue_PushConfirm.restype = ctypes.c_uint16
        lib.EventQueue_PushConfirm.argtypes = (ctypes.c_void_p, ctypes.c_bool, ctypes.c_int32, ctypes.c_int32, ctypes.c_uint16)
        lib.EventQueue_Frame.restype = ctypes.c_size_t
        lib.EventQueue_Frame.argtypes = (ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t)
        lib.EventQueue_Ack.restype = ctypes.c_bool
        lib.EventQueue_Ack.argtypes = (ctypes.c_void_p, ctypes.c_uint16)
        lib.RouteDownlink_Ack.restype = ctypes.c_size_t
        lib.RouteDownlink_Ack.argtypes = (ctypes.c_char_p, ctypes.c_uint16, ctypes.c_uint8, ctypes.c_bool, ctypes.c_double, ctypes.c_double,
                                          ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t)
        # RETURN and the formation (docs/fleet-control.md), a leader's beacon, the fleet's time slots
        lib.sim_shore_ret.restype = ctypes.c_bool
        lib.sim_shore_ret.argtypes = (ctypes.c_void_p,)
        lib.sim_shore_follow.restype = ctypes.c_bool
        lib.sim_shore_follow.argtypes = (ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_int16),
                                         ctypes.POINTER(ctypes.c_int16))
        lib.ShoreLink_FollowLine.restype = ctypes.c_size_t
        lib.ShoreLink_FollowLine.argtypes = (ctypes.c_void_p, ctypes.c_uint16, ctypes.c_char_p, ctypes.c_size_t)
        lib.sim_beacon_parse.restype = ctypes.c_bool
        lib.sim_beacon_parse.argtypes = (ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_double),
                                         ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                                         ctypes.POINTER(ctypes.c_float))
        lib.LoraSlot_FromId.restype = ctypes.c_uint8
        lib.LoraSlot_FromId.argtypes = (ctypes.c_char_p, ctypes.c_uint8)
        lib.LoraSlot_Wait.restype = ctypes.c_uint32
        lib.LoraSlot_Wait.argtypes = (ctypes.c_uint32, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint32)
        self.lib = lib


class BuoyRoute:
    """route_link_t of the F407: RTE frames in, the beacon's w= field and $RTE lines out."""

    def __init__(self, lib_path: Path) -> None:
        self._lib = _Lib(lib_path).lib
        self._state = ctypes.create_string_buffer(self._lib.sim_route_link_size())   # zeroed = RouteLink_Init

    def on_frame(self, frame: bytes, buoy_id: str) -> bool:
        return bool(self._lib.RouteLink_OnFrame(self._state, frame, buoy_id.encode()))

    def beacon_field(self) -> str:
        buf = ctypes.create_string_buffer(32)
        self._lib.RouteLink_BeaconField(self._state, buf, len(buf))
        return buf.value.decode()

    def set_pi_rev(self, rev: int) -> None:
        self._lib.RouteLink_SetPiRev(self._state, rev & 0xFFFF)

    def next_pi_line(self, seq: int, now_ms: int) -> str | None:
        buf = ctypes.create_string_buffer(180)
        if not self._lib.RouteLink_NextPiLine(self._state, seq & 0xFFFF, now_ms & 0xFFFFFFFF, buf, len(buf)):
            return None
        return buf.value.decode()

    @property
    def rev(self) -> int:
        return self._lib.sim_route_link_rev(self._state)

    @property
    def have(self) -> int:
        return self._lib.sim_route_link_have(self._state)

    @property
    def total(self) -> int:
        return self._lib.sim_route_link_total(self._state)

    @property
    def receiving(self) -> bool:
        return self.rev != 0 and self.have < self.total


class _Ack(ctypes.Structure):
    _fields_ = [("has", ctypes.c_bool), ("rev", ctypes.c_uint16), ("have", ctypes.c_uint16), ("pi", ctypes.c_uint8)]


class FormationRole(ctypes.Structure):
    """formation_role_t of route_downlink.h"""
    _fields_ = [("on", ctypes.c_bool), ("leader", ctypes.c_char * 32), ("right_m", ctypes.c_int16), ("back_m", ctypes.c_int16)]


class StationRoute:
    """route_downlink_t of the ESP32: the route the admin sent, and the RTE frame answering a beacon."""

    def __init__(self, lib_path: Path) -> None:
        self._lib = _Lib(lib_path).lib
        self._state = ctypes.create_string_buffer(self._lib.sim_route_downlink_size())
        self.rev = 0

    def set(self, rev: int, buoy_id: str, loop: bool, points: list[tuple[float, float]]) -> bool:
        lat = (ctypes.c_double * max(len(points), 1))(*(p[0] for p in points))
        lon = (ctypes.c_double * max(len(points), 1))(*(p[1] for p in points))
        ok = bool(self._lib.sim_route_downlink_set(self._state, rev, buoy_id.encode(), loop, len(points), lat, lon))
        self.rev = rev if ok else 0
        return ok

    def frame(self, beacon_id: str, w_value: str | None) -> bytes:
        """The frame to send after a beacon from beacon_id whose w= value is w_value (None: absent)."""
        ack = _Ack()
        if w_value is not None:
            self._lib.RouteDownlink_ParseAck(w_value.encode(), ctypes.byref(ack))
        buf = ctypes.create_string_buffer(121)
        length = self._lib.RouteDownlink_Frame(self._state, beacon_id.encode(), ctypes.byref(ack), buf, len(buf))
        return buf.raw[:length]

    def ack(self, buoy_id: str, eseq: int, lights: int, position: tuple[float, float] | None,
            role: tuple[str, int, int] | None = None) -> bytes:
        """RouteDownlink_Ack: the answer to every frame that gets no RTE frame. lights: the whole command
        (lights, STOP, RETURN); role: (leader, right_m, back_m) while the boat sails in formation."""
        buf = ctypes.create_string_buffer(121)
        lat, lon = position if position is not None else (0.0, 0.0)
        r = FormationRole()
        if role is not None:
            r.on, r.leader, r.right_m, r.back_m = True, role[0].encode()[:31], int(role[1]), int(role[2])
        length = self._lib.RouteDownlink_Ack(buoy_id.encode(), eseq & 0xFFFF, lights & 15, position is not None, lat, lon,
                                             ctypes.byref(r), buf, len(buf))
        return buf.raw[:length]

    def event_json(self, frame: bytes) -> tuple[str, dict] | None:
        """The buoy id and the "event" group the station posts for an ev= frame, or None."""
        import json
        buf = ctypes.create_string_buffer(256)
        length = self._lib.sim_event_json(frame, buf, len(buf))
        if length == 0:
            return None
        buoy_id, _, body = buf.raw[:length].decode().partition("\t")
        return buoy_id, json.loads(body)


def _e6(value: float) -> int:
    return int(round(value * 1e6))


class BuoyShore:
    """shore_link_t of the F407: the station's answers keep the link, bring the lights and the way home."""

    def __init__(self, lib_path: Path) -> None:
        self._lib = _Lib(lib_path).lib
        self._state = ctypes.create_string_buffer(self._lib.sim_shore_link_size())   # zeroed = ShoreLink_Init

    def heard(self, now_ms: int) -> None:
        self._lib.ShoreLink_Heard(self._state, now_ms & 0xFFFFFFFF)

    def on_frame(self, frame: bytes, buoy_id: str, now_ms: int) -> int | None:
        """The eseq an ACK acknowledges (0: none), or None if the frame is not an ACK for buoy_id."""
        eseq = ctypes.c_uint16(0)
        if not self._lib.ShoreLink_OnFrame(self._state, frame, buoy_id.encode(), now_ms & 0xFFFFFFFF, ctypes.byref(eseq)):
            return None
        return eseq.value

    def ok(self, now_ms: int) -> bool:
        return bool(self._lib.ShoreLink_Ok(self._state, now_ms & 0xFFFFFFFF))

    def pi_line(self, seq: int, now_ms: int) -> str:
        buf = ctypes.create_string_buffer(96)
        self._lib.ShoreLink_PiLine(self._state, seq & 0xFFFF, now_ms & 0xFFFFFFFF, buf, len(buf))
        return buf.value.decode()

    @property
    def lights(self) -> int:
        return self._lib.sim_shore_lights(self._state)

    @property
    def hold(self) -> bool:
        """STOP from the dashboard (bit 2 of the ACK's command): the motors stay off."""
        return bool(self._lib.sim_shore_hold(self._state))

    @property
    def home(self) -> tuple[float, float] | None:
        lat, lon = ctypes.c_double(), ctypes.c_double()
        return (lat.value, lon.value) if self._lib.sim_shore_home(self._state, ctypes.byref(lat), ctypes.byref(lon)) else None

    @property
    def ret(self) -> bool:
        """RETURN from the dashboard (bit 3 of the ACK's command): the Pi takes the boat home."""
        return bool(self._lib.sim_shore_ret(self._state))

    @property
    def follow(self) -> tuple[str, int, int] | None:
        """(leader, right_m, back_m) while the boat sails in formation, else None."""
        leader, right, back = ctypes.create_string_buffer(32), ctypes.c_int16(), ctypes.c_int16()
        if not self._lib.sim_shore_follow(self._state, leader, len(leader), ctypes.byref(right), ctypes.byref(back)):
            return None
        return leader.value.decode(), right.value, back.value

    def follow_line(self, seq: int) -> str:
        buf = ctypes.create_string_buffer(64)
        self._lib.ShoreLink_FollowLine(self._state, seq & 0xFFFF, buf, len(buf))
        return buf.value.decode()


class Slots:
    """App/lora_slot.c: the boat's own time slot on the shared channel, on the GPS clock (the wall
    clock here, the same for every process)."""

    def __init__(self, lib_path: Path, buoy_id: str, slots: int, slot_s: float) -> None:
        self._lib = _Lib(lib_path).lib
        self.slots, self.slot_ms = slots, int(slot_s * 1000)
        self.slot = self._lib.LoraSlot_FromId(buoy_id.encode(), slots)

    def wait_s(self, after_s: float = 0.0) -> float:
        """Seconds from after_s seconds from now to the start of the boat's next slot."""
        clock_ms = int((time.time() + after_s) * 1000) % 86_400_000
        return self._lib.LoraSlot_Wait(clock_ms, self.slot, self.slots, self.slot_ms) / 1000.0


class BeaconParser:
    """A beacon heard on the channel (lora_payload.c), for a follower watching its leader."""

    def __init__(self, lib_path: Path) -> None:
        self._lib = _Lib(lib_path).lib

    def parse(self, frame: bytes) -> dict | None:
        """id, lat, lon, yaw, course and speed (None: absent), or None if it is not a beacon with a position."""
        buoy = ctypes.create_string_buffer(32)
        lat, lon = ctypes.c_double(), ctypes.c_double()
        yaw, course, speed = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
        if not self._lib.sim_beacon_parse(frame, buoy, len(buoy), ctypes.byref(lat), ctypes.byref(lon), ctypes.byref(yaw),
                                          ctypes.byref(course), ctypes.byref(speed)):
            return None
        return {"id": buoy.value.decode(), "lat": lat.value, "lon": lon.value,
                "yaw": yaw.value if yaw.value >= 0 else None, "course": course.value if course.value >= 0 else None,
                "speed": speed.value if speed.value >= 0 else None}


class BuoyEvents:
    """event_queue_t of the F407: events in their own frames until the station acknowledges them."""

    def __init__(self, lib_path: Path) -> None:
        self._lib = _Lib(lib_path).lib
        self._state = ctypes.create_string_buffer(self._lib.sim_event_queue_size())
        self._lib.EventQueue_Init(self._state)

    def victim(self, position: tuple[float, float] | None, risk: int, confidence_pct: int, people: int) -> int:
        lat, lon = position or (0.0, 0.0)
        return self._lib.EventQueue_PushVictim(self._state, position is not None, _e6(lat), _e6(lon), risk, confidence_pct, people)

    def tilt(self, position: tuple[float, float] | None, roll: float, pitch: float) -> int:
        lat, lon = position or (0.0, 0.0)
        return self._lib.EventQueue_PushTilt(self._state, position is not None, _e6(lat), _e6(lon), roll, pitch)

    def confirm(self, position: tuple[float, float] | None, presses: int) -> int:
        lat, lon = position or (0.0, 0.0)
        return self._lib.EventQueue_PushConfirm(self._state, position is not None, _e6(lat), _e6(lon), presses & 0xFFFF)

    @property
    def pending(self) -> int:
        return self._lib.sim_event_count(self._state)

    def frame(self, buoy_id: str) -> str:
        buf = ctypes.create_string_buffer(120)
        length = self._lib.EventQueue_Frame(self._state, buoy_id.encode(), buf, len(buf))
        return buf.raw[:length].decode()

    def ack(self, eseq: int) -> bool:
        return bool(self._lib.EventQueue_Ack(self._state, eseq & 0xFFFF))

"""The LoRa link between the simulated STM32F407 (stm32_board_sim.py) and the simulated shore
station (shore_station_sim.py): UDP on 127.0.0.1 standing in for the 433 MHz radio, with the
air time of each frame, and the firmware's own route code (librouteradio.so, built by
tools/hw_standins.sh from STM32F4/App/route_link.c and esp32-lora-station/main/route_downlink.c).

A datagram leaves when the frame has finished on air, which is when the real receiver gets it.
The buoy only takes what arrives while it listens (its 1.5 s window after each beacon), as the
SX1278 hears nothing outside receive mode.
"""
from __future__ import annotations

import ctypes
import math
from pathlib import Path

BEACON_PORT = 47101         # buoy -> station
DOWNLINK_PORT = 47102       # station -> buoy
# Radio settings of both ends (STM32F4/App/app_config.h, esp32-lora-station/main/main.c)
SF, BW_HZ, CR, PREAMBLE = 9, 125000, 5, 8


def airtime_s(length: int) -> float:
    """Semtech AN1200.13 time on air: explicit header, CRC on, no low data rate optimisation."""
    symbol = (1 << SF) / BW_HZ
    payload = 8 + max(math.ceil((8 * length - 4 * SF + 28 + 16) / (4 * SF)) * CR, 0)
    return (PREAMBLE + 4.25) * symbol + payload * symbol


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
                                          ctypes.c_char_p, ctypes.c_size_t)
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

    def ack(self, buoy_id: str, eseq: int, lights: int, position: tuple[float, float] | None) -> bytes:
        """RouteDownlink_Ack: the answer to every frame that gets no RTE frame."""
        buf = ctypes.create_string_buffer(121)
        lat, lon = position if position is not None else (0.0, 0.0)
        length = self._lib.RouteDownlink_Ack(buoy_id.encode(), eseq & 0xFFFF, lights & 7, position is not None, lat, lon, buf, len(buf))
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

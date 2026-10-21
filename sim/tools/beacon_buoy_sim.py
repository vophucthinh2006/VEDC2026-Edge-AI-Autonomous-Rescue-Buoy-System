#!/usr/bin/env python3
"""Buoys that only beacon, for the dashboard next to the simulated boat (sim/tools/stm32_web_sim.sh).

Each one stands still where it is put and sends the STM32F407's LoRa beacon (STM32F4/App/lora_beacon.c
build_frame, as tools/stm32_board_sim.py does) to the simulated shore station (tools/shore_station_sim.py,
UDP tools/lora_radio.py BEACON_PORT) every 5 s; the station posts it to the Worker like any other.
They listen for nothing: no route, no lights, no STOP; send routes to the simulated boat only.

    python3 beacon_buoy_sim.py --buoy PHAO-02,10.883888,106.796469,90,72 --buoy PHAO-03,10.882988,106.796469,270,64
                                       id,lat,lon,heading deg,battery %
"""
from __future__ import annotations

import argparse
import socket
import time

from lora_radio import BEACON_PORT, airtime_s

PERIOD_S = 5.0     # LORA_BEACON_PERIOD_MS


def parse_buoy(text: str) -> tuple[str, float, float, float, int]:
    fields = text.split(",")
    if len(fields) != 5:
        raise argparse.ArgumentTypeError("id,lat,lon,heading,battery")
    return fields[0], float(fields[1]), float(fields[2]), float(fields[3]) % 360.0, int(fields[4])


def frame(buoy: tuple[str, float, float, float, int], seq: int) -> str:
    """lora_beacon.c build_frame of a buoy at rest, armed and stopped (m=S), level, its IMU calibrated."""
    buoy_id, lat, lon, heading, battery = buoy
    return (f"id={buoy_id},{lat:.6f},{lon:.6f},r=0.0,p=0.0,y={heading:.1f},t=-1.0,m=S,i=1,c=3,q={seq},v=0,"
            f"H=0,L=0,b={battery},g={heading:.1f},s=0.00")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--buoy", type=parse_buoy, action="append", required=True, help="id,lat,lon,heading,battery")
    args = parser.parse_args()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    station = ("127.0.0.1", BEACON_PORT)
    # Spread over the period, as boats switched on at different times are: their frames do not meet on air.
    count = len(args.buoy)
    start = time.monotonic() + 2.0
    due = [start + PERIOD_S * (i + 0.5) / count for i in range(count)]
    seq = [0] * count
    print(f"beaconing {', '.join(b[0] for b in args.buoy)} to udp {BEACON_PORT} every {PERIOD_S:.0f} s", flush=True)
    while True:
        i = min(range(count), key=lambda k: due[k])
        time.sleep(max(0.0, due[i] - time.monotonic()))
        seq[i] += 1
        text = frame(args.buoy[i], seq[i])
        time.sleep(airtime_s(len(text)))       # the station has it once it is off the air
        tx.sendto(text.encode(), station)
        due[i] += PERIOD_S


if __name__ == "__main__":
    raise SystemExit(main())

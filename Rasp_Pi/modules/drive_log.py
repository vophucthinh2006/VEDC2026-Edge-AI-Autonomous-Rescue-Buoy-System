"""Why the STM32 boat stopped, in the log (main.py): one line on each change, with what the Pi knew.

  the boat stops       the reason of the NAV command that stopped it, and the state behind it
  the boat drives      the reason it drives again
  the F407 disarms     from its $SYS; the likely cause (overturned, IMU lost) from its $IMU
  the F407 goes silent no $SYS for SILENT_S: the UART, the adapter or the board itself

The terminal of main.py used to be the only record, and its status line changes too often to read
a stop out of it afterwards.
"""
from __future__ import annotations

from modules.navigation import NavCommand
from modules.state_store import Snapshot

SILENT_S = 1.0   # $SYS comes 10 times a second


def _age(now: float, stamp: float) -> str:
    return f"{now - stamp:.1f} s ago" if stamp > 0.0 else "never"


def describe(snap: Snapshot, now: float) -> str:
    """The state behind a stop, in one line."""
    parts = [
        f"IMU {'ok' if snap.imu.imu_ok else 'NOT ok'} {_age(now, snap.imu.timestamp)}"
        + (" OVERTURNED" if snap.imu.overturned else ""),
        f"GPS fix {snap.gps.fix} {_age(now, snap.gps.timestamp)}",
        f"F407 {'armed' if snap.vehicle.armed else 'disarmed'} {_age(now, snap.vehicle.timestamp)}",
        f"F407 hears the Pi: {'yes' if snap.system.link_ok else 'no'}",
        f"shore {'ok' if snap.link.shore_ok else 'LOST'}" + (", HOLD from the dashboard" if snap.link.hold else ""),
        f"battery {snap.system.battery_v:.2f} V",
        f"{len(snap.obstacles)} obstacles",
    ]
    return "; ".join(parts)


def _disarm_cause(snap: Snapshot) -> str:
    if snap.imu.overturned:
        return "overturned"
    if not snap.imu.imu_ok:
        return "IMU lost"
    return "cause not reported"


class DriveLog:
    def __init__(self) -> None:
        self._driving = False
        self._armed: bool | None = None     # None until the first $SYS
        self._silent = False

    def update(self, command: NavCommand, snap: Snapshot, now: float) -> list[str]:
        """Lines for the log, none while nothing changes."""
        lines: list[str] = []
        heard = snap.vehicle.timestamp > 0.0
        silent = heard and now - snap.vehicle.timestamp > SILENT_S
        if silent and not self._silent:
            lines.append(f"F407 silent: no $SYS for {now - snap.vehicle.timestamp:.1f} s (UART, adapter or the board)")
        elif self._silent and not silent:
            lines.append("F407 heard again")
        self._silent = silent
        if heard and not silent:
            armed = snap.vehicle.armed
            if self._armed is not None and armed != self._armed:
                lines.append("F407 armed" if armed else f"F407 DISARMED ({_disarm_cause(snap)}): {describe(snap, now)}")
            self._armed = armed
        # Coasting down to the speed it was given (overspeed) the boat is still under way, not stopped.
        driving = command.mode == "BACK" or (command.mode == "AUTO" and (command.speed_mps > 0.0 or command.reason == "overspeed"))
        if self._driving and not driving:
            lines.append(f"boat stopped: {command.mode} {command.reason}; {describe(snap, now)}")
        elif driving and not self._driving:
            lines.append(f"boat driving: {command.reason} {command.speed_mps:.2f} m/s heading {command.heading_deg:.0f}")
        self._driving = driving
        return lines

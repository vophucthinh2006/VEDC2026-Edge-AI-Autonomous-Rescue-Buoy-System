"""Existing shore wire protocol; no invented ownership or GPS coordinates."""
import math
import re

ID = re.compile(r"PHAO-[0-9]{2}")

def coordinate(lat, lon):
    lat, lon = float(lat), float(lon)
    if not math.isfinite(lat + lon) or abs(lat) > 90 or abs(lon) > 180:
        raise ValueError("invalid coordinates")
    return lat, lon

def distance(a, b):
    p, q = math.radians(a[0]), math.radians(b[0])
    h = math.sin((q-p)/2)**2 + math.cos(p)*math.cos(q)*math.sin(math.radians(b[1]-a[1])/2)**2
    return 12742000 * math.asin(math.sqrt(min(1, max(0, h))))

def checked_body(frame):
    body, check = frame.rsplit("*", 1)
    cs = 0
    for b in body.encode("ascii"): cs ^= b
    if len(check) != 2 or int(check, 16) != cs:
        raise ValueError("checksum")
    return body

def ack(frame, buoy_id):
    f = checked_body(frame).split(",")
    if len(f) not in (6, 9, 11) or f[:2] != ["ACK", buoy_id]:
        raise ValueError("not our ACK")
    seq, command = int(f[2]), int(f[3])
    if not 0 <= seq <= 65535 or not 0 <= command <= 15: raise ValueError("ACK range")
    if f[4] or f[5]: coordinate(f[4], f[5])
    if len(f) >= 9 and any(f[6:9]):
        if not ID.fullmatch(f[6]) or abs(int(f[7])) > 500 or abs(int(f[8])) > 500:
            raise ValueError("formation")
    skip = coordinate(f[9], f[10]) if len(f) == 11 else None
    return seq, skip

def offset(origin, bearing_deg, metres):
    """Point metres away from origin (lat, lon) along a compass bearing."""
    lat, lon = origin
    return (lat + metres*math.cos(math.radians(bearing_deg))/111_320.0,
            lon + metres*math.sin(math.radians(bearing_deg))/(111_320.0*math.cos(math.radians(lat))))

def victim(buoy_id, seq, pos, confidence, count, risk=2):
    lat, lon = coordinate(*pos)
    # Legacy protocol requires risk 0..3 (0 = safe; the dashboard alerts only on risk > 0).
    # Default 2 = high, the F407 default when its Pi sends no risk: not a drowning assessment.
    return f"ev={buoy_id},{seq},V,{lat:.6f},{lon:.6f},{risk},{confidence:.2f},{count}"

def beacon(buoy_id, seq, pos, count):
    location = f"{pos[0]:.6f},{pos[1]:.6f}" if pos else "NO_FIX"
    return f"id={buoy_id},{location},r=0,p=0,y=0,t=-1,m=S,i=0,c=0,q={seq},v={count}"

def diag(buoy_id, fps):
    """The F407's diagnostics frame (STM32F4/App/diag_frame.h) for a buoy that stands still: no IMU, no motors.
    fps: the detector's frames a second, None while it is off."""
    rate = "" if fps is None else f",f={min(999, round(fps*10))}"
    return f"dg={buoy_id},n=stationary,s=-,w=0/0,a=1,d=1,k=0:0{rate}"

class Claims:
    def __init__(self, radius=6, ttl=120):
        self.radius, self.ttl, self.points = radius, ttl, {}

    def add(self, pos, now):
        self.points[pos] = now + self.ttl

    def blocked(self, pos, now):
        self.points = {p: end for p, end in self.points.items() if end > now}
        return any(distance(pos, p) <= self.radius for p in self.points)

class Relay:
    """One transparent store-and-forward relay; legacy peers need no envelope changes.

    Not general multi-hop routing: dedup only bounds repeats for this prototype.
    ACK retransmission is allowed after cache expiry for legitimate event retries.
    """
    def __init__(self, buoy_id, ttl=2.0):
        self.buoy_id, self.ttl, self.seen = buoy_id, ttl, {}

    def accept(self, frame, now):
        self.seen = {p: end for p, end in self.seen.items() if end > now}
        if frame in self.seen or len(frame.encode("ascii")) > 120: return False
        if frame.startswith(("id=", "ev=")):
            source = frame.split(",", 1)[0][3:]
            if not ID.fullmatch(source) or source == self.buoy_id: return False
        elif frame.startswith("ACK,"):
            fields = checked_body(frame).split(",")
            if len(fields) not in (6, 9, 11) or not ID.fullmatch(fields[1]) or fields[1] == self.buoy_id:
                return False
            ack(frame, fields[1])
        else: return False
        self.seen[frame] = now + self.ttl
        return True

class NMEA:
    """Only accept checksum-valid RMC/GGA fixes. Unknown converter data stays raw."""
    def __init__(self):
        self.buffer = bytearray()
        self.position = None
        self.updated = -1e9

    def feed(self, data, now):
        for b in data:
            if b == 10:
                try: self.line(bytes(self.buffer).decode("ascii").strip(), now)
                except (ValueError, UnicodeError, IndexError): pass
                self.buffer.clear()
            elif len(self.buffer) < 256: self.buffer.append(b)
            else: self.buffer.clear()

    def line(self, line, now):
        if not line.startswith("$"): return
        f = checked_body(line[1:]).split(",")
        if f[0] in ("GPRMC", "GNRMC"):
            if f[2] != "A": self.position = None; return
            a, ns, b, ew = f[3:7]
        elif f[0] in ("GPGGA", "GNGGA"):
            if int(f[6]) <= 0: self.position = None; return
            a, ns, b, ew = f[2:6]
        else: return
        if ns not in ("N", "S") or ew not in ("E", "W"): raise ValueError("hemisphere")
        def degrees(s):
            value = float(s); d = int(value // 100); minutes = value-d*100
            if not 0 <= minutes < 60: raise ValueError("minutes")
            return d + minutes/60
        self.position = coordinate(degrees(a)*(-1 if ns == "S" else 1), degrees(b)*(-1 if ew == "W" else 1))
        self.updated = now

    def fix(self, now):
        return self.position if now-self.updated <= 3 else None

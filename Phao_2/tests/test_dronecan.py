import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from test_prototype import ROOT
from protocol import NMEA

FIX2_SIG, ALLOC_SIG = 0xca41e7000f37435f, 0x0b2a812620a11d40

def crc(sig, data):
    c = 0xFFFF
    for b in sig.to_bytes(8, "little") + data:
        c ^= b << 8
        for _ in range(8): c = (c << 1 ^ 0x1021 if c & 0x8000 else c << 1) & 0xFFFF
    return c

def field(value, n):
    """DSDL bit string of an n-bit integer: little-endian bytes, the odd top bits last."""
    value &= (1 << n) - 1
    out = ""
    while n > 0:
        k = min(8, n); out += format(value & 0xFF if k == 8 else value, "0%db" % k)
        value >>= 8; n -= k
    return out

def fix2(lat, lon, status=3, sats=14):
    s = field(0, 56)*2 + field(2, 3) + field(0, 13) + field(0, 8)
    s += field(round(lon*1e8), 37) + field(round(lat*1e8), 37) + field(5000, 27) + field(-5000, 27)
    s += field(0, 96) + field(sats, 6) + field(status, 2) + field(0, 10) + field(0, 6) + field(0, 16)
    s += "0"*(-len(s) % 8)
    return bytes(int(s[i:i+8], 2) for i in range(0, len(s), 8))

def frames(data, sig, can_id, tid=3):
    body = crc(sig, data).to_bytes(2, "little") + data
    parts = [body[i:i+7] for i in range(0, len(body), 7)]
    return [(can_id, p + bytes([(0x80 if i == 0 else 0) | (0x40 if i == len(parts)-1 else 0) | (0x20 if i % 2 else 0) | tid]))
            for i, p in enumerate(parts)]

@unittest.skipUnless(shutil.which("gcc"), "host gcc required")
class DroneCan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.temp.name)/("can.exe" if os.name == "nt" else "can")
        fw = ROOT/"stm32f1"
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(fw), str(ROOT/"tests/dronecan_fixture.c"),
                        str(fw/"dronecan.c"), "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def run_frames(self, items, ms=0):
        text = "".join("%d %x %s\n" % (ms, i, d.hex()) for i, d in items)
        return subprocess.run([str(self.exe)], input=text, capture_output=True, text=True, check=True).stdout.splitlines()

    FIX_ID = 16 << 24 | 1063 << 8 | 50

    def test_fix_reaches_the_pi_parser(self):
        for lat, lon in ((10.88345678, 106.79612345), (-33.5, -70.25), (89.99999999, 179.99999999)):
            out = self.run_frames(frames(fix2(lat, lon), FIX2_SIG, self.FIX_ID))
            gps = NMEA(); gps.feed((out[0] + "\r\n").encode(), 1)
            self.assertAlmostEqual(gps.fix(1)[0], lat, places=6); self.assertAlmostEqual(gps.fix(1)[1], lon, places=6)
            self.assertIn("status=3 sats=14 node=50", out[-1])

    def test_no_fix_gives_no_position(self):
        out = self.run_frames(frames(fix2(0, 0, status=0), FIX2_SIG, self.FIX_ID))
        gps = NMEA(); gps.feed((out[0] + "\r\n").encode(), 1)
        self.assertIsNone(gps.fix(1))

    def test_corrupt_or_lost_frames_are_rejected(self):
        good = frames(fix2(10, 106), FIX2_SIG, self.FIX_ID)
        bad = list(good); bad[3] = (bad[3][0], b"\xff" + bad[3][1][1:])
        self.assertEqual(self.run_frames(bad), ["END fixes=0 bad=1 status=0 sats=0 node=0"])
        self.assertIn("END fixes=0", self.run_frames(good[:2] + good[3:])[-1])
        self.assertIn("fixes=1", self.run_frames(bad + good)[-1])

    def test_allocates_a_node_id(self):
        uid = bytes(range(1, 17))
        anon = 30 << 24 | 0x155 << 10 | 1 << 8
        ask = lambda first, part, tid: (anon, bytes([first]) + part + bytes([0xC0 | tid]))
        out = self.run_frames([ask(1, uid[:6], 0), ask(0, uid[6:12], 1), ask(0, uid[12:], 2)])
        sent = [(int(l.split()[1], 16), bytes.fromhex(l.split()[2])) for l in out if l.startswith("TX")]
        mine = 30 << 24 | 1 << 8 | 100
        expected = [(mine, bytes([0]) + uid[:6] + bytes([0xC0]))]
        expected += frames(bytes([0]) + uid[:12], ALLOC_SIG, mine, 1)
        expected += frames(bytes([50 << 1]) + uid, ALLOC_SIG, mine, 2)
        self.assertEqual(sent, expected)

    def test_followup_without_a_first_part_is_ignored(self):
        self.assertEqual(len(self.run_frames([(30 << 24 | 1 << 8, b"\x00abcdef\xc1")])), 1)

if __name__ == "__main__": unittest.main()

import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"pi4"))
from protocol import ack, NMEA, Claims, Relay, beacon, distance
from main import Engine

def checksum(body):
    cs = 0
    for b in body.encode(): cs ^= b
    return f"{body}*{cs:02X}"

def config():
    c = json.loads((ROOT/"pi4/config.example.json").read_text())
    c["demo_target"] = [10.8835, 106.7961]
    return c

class PrototypeTests(unittest.TestCase):
    def test_ack_checksum_and_recipient(self):
        frame = checksum("ACK,PHAO-02,7,0,,,,,,10.883500,106.796100")
        self.assertEqual(ack(frame,"PHAO-02"), (7,(10.8835,106.7961)))
        for bad in (frame[:-2]+"XX", frame.replace("PHAO-02","PHAO-01")):
            with self.assertRaises(ValueError): ack(bad,"PHAO-02")

    def test_claim_expiry(self):
        c = Claims(); c.add((10,106), 0)
        self.assertTrue(c.blocked((10.00001,106),1))
        self.assertFalse(c.blocked((11,106),1))
        self.assertFalse(c.blocked((10,106),121))

    def test_camera_report_shore_skip_suppresses_repeat(self):
        events=[]; e=Engine(config(), lambda event,**kw: events.append(event))
        e.detected(.8,10)
        frame, _ = e.next_frame(10)
        self.assertTrue(frame.startswith("ev=PHAO-02,1,V,"))
        e.receive(checksum("ACK,PHAO-02,1,0,,,,,,10.883500,106.796100"),11)
        self.assertEqual(e.state,"SUPPRESSED"); self.assertIsNone(e.pending)
        e.detected(.9,40); self.assertIsNone(e.pending)
        self.assertEqual(events.count("DETECTION"),1)

    def test_no_ack_does_not_claim_owner(self):
        c=config(); c["max_attempts"]=1
        e=Engine(c,lambda *a,**k: None); e.detected(.8,0)
        e.next_frame(0); e.next_frame(10)
        self.assertEqual(e.state,"UNCONFIRMED")

    def test_unrelated_ack_does_not_clear_pending(self):
        e=Engine(config(),lambda *a,**k:None); e.detected(.8,1)
        e.receive(checksum("ACK,PHAO-02,9,0,,"),2)
        self.assertIsNotNone(e.pending)

    def test_no_fake_position_from_buoy_gps(self):
        c=config(); c["demo_target"]=None
        e=Engine(c,lambda *a,**k:None); e.gps.position=(10,106); e.gps.updated=0
        e.detected(.9,1)
        self.assertIsNone(e.pending)
        self.assertEqual(e.state,"DETECTED_NO_TARGET_POSITION")

    def test_relay_loop_cache_and_ack(self):
        r=Relay("PHAO-02"); frame=beacon("PHAO-01",1,None,0)
        self.assertTrue(r.accept(frame,0)); self.assertFalse(r.accept(frame,1))
        self.assertTrue(r.accept(frame,3))
        self.assertFalse(r.accept(beacon("PHAO-02",1,None,0),4))
        self.assertTrue(r.accept(checksum("ACK,PHAO-01,1,0,,"),4))
        self.assertFalse(r.accept("RTE,PHAO-01,bad",4))

    def test_gps_chunking_checksum_invalid_and_stale(self):
        n=NMEA(); s=("$"+checksum("GNRMC,123519,A,4807.038,N,01131.000,E,0,0,230394,,,A")+"\r\n").encode()
        n.feed(s[:17],0); self.assertIsNone(n.fix(0))
        n.feed(s[17:],1)
        self.assertAlmostEqual(n.fix(1)[0],48.1173)
        self.assertIsNone(n.fix(5))
        n.feed(s.replace(b"4807.038",b"9907.038"),6); self.assertIsNone(n.fix(6))
        n.feed(("$"+checksum("GNRMC,123519,V,,,,,,,230394,,,A")+"\n").encode(),7)
        self.assertIsNone(n.fix(7))

    def test_raw_can_not_mistaken_for_gps(self):
        n=NMEA(); n.feed(b"t1238AABBCCDDEEFF0011\r\n",0)
        self.assertIsNone(n.fix(0))

    def test_victim_risk_alerts_dashboard(self):
        e=Engine(config(),lambda *a,**k:None); e.detected(.6,1)
        risk=e.next_frame(1)[0].split(",")[5]
        self.assertGreater(int(risk),0)  # dashboard alerts only on risk > 0

    def test_edge_risk_is_reported_and_people_on_boats_are_not(self):
        from types import SimpleNamespace as P
        from camera import assess
        self.assertEqual(assess([]),(0.0,0,None))
        self.assertEqual(assess([P(on_boat=True,risk=0,confidence=.9)]),(0.0,0,None))
        self.assertEqual(assess([P(on_boat=False,risk=1,confidence=.9),P(on_boat=False,risk=3,confidence=.5)])[:2],(.5,3))
        e=Engine(config(),lambda *a,**k:None); e.detected(.5,1,3)
        self.assertEqual(e.next_frame(1)[0].split(",")[5:7],["3","0.50"])

    def test_indoor_position_from_the_assumed_place_and_the_camera(self):
        from camera import ranged
        cam={"horizontal_fov_deg":60,"vertical_fov_deg":47,"person_height_m":1.7}
        bearing,metres=ranged((.3,.6,.8,.8),cam)
        self.assertAlmostEqual(bearing,12); self.assertAlmostEqual(metres,1.7/(2*math.tan(math.radians(23.5))*.5))
        self.assertIsNone(ranged((.3,.4,1.0,.6),cam)[1])
        c=config(); c["assumed_position"]=[10.8835,106.7961]; c["assumed_heading_deg"]=90
        e=Engine(c,lambda *a,**k:None); e.detected(.5,1,1,(0,5.0))
        self.assertAlmostEqual(distance(e.target,(10.8835,106.7961)),5,places=1); self.assertGreater(e.target[1],106.7961)
        far=Engine(c,lambda *a,**k:None); far.detected(.5,1,1,(0,12.0)); self.assertIsNone(far.pending)   # beyond 10 m
        cut=Engine(c,lambda *a,**k:None); cut.detected(.5,1,1,(0,None)); self.assertEqual(cut.target,(10.8835,106.7961))

    def test_a_real_fix_replaces_everything_assumed(self):
        c=config(); c["assumed_position"]=[10.8835,106.7961]
        real=(20.0,100.0)
        e=Engine(c,lambda *a,**k:None); e.gps.position=real; e.gps.updated=1
        e.detected(.5,1,1,(0,5.0)); self.assertAlmostEqual(distance(e.target,real),5,places=1)
        cut=Engine(c,lambda *a,**k:None); cut.gps.position=real; cut.gps.updated=1
        cut.detected(.5,1,1,(0,None)); self.assertIsNone(cut.pending)     # no range: not the demo point
        self.assertEqual(cut.state,"DETECTED_NO_TARGET_POSITION")
        old=Engine(c,lambda *a,**k:None); old.gps.position=real; old.gps.updated=1
        old.detected(.5,10,1,(0,5.0)); self.assertAlmostEqual(distance(old.target,(10.8835,106.7961)),5,places=1)   # fix lost

    def test_the_demo_switch_beats_a_real_fix(self):
        c=config(); c["assumed_position"]=[10.8835,106.7961]
        e=Engine(c,lambda *a,**k:None); e.gps.position=(20.0,100.0); e.gps.updated=1; e.demo=True
        e.detected(.5,1,1,(0,5.0)); self.assertAlmostEqual(distance(e.target,(10.8835,106.7961)),5,places=1)
        cut=Engine(c,lambda *a,**k:None); cut.gps.position=(20.0,100.0); cut.gps.updated=1; cut.demo=True
        cut.detected(.5,1,1,(0,None)); self.assertEqual(cut.target,(10.8835,106.7961))

    def test_diagnostics_go_last_and_every_ten_seconds(self):
        e=Engine(config(),lambda *a,**k:None); e.fps=2.3; e.detected(.5,1)
        self.assertTrue(e.next_frame(1)[0].startswith("ev=")); self.assertTrue(e.next_frame(1)[0].startswith("id="))
        self.assertIsNone(e.next_frame(1)); e.pending=None      # a report waiting for its ACK comes first
        self.assertEqual(e.next_frame(1),("dg=PHAO-02,n=stationary,s=-,w=0/0,a=1,d=1,k=0:0,f=23","diag"))
        e.next_beacon=1e9
        self.assertIsNone(e.next_frame(10.9)); self.assertEqual(e.next_frame(11)[1],"diag")

    def test_fps_counts_the_last_ten_seconds(self):
        from camera import rate
        self.assertIsNone(rate([],5)); self.assertIsNone(rate([1.0],5))
        self.assertAlmostEqual(rate([10+i*.5 for i in range(9)],14),2.0)
        self.assertIsNone(rate([1.0,1.5],30))      # the detector stopped

    def test_relay_uplink_waits_out_shore_ack_window(self):
        events=[]; e=Engine(config(),lambda ev,**k: events.append((ev,k))); e.next_beacon=1e9
        e.receive(beacon("PHAO-01",1,None,0),0)
        self.assertIsNone(e.next_frame(1.5))  # shore ACK slot for PHAO-01
        self.assertTrue(e.next_frame(2.0)[0].startswith("id=PHAO-01,"))

    def test_relay_uplink_dropped_when_shore_acks_origin(self):
        events=[]; e=Engine(config(),lambda ev,**k: events.append((ev,k))); e.next_beacon=e.next_diag=1e9
        e.receive(beacon("PHAO-01",1,None,0),0)
        ackf=checksum("ACK,PHAO-01,1,0,,")
        e.receive(ackf,0.1)
        self.assertEqual(e.next_frame(0.3),(ackf,"relay"))  # downlink ACK still relayed promptly
        self.assertIsNone(e.next_frame(2.0))
        self.assertIn(("RELAY_DROP",{"reason":"shore_acked","frame":beacon("PHAO-01",1,None,0)}),events)

    def test_requeue_keeps_relay_frame(self):
        e=Engine(config(),lambda *a,**k:None); e.next_beacon=1e9
        e.requeue("id=PHAO-01,NO_FIX",1)
        self.assertEqual(e.next_frame(1),("id=PHAO-01,NO_FIX","relay"))

    def test_radio_frame_size(self):
        self.assertLessEqual(len(beacon("PHAO-02",4294967295,(-89.999999,-179.999999),65535)),120)

if __name__ == "__main__": unittest.main()

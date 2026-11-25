#!/usr/bin/env python3
"""Stationary person-detection prototype, compatible with the existing shore arbiter."""
import argparse
from collections import deque
import json
from pathlib import Path
import random
import sys
import time

from protocol import ID, Claims, Relay, NMEA, ack, beacon, coordinate, diag, distance, offset, victim

DIAG_SECONDS = 10   # LORA_DIAG_PERIOD_MS of the F407
UPLINK_HOLD = 1.6  # s: past the origin's ~1.5 s ACK listen window

class Engine:
    def __init__(self, cfg, emit, persist=lambda seq, count: None, seq=0, count=0):
        self.cfg, self.emit, self.persist = cfg, emit, persist
        self.id = cfg["id"]
        self.seq, self.count = seq, count
        self.claims = Claims(cfg["claim_radius_m"], cfg["claim_ttl_s"])
        self.relay = Relay(self.id)
        self.gps = NMEA()
        self.pending = None
        self.state = "SEARCHING"
        self.next_report = 0
        self.next_beacon = 0
        self.beacon_seq = 0
        self.demo = False    # the switch (Rasp_Pi/modules/demo_switch.py)
        self.fps = None      # the detector's, for the diagnostics frame
        self.next_diag = 0
        self.jobs = deque(maxlen=16)
        self.last_ack = -1e9
        self.target = self.fallback = coordinate(*cfg["demo_target"]) if cfg.get("demo_target") else None

    def locate(self, where, now):
        """(target, source) of the person, or None when they are further than max_range_m. self.demo, the switch: the
        assumed place whatever the GPS says, so a fix coming in cannot move the demo. Otherwise, with a real fix: from it and
        the camera, and no position without a range. Only without one, what is assumed: the buoy's place
        (assumed_position) and, without a range (box cut by the frame), the configured point."""
        fix = None if self.demo else self.gps.fix(now)
        assumed = None if fix else self.cfg.get("assumed_position")
        origin = fix or assumed
        fallback = (None, "no_position") if fix else (self.fallback, "configured_demo_target")
        if not origin or not where: return fallback
        bearing, metres = where
        if metres is None: return fallback
        if metres > self.cfg["max_range_m"]: return None
        target = coordinate(*offset(origin, self.cfg["assumed_heading_deg"] + bearing, metres))
        return target, "gps_and_camera" if fix else "assumed_position_and_camera"

    def detected(self, confidence, now, risk=2, where=None):
        located = self.locate(where, now)
        if located is None: return
        self.target, source = located
        if self.target is None:
            if self.state != "DETECTED_NO_TARGET_POSITION":
                self.state = "DETECTED_NO_TARGET_POSITION"; self.emit(self.state)
            return
        if self.claims.blocked(self.target, now):
            if self.state != "SUPPRESSED": self.emit("SUPPRESSED", reason="shore_claim", target=self.target)
            self.state = "SUPPRESSED"; return
        if self.pending or now < self.next_report: return
        self.seq = self.seq % 65535 + 1
        # This prototype has one configured rescue point; repeat observations aren't new people.
        self.count = max(1, self.count)
        self.persist(self.seq, self.count)
        self.pending = {"seq": self.seq, "frame": victim(self.id, self.seq, self.target, confidence, self.count, risk),
                        "next": now, "attempts": 0}
        self.state = "WAITING_ACK"
        self.emit("DETECTION", target=self.target, sequence=self.seq, confidence=confidence, risk=risk,
                  position_source=source)

    def receive(self, frame, now):
        if frame.startswith("ACK,"):
            try: seq, skip = ack(frame, self.id)
            except (ValueError, UnicodeError): pass
            else:
                self.last_ack = now
                if skip:
                    self.claims.add(skip, now)
                    if self.target and distance(self.target, skip) <= self.claims.radius:
                        self.state = "SUPPRESSED"
                        self.emit("SUPPRESSED", reason="shore_claim", target=skip)
                if self.pending and seq == self.pending["seq"]:
                    self.pending = None
                    # ACK confirms delivery, not an explicit owner identifier.
                    if self.state != "SUPPRESSED": self.state = "REPORT_ACKED"
                    self.next_report = now + 20
                    self.emit("REPORT_ACKED", sequence=seq, state=self.state)
                return
        if frame.startswith("ACK,"):
            # The shore heard that buoy directly: its held uplink no longer needs forwarding.
            heard = ("id=" + frame[4:11] + ",", "ev=" + frame[4:11] + ",")
            for job in [j for j in self.jobs if j[1].startswith(heard)]:
                self.jobs.remove(job); self.emit("RELAY_DROP", reason="shore_acked", frame=job[1])
        if self.cfg["relay"]:
            try: accepted = self.relay.accept(frame, now)
            except (ValueError, UnicodeError): accepted = False
            if accepted:
                if len(self.jobs) == self.jobs.maxlen:
                    self.emit("RELAY_DROP", reason="queue_full"); return
                # Prioritize downlink ACKs over forwarded telemetry. Uplink waits out the
                # shore's reply window (~50 ms + ~0.3 s ACK at SF9) so it can't collide with it.
                if frame.startswith("ACK,"):
                    self.jobs.appendleft((now + random.uniform(0.06, 0.12), frame, "relay"))
                else: self.jobs.append((now + UPLINK_HOLD + random.uniform(0, 0.3), frame, "relay"))

    def next_frame(self, now):
        # Due times already include the hold; drop jobs more than 2 s late.
        while self.jobs and now-self.jobs[0][0] > 2:
            self.jobs.popleft(); self.emit("RELAY_DROP", reason="expired")
        if self.jobs and self.jobs[0][0] <= now:
            _, frame, kind = self.jobs.popleft()
            self.emit("RELAY_TX", frame=frame)
            return frame, kind
        if self.pending and now >= self.pending["next"]:
            p = self.pending
            if p["attempts"] >= self.cfg["max_attempts"]:
                self.emit("REPORT_UNCONFIRMED", sequence=p["seq"])
                self.pending = None; self.state = "UNCONFIRMED"; self.next_report = now+20
            else:
                p["attempts"] += 1
                p["next"] = now + self.cfg["retry_seconds"] + random.uniform(0, 0.5)
                self.emit("REPORT_TX", sequence=p["seq"], attempt=p["attempts"])
                return p["frame"], "own"
        if now >= self.next_beacon:
            self.next_beacon = now + self.cfg["beacon_seconds"] + random.uniform(0, 0.5)
            self.beacon_seq += 1
            return beacon(self.id, self.beacon_seq, self.gps.fix(now), self.count), "own"
        # Last, and never in the middle of a report or a relayed frame: the station does not answer it.
        if now >= self.next_diag and not self.jobs and not self.pending:
            self.next_diag = now + DIAG_SECONDS
            return diag(self.id, self.fps), "diag"
        return None

    def requeue(self, frame, due):
        if len(self.jobs) == self.jobs.maxlen: self.emit("RELAY_DROP", reason="queue_full")
        else: self.jobs.appendleft((due, frame, "relay"))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--no-camera", action="store_true", help="relay/GPS diagnostic only; no synthetic detections")
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args()
    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    if not ID.fullmatch(cfg["id"]): raise SystemExit("id must be PHAO-NN")
    if cfg["beacon_seconds"] < 3 or cfg["retry_seconds"] < 3 or not 1 <= cfg["max_attempts"] <= 20:
        raise SystemExit("invalid radio timing / retry configuration")
    logs = Path(args.log_dir); logs.mkdir(parents=True, exist_ok=True)
    state_file = logs/"state.json"
    saved = json.loads(state_file.read_text()) if state_file.exists() else {}
    if saved and saved.get("id") != cfg["id"]: raise SystemExit("log directory belongs to a different buoy")
    with (logs/"events.jsonl").open("a", encoding="utf-8", buffering=1) as log, (logs/"gps_uart.bin").open("ab") as raw:
        def emit(event, **fields):
            line = json.dumps({"time": time.time(), "id": cfg["id"], "event": event, **fields}, ensure_ascii=False)
            print(line, flush=True); log.write(line+"\n")
        def persist(seq, count):
            temp = state_file.with_suffix(".tmp")
            temp.write_text(json.dumps({"id": cfg["id"], "seq": seq, "count": count}))
            temp.replace(state_file)
        engine = Engine(cfg, emit, persist, saved.get("seq", 0), saved.get("count", 0))
        import serial
        switch = None
        if cfg.get("assumed_position"):
            sys.path.append(str(Path(cfg["camera"]["rasp_pi"]).resolve()))
            from modules.demo_switch import DemoSwitch
            switch = DemoSwitch(cfg.get("demo_flag"), cfg.get("demo_url"))
        camera = None
        if not args.no_camera:
            from camera import Camera
            camera = Camera(cfg["camera"])
        emit("START", target=engine.target, mode="stationary", relay=cfg["relay"],
             detector=None if args.no_camera else cfg["camera"]["detector"])
        port = None; buffer = bytearray(); busy_until = 0; quiet_until = 0
        tx_kind = None; sent = None; last_frame = 0; consecutive = 0; reconnect = 0; ready = False
        ping_due = 0; camera_error = None; status_due = 0; flag_due = 0
        try:
            while True:
                now = time.monotonic()
                if now >= flag_due:
                    flag_due = now+1
                    if switch and engine.demo != switch.on:
                        engine.demo = not engine.demo; emit("DEMO_GPS", on=engine.demo)
                if camera and camera.error != camera_error:
                    camera_error = camera.error
                    if camera_error: emit("CAMERA_ERROR", detail=camera_error)
                if camera:
                    engine.fps = camera.fps(now)
                    captured, score, risk, where = camera.latest
                    if captured > last_frame:
                        last_frame = captured
                        consecutive = consecutive+1 if score > 0 else 0
                        if consecutive >= cfg["camera"]["confirm_frames"] and now-captured < 3:
                            engine.detected(score, now, risk, where)
                if port is None and now >= reconnect:
                    try:
                        port = serial.Serial(cfg["serial_port"], cfg["baud"], timeout=0, write_timeout=0.2)
                        buffer.clear(); ready=False; ping_due=0; busy_until=0
                        emit("SERIAL_CONNECTED")
                    except (serial.SerialException, OSError) as exc:
                        emit("SERIAL_ERROR", detail=str(exc)); reconnect=now+3
                if port:
                    try:
                        buffer.extend(port.read(4096))
                        if len(buffer) > 8192: buffer.clear(); emit("SERIAL_OVERFLOW")
                        while b"\n" in buffer:
                            line, _, rest = buffer.partition(b"\n"); buffer=bytearray(rest)
                            text = line.decode("ascii", errors="replace").strip()
                            if text.startswith("RX,"):
                                emit("RADIO_RX", frame=text[3:]); engine.receive(text[3:], now)
                                if text.startswith("RX,ACK,"): quiet_until=now
                            elif text.startswith("GPSHEX,"):
                                try: data = bytes.fromhex(text[7:])
                                except ValueError: emit("GPS_HEX_ERROR"); continue
                                raw.write(data); raw.flush(); engine.gps.feed(data, now)
                            elif text == "READY,2": ready=True
                            elif text == "TXDONE":
                                busy_until=0
                                # Own/forwarded uplink gets a clear receive window for the shore ACK.
                                quiet_until=now+(1.5 if tx_kind in ("own", "uplink") else 0.05)
                            elif text.startswith("ERROR,"):
                                emit("BRIDGE_ERROR", detail=text)
                                busy_until=0
                                if "RADIO_INIT" in text: ready=False
                                # Bridge never started this frame: put a relay job back (own reports retry anyway).
                                if text in ("ERROR,BUSY", "ERROR,TX") and sent and sent[1] == "relay":
                                    engine.requeue(sent[0], now+0.2); sent=None
                                if text == "ERROR,BUSY": busy_until=now+0.2  # bridge still sending
                            elif text.startswith("STATUS,"): emit("BRIDGE_STATUS", detail=text)
                        if not ready and now >= ping_due:
                            port.write(b"PING\n"); ping_due=now+3
                        if ready and now >= busy_until and now >= quiet_until:
                            job = engine.next_frame(now)
                            if job:
                                frame, kind = job
                                tx_kind = "uplink" if kind == "relay" and frame.startswith(("id=", "ev=")) else kind
                                encoded = frame.encode("ascii")
                                if len(encoded) > 120: emit("TX_DROP", reason="payload_too_long", frame=frame)
                                else: port.write(b"TX,"+encoded+b"\n"); busy_until=now+2.5; sent=job
                    except (serial.SerialException, OSError) as exc:
                        emit("SERIAL_ERROR", detail=str(exc)); port.close(); port=None; reconnect=now+3
                if now >= status_due:
                    emit("STATUS", state=engine.state, gps=engine.gps.fix(now),
                         shore_recent=now-engine.last_ack < 30, radio_ready=ready and port is not None)
                    status_due=now+5
                time.sleep(0.01)
        except KeyboardInterrupt: emit("STOP")
        finally:
            if port: port.close()
            if camera: camera.close()

if __name__ == "__main__": main()

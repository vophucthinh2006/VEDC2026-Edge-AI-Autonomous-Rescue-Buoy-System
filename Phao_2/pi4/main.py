#!/usr/bin/env python3
"""Stationary person-detection prototype, compatible with the existing shore arbiter."""
import argparse
from collections import deque
import json
from pathlib import Path
import random
import time

from protocol import ID, Claims, Relay, NMEA, ack, beacon, coordinate, distance, victim

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
        self.jobs = deque(maxlen=16)
        self.last_ack = -1e9
        self.target = coordinate(*cfg["demo_target"]) if cfg.get("demo_target") else None

    def detected(self, confidence, now):
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
        self.pending = {"seq": self.seq, "frame": victim(self.id, self.seq, self.target, confidence, self.count),
                        "next": now, "attempts": 0}
        self.state = "WAITING_ACK"
        self.emit("DETECTION", target=self.target, sequence=self.seq, confidence=confidence,
                  position_source="configured_demo_target")

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
        if self.cfg["relay"]:
            try: accepted = self.relay.accept(frame, now)
            except (ValueError, UnicodeError): accepted = False
            if accepted:
                if len(self.jobs) == self.jobs.maxlen:
                    self.emit("RELAY_DROP", reason="queue_full"); return
                # Prioritize downlink ACKs over forwarded telemetry.
                job = (now + random.uniform(0.06, 0.12), frame, "relay")
                if frame.startswith("ACK,"): self.jobs.appendleft(job)
                else: self.jobs.append(job)

    def next_frame(self, now):
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
        return None

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
        camera = None
        if not args.no_camera:
            from camera import Camera
            camera = Camera(cfg["camera"])
        emit("START", target=engine.target, mode="stationary", relay=cfg["relay"],
             detector=None if args.no_camera else cfg["camera"]["detector"])
        port = None; buffer = bytearray(); busy_until = 0; quiet_until = 0
        tx_kind = None; last_frame = 0; consecutive = 0; reconnect = 0; ready = False
        ping_due = 0; camera_error = None; status_due = 0
        try:
            while True:
                now = time.monotonic()
                if camera and camera.error != camera_error:
                    camera_error = camera.error
                    if camera_error: emit("CAMERA_ERROR", detail=camera_error)
                if camera:
                    captured, score = camera.latest
                    if captured > last_frame:
                        last_frame = captured
                        consecutive = consecutive+1 if score >= cfg["camera"]["confidence"] else 0
                        if consecutive >= cfg["camera"]["confirm_frames"] and now-captured < 3:
                            # HOG confidence is deliberately unspecified on the radio.
                            engine.detected(score if cfg["camera"]["detector"] == "yolo" else 0.0, now)
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
                            elif text.startswith("STATUS,"): emit("BRIDGE_STATUS", detail=text)
                        if not ready and now >= ping_due:
                            port.write(b"PING\n"); ping_due=now+3
                        if ready and now >= busy_until and now >= quiet_until:
                            job = engine.next_frame(now)
                            if job:
                                frame, kind = job
                                tx_kind = "uplink" if kind == "relay" and frame.startswith(("id=", "ev=")) else kind
                                encoded = frame.encode("ascii")
                                if len(encoded) > 120: raise ValueError("radio payload too long")
                                port.write(b"TX,"+encoded+b"\n"); busy_until=now+2.5
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

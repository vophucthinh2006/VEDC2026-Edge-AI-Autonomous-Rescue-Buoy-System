"""Live camera view with the person detector, on the Pi's own screen.

    DISPLAY=:0 python3 tools/cam_view.py                  # view only
    DISPLAY=:0 python3 tools/cam_view.py --uart /dev/ttyUSB0   # also drive the camera servo (CAM packets)
    python3 tools/cam_view.py --http 8080 --no-window          # MJPEG at http://<pi>:8080/ , no screen needed
Press q in the window to quit.
"""
from __future__ import annotations

import argparse
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from time import monotonic, strftime

import cv2
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from modules.ai_vision import VisionWorker  # noqa: E402
from modules.perception import estimate_distance_bbox, track_pan  # noqa: E402
from modules.state_store import StateStore  # noqa: E402
from utils.nmea_packet import encode  # noqa: E402


_jpeg = b""
_jpeg_lock = threading.Lock()


class _Stream(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        if self.path != "/stream":
            body = b'<!doctype html><title>Human tracking</title><body style="margin:0;background:#111"><img src="/stream" style="width:100%">'
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _jpeg_lock:
                    data = _jpeg
                if data:
                    header = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(data)
                    self.wfile.write(header + data + b"\r\n")
                threading.Event().wait(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_args: object) -> None:
        pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("config/settings.yaml"))
    parser.add_argument("--uart", help="serial port to send CAM packets to the STM32")
    parser.add_argument("--http", type=int, help="serve an MJPEG stream on this port")
    parser.add_argument("--no-window", action="store_true", help="do not open a window on the Pi screen")
    args = parser.parse_args()
    global _jpeg
    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    c = cfg["camera"]
    from tflite_runtime.interpreter import Interpreter
    interpreter = Interpreter(model_path=c["model_path"])
    interpreter.allocate_tensors()
    details = interpreter.get_input_details() + interpreter.get_output_details()
    detector = VisionWorker(c, StateStore(), threading.Event(), lambda _t: None, lambda _p: None, cfg["vehicle"]["target_standoff_m"])
    ser = None
    if args.uart:
        import serial
        ser = serial.Serial(args.uart, 115200, timeout=0.02)
    if args.http:
        threading.Thread(target=ThreadingHTTPServer(("0.0.0.0", args.http), _Stream).serve_forever, daemon=True).start()
        print(f"stream on http://0.0.0.0:{args.http}/")
    cap = cv2.VideoCapture(int(c["device"]))
    if not cap.isOpened():
        print("cannot open camera")
        return 1
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # a queue of old frames delays the servo loop
    pan, seq, last_sent, last_seen, fps_t, fps = 0.0, 0, 0.0, 0.0, monotonic(), 0.0
    last_log = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        h, w = frame.shape[:2]
        det = detector._detect(interpreter, details, frame)
        now = monotonic()
        text = "no person"
        if det:
            last_seen = now
            ymin, xmin, ymax, xmax = det.box
            dist = estimate_distance_bbox(det.box_height_frac, c["vertical_fov_deg"], c["person_height_m"])
            body = pan + det.bearing_cam_deg
            pan = track_pan(pan, det.bearing_cam_deg, c["pan_gain"], c["pan_deadband_deg"], c["pan_limit_deg"])
            cv2.rectangle(frame, (int(xmin * w), int(ymin * h)), (int(xmax * w), int(ymax * h)), (0, 255, 0), 2)
            text = f"person {det.confidence:.0%}  cam {det.bearing_cam_deg:+.1f}  body {body:+.1f}  ~{dist:.1f} m{' (cropped)' if det.cropped else ''}"
        elif now - last_seen > c["recenter_after_s"]:
            pan = 0.0 if abs(pan) <= c["recenter_step_deg"] else pan - c["recenter_step_deg"] * (1 if pan > 0 else -1)
        if ser and now - last_sent >= 0.1:
            seq = (seq + 1) & 0xFFFF
            ser.write(encode("CAM", seq, f"{pan:.1f}"))
            last_sent = now
        fps = 0.9 * fps + 0.1 / max(now - fps_t, 1e-3)
        fps_t = now
        if now - last_log >= 0.25:
            last_log = now
            info = f"cam={det.bearing_cam_deg:+6.1f} body={body:+6.1f} dist={dist:4.1f} conf={det.confidence:.2f}" if det else "no person"
            print(f"{strftime('%H:%M:%S')} {info} pan_cmd={pan:+6.1f} fps={fps:.1f}", flush=True)
        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        cv2.putText(frame, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"servo pan {pan:+.1f} deg   {fps:.1f} fps", (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        if args.http:
            done, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if done:
                with _jpeg_lock:
                    _jpeg = buf.tobytes()
        if not args.no_window:
            cv2.imshow("Human tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

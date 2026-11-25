"""Capture/inference off the radio thread: slow Pi 4 inference must not delay ACKs."""
import sys
import threading
import time
from collections import deque
from math import radians, tan
from pathlib import Path

FPS_WINDOW_S = 10.0  # as VisionWorker.fps of Phao 1

def rate(times, now):
    """Frames a second over the last FPS_WINDOW_S; None without any."""
    recent = [t for t in times if now - t <= FPS_WINDOW_S]
    if len(recent) < 2: return None
    return (len(recent) - 1) / (recent[-1] - recent[0])

def load_edge_model(rasp_pi):
    """Phao 1's YOLOv8n + MoveNet with Phao 1's own settings, so both buoys judge a frame the same way."""
    root = Path(rasp_pi).resolve()
    if str(root) not in sys.path: sys.path.append(str(root))
    from modules.edge_model import EdgeModel
    from utils.config import load_config
    try: from tflite_runtime.interpreter import Interpreter
    except ImportError: from ai_edge_litert.interpreter import Interpreter
    camera = load_config(root/"config/settings.yaml", [root/"config/boat_stm32.yaml"])["camera"]
    for key in ("model_path", "pose_model_path"):
        if camera.get(key): camera[key] = str(root/camera[key])
    return EdgeModel(Interpreter, camera), camera

def ranged(box, camera):
    """(bearing from the lens in degrees, positive right, distance in m) of a person box (ymin, xmin, ymax, xmax), by
    the box height; distance None when the frame cuts the box off. Rough (about +-30 %), a pinhole camera."""
    ymin, xmin, ymax, xmax = box
    bearing = ((xmin + xmax) / 2 - 0.5) * float(camera["horizontal_fov_deg"])
    if ymin < 0.02 or ymax > 0.98: return bearing, None
    span = 2 * tan(radians(float(camera["vertical_fov_deg"])) / 2) * max(ymax - ymin, 1e-3)
    return bearing, float(camera["person_height_m"]) / span

def assess(people):
    """(confidence, risk, person) of the person most at risk; (0.0, 0, None) with nobody to rescue (on a boat: safe)."""
    victims = [p for p in people if not p.on_boat]
    if not victims: return 0.0, 0, None
    worst = max(victims, key=lambda p: (p.risk, p.confidence))
    return worst.confidence, worst.risk, worst

class Camera:
    def __init__(self, cfg):
        self.cfg, self.error = cfg, None
        self.latest = (0.0, 0.0, 0, None)  # time, confidence (0: nobody), risk, (bearing, distance or None)
        self.times = deque(maxlen=400)       # when each detection finished, for fps
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        cap = None
        try:
            import cv2
            if self.cfg["backend"] == "picamera2":
                from picamera2 import Picamera2
                cap = Picamera2()
                cap.configure(cap.create_video_configuration(main={"size": (640, 480), "format": "RGB888"}))
                cap.start()
                read = lambda: cap.capture_array()
            else:
                # V4L2: the default (GStreamer, with apt's OpenCV) opens the camera at full size first.
                cap = cv2.VideoCapture(self.cfg.get("device", 0), cv2.CAP_V4L2)
                if not cap.isOpened(): raise RuntimeError("camera cannot open")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                def read():
                    ok, image = cap.read()
                    if not ok: raise RuntimeError("camera frame unavailable")
                    return image
            if self.cfg["detector"] == "yolo_movenet":
                model, shared = load_edge_model(self.cfg["rasp_pi"])
            elif self.cfg["detector"] != "hog":
                raise RuntimeError("camera.detector must be yolo_movenet or hog")
            else:
                hog = cv2.HOGDescriptor(); hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            while not self.stop.is_set():
                frame = read()
                if self.cfg["detector"] == "yolo_movenet":
                    score, risk, person = assess(model.detect(frame)[0])
                    where = ranged(person.box, shared) if person else None
                else:
                    boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8), padding=(8, 8), scale=1.05)
                    # HOG weights are NOT calibrated probabilities: hit or miss, nominal confidence and risk.
                    score, risk, where = (self.cfg["confidence"], 2, None) if len(boxes) else (0.0, 0, None)
                self.latest = (time.monotonic(), score, risk, where)
                self.times.append(self.latest[0])
                self.stop.wait(0.05)
        except Exception as exc:
            self.error = str(exc)
        finally:
            if cap is not None:
                if self.cfg["backend"] == "picamera2": cap.stop(); cap.close()
                else: cap.release()

    def fps(self, now):
        return None if self.error else rate(list(self.times), now)

    def close(self):
        self.stop.set(); self.thread.join(timeout=3)

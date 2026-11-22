"""Capture/inference off the radio thread: slow Pi 4 inference must not delay ACKs."""
import threading
import time

class Camera:
    def __init__(self, cfg):
        self.cfg, self.latest, self.error = cfg, (0.0, 0.0), None
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
                cap = cv2.VideoCapture(self.cfg.get("device", 0))
                if not cap.isOpened(): raise RuntimeError("camera cannot open")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                def read():
                    ok, image = cap.read()
                    if not ok: raise RuntimeError("camera frame unavailable")
                    return image
            if self.cfg["detector"] == "yolo":
                from ultralytics import YOLO
                from pathlib import Path
                model_path = Path(self.cfg["model"])
                if not model_path.is_file(): raise RuntimeError("YOLO model missing; set camera.model")
                model = YOLO(str(model_path))
            else:
                hog = cv2.HOGDescriptor(); hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            while not self.stop.is_set():
                frame = read()
                if self.cfg["detector"] == "yolo":
                    result = model.predict(frame, imgsz=320, classes=[0], verbose=False,
                                           conf=self.cfg["confidence"])[0]
                    score = float(result.boxes.conf.max()) if len(result.boxes) else 0.0
                else:
                    boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8), padding=(8, 8), scale=1.05)
                    # HOG weights are NOT calibrated probabilities. Use binary demo evidence.
                    score = 1.0 if len(boxes) else 0.0
                self.latest = (time.monotonic(), score)
                self.stop.wait(0.05)
        except Exception as exc:
            self.error = str(exc)
        finally:
            if cap is not None:
                if self.cfg["backend"] == "picamera2": cap.stop(); cap.close()
                else: cap.release()

    def close(self):
        self.stop.set(); self.thread.join(timeout=3)

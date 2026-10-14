import time
import cv2
import numpy as np

from src.config import (
    YOLO_MODEL_PATH, MOVENET_MODEL_PATH, YOLO_INPUT_SIZE, MOVENET_INPUT_SIZE,
    CONF_THRESH_YOLO, NMS_THRESH_YOLO, KEYPOINT_THRESH, KEYPOINT_EDGES
)
from src.utils import (
    letterbox, draw_detection_results, merge_overlapping_boxes, draw_fps_info
)
from src.yolo_detector import YOLOv8Detector
from src.movenet_pose import MoveNetPose
from src.risk_analyzer import RiskAnalyzer, KeypointSmoother

def main():
    print("==================================================")
    print("  Khởi tạo Hệ thống Edge AI Rescue (Rescue-Boat)  ")
    print("==================================================")

    # 1. Nạp các Model
    yolo = YOLOv8Detector(YOLO_MODEL_PATH, CONF_THRESH_YOLO, NMS_THRESH_YOLO)
    movenet = MoveNetPose(MOVENET_MODEL_PATH)
    smoother = KeypointSmoother(alpha=0.65)
    print("-> Đã nạp thành công YOLOv8 & MoveNet.")

    # 2. Khởi tạo Camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Lỗi: Không thể mở Camera!")
        return

    print("Bắt đầu luồng xử lý... Bấm 'q' để dừng.")

    fps_ema, yolo_ms_ema, movenet_ms_ema = 0.0, 0.0, 0.0
    alpha_ema = 0.15

    while True:
        t_frame_start = time.perf_counter()

        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        orig_h, orig_w = frame.shape[:2]

        # ================= STAGE 1: YOLO DETECTION =================
        t_yolo_start = time.perf_counter()

        img_320, ratio, pad_info = letterbox(frame, new_shape=YOLO_INPUT_SIZE)
        img_rgb = cv2.cvtColor(img_320, cv2.COLOR_BGR2RGB)

        detections = yolo.detect(img_rgb, (orig_h, orig_w), pad_info, ratio)
        detections = merge_overlapping_boxes(detections)

        t_yolo_end = time.perf_counter()
        yolo_ms = (t_yolo_end - t_yolo_start) * 1000.0

        # Tách danh sách BBox Thuyền để đối chiếu vị trí tương quan
        boat_boxes = [d for d in detections if int(d[2]) != 0]

        # ================= STAGE 2: POSE & RISK ANALYSIS =================
        t_movenet_start = time.perf_counter()
        
        for (box, conf, cls_id) in detections:
            x, y, w, h = box
            cls_id = int(cls_id)

            # 1. Thuyền (cls_id != 0): Vẽ BBox cam và qua môn
            if cls_id != 0:
                draw_detection_results(frame, x, y, w, h, conf, cls_id)
                continue

            # 2. Người (cls_id == 0): Kiểm tra xem có ở trên thuyền không
            if RiskAnalyzer.is_person_on_boat((x, y, w, h), boat_boxes):
                draw_detection_results(
                    frame, x, y, w, h, conf, cls_id,
                    risk_status="ON_BOAT_SAFE"
                )
                continue

            # 3. Nạn nhân dưới nước: Cắt ROI và chạy MoveNet
            top_pad = int(h * 0.3)
            side_pad = int(w * 0.15)
            
            rx1 = max(0, x - side_pad)
            ry1 = max(0, y - top_pad)
            rx2 = min(orig_w, x + w + side_pad)
            ry2 = min(orig_h, y + h)

            crop_img = frame[ry1:ry2, rx1:rx2]
            if crop_img.size == 0:
                continue

            crop_h, crop_w = crop_img.shape[:2]
            crop_resized = cv2.resize(crop_img, MOVENET_INPUT_SIZE)
            crop_rgb = cv2.cvtColor(crop_resized, cv2.COLOR_BGR2RGB)

            keypoints = movenet.estimate_pose(crop_rgb)
            keypoints = smoother.smooth(keypoints)

            risk_status, _ = RiskAnalyzer.evaluate_water_risk(keypoints)

            kpts_absolute = []
            for ky, kx, score in keypoints:
                abs_x = int(kx * crop_w + rx1)
                abs_y = int(ky * crop_h + ry1)
                kpts_absolute.append((abs_x, abs_y, score))

            draw_detection_results(
                frame, x, y, w, h, conf, cls_id,
                risk_status, kpts_absolute, KEYPOINT_EDGES, KEYPOINT_THRESH
            )

        t_movenet_end = time.perf_counter()
        movenet_ms = (t_movenet_end - t_movenet_start) * 1000.0 if len(detections) > 0 else 0.0

        # ================= TÍNH TOÁN FPS & HIỂN THỊ =================
        t_frame_end = time.perf_counter()
        total_frame_time = t_frame_end - t_frame_start
        current_fps = 1.0 / total_frame_time if total_frame_time > 0 else 0.0

        fps_ema = (1 - alpha_ema) * fps_ema + alpha_ema * current_fps if fps_ema > 0 else current_fps
        yolo_ms_ema = (1 - alpha_ema) * yolo_ms_ema + alpha_ema * yolo_ms if yolo_ms_ema > 0 else yolo_ms
        movenet_ms_ema = (1 - alpha_ema) * movenet_ms_ema + alpha_ema * movenet_ms if movenet_ms_ema > 0 else movenet_ms

        draw_fps_info(frame, fps_ema, yolo_ms_ema, movenet_ms_ema)

        gui_frame = cv2.resize(frame, (640, 640))
        cv2.imshow("EdgeAI Rescue Detection System", gui_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
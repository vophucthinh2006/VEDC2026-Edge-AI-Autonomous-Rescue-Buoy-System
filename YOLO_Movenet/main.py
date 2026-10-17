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
from src.risk_analyzer import BoxTracker, RiskAnalyzer, KeypointSmoother

def get_square_crop_coords(x, y, w, h, orig_w, orig_h, margin_factor=1.35):
    """
    Tạo Bounding Box hình vuông (Tỷ lệ 1:1) quanh người để MoveNet
    không bị méo tỷ lệ cơ thể khi resize.
    """
    cx = x + w / 2.0
    cy = y + h / 2.0
    square_size = max(w, h) * margin_factor

    rx1 = max(0, int(cx - square_size / 2.0))
    ry1 = max(0, int(cy - square_size / 2.0))
    rx2 = min(orig_w, int(cx + square_size / 2.0))
    ry2 = min(orig_h, int(cy + square_size / 2.0))

    return rx1, ry1, rx2, ry2

def main():
    print("==================================================")
    print("  Khởi tạo Hệ thống Edge AI Rescue (Rescue-Boat)  ")
    print("==================================================")

    # 1. Nạp các Model & Tracker
    yolo = YOLOv8Detector(YOLO_MODEL_PATH, CONF_THRESH_YOLO, NMS_THRESH_YOLO)
    movenet = MoveNetPose(MOVENET_MODEL_PATH)
    smoother = KeypointSmoother(alpha=0.65)
    tracker = BoxTracker(max_missing_frames=4) # Khởi tạo ngoài vòng lặp
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

        img_yolo, ratio, pad_info = letterbox(frame, new_shape=YOLO_INPUT_SIZE)
        img_rgb = cv2.cvtColor(img_yolo, cv2.COLOR_BGR2RGB)

        detections = yolo.detect(img_rgb, (orig_h, orig_w), pad_info, ratio)
        detections = merge_overlapping_boxes(detections)
        detections = tracker.update(detections) # Cập nhật vị trí BBox qua Tracker

        t_yolo_end = time.perf_counter()
        yolo_ms = (t_yolo_end - t_yolo_start) * 1000.0

        boat_boxes = [d for d in detections if int(d[2]) != 0]

        # ================= STAGE 2: POSE & RISK ANALYSIS =================
        t_movenet_start = time.perf_counter()
        
        for (box, conf, cls_id) in detections:
            x, y, w, h = box
            cls_id = int(cls_id)

            # 1. Thuyền
            if cls_id != 0:
                draw_detection_results(frame, x, y, w, h, conf, cls_id)
                continue

            # 2. Người ở trên thuyền
            if RiskAnalyzer.is_person_on_boat((x, y, w, h), boat_boxes):
                draw_detection_results(
                    frame, x, y, w, h, conf, cls_id,
                    risk_status="ON_BOAT_SAFE"
                )
                continue

            # 3. Nạn nhân dưới nước / Người đứng gần
            # Mở rộng BBox thành hình vuông (Ratio 1:1) chuẩn đầu vào MoveNet
            rx1, ry1, rx2, ry2 = get_square_crop_coords(x, y, w, h, orig_w, orig_h, margin_factor=1.35)

            crop_img = frame[ry1:ry2, rx1:rx2]
            if crop_img.size == 0:
                continue

            # Giữ nguyên tỷ lệ hình thể bằng letterbox thay vì resize làm bóp bẹt dáng người
            crop_padded, move_ratio, (move_dw, move_dh) = letterbox(crop_img, new_shape=MOVENET_INPUT_SIZE)
            crop_rgb = cv2.cvtColor(crop_padded, cv2.COLOR_BGR2RGB)

            keypoints = movenet.estimate_pose(crop_rgb)
            keypoints = smoother.smooth(keypoints)

            # Đánh giá nguy cơ (Truyền BBox & orig_shape để check viền dưới)
            risk_status, _ = RiskAnalyzer.evaluate_water_risk(
                keypoints, person_box=(x, y, w, h), orig_shape=(orig_h, orig_w)
            )

            # Quy đổi tọa độ keypoints từ MoveNet (192x192 letterbox) về tọa độ khung hình gốc
            kpts_absolute = []
            for ky, kx, score in keypoints:
                px_pad = kx * MOVENET_INPUT_SIZE[1]
                py_pad = ky * MOVENET_INPUT_SIZE[0]

                unpad_x = (px_pad - move_dw) / move_ratio
                unpad_y = (py_pad - move_dh) / move_ratio

                abs_x = int(unpad_x + rx1)
                abs_y = int(unpad_y + ry1)
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
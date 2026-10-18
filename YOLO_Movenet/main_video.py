import argparse
import os
import time
import cv2

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

# ==============================================================================
# 🛠️ CẤU HÌNH ĐƯỜNG DẪN & HIỂN THỊ
# ==============================================================================
DEFAULT_INPUT_VIDEO  = "videos/rescue_test1.mp4"
DEFAULT_OUTPUT_VIDEO = "videos/output_rescue1.640.mp4"
SHOW_GUI             = True
MAX_DISPLAY_WIDTH    = 960
MAX_DISPLAY_HEIGHT   = 750
# ==============================================================================

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

def process_video(video_path=DEFAULT_INPUT_VIDEO, output_path=DEFAULT_OUTPUT_VIDEO, display=SHOW_GUI):
    if not os.path.exists(video_path):
        print(f"Lỗi: Không tìm thấy file video tại '{video_path}'!")
        print("Vui lòng kiểm tra lại đường dẫn file trong phần CẤU HÌNH ở đầu file main_video.py.")
        return

    print("==================================================")
    print("  Khởi tạo Hệ thống Rescue Video Testing          ")
    print("==================================================")
    yolo = YOLOv8Detector(YOLO_MODEL_PATH, CONF_THRESH_YOLO, NMS_THRESH_YOLO)
    movenet = MoveNetPose(MOVENET_MODEL_PATH)
    smoother = KeypointSmoother(alpha=0.65)
    tracker = BoxTracker(max_missing_frames=4) # Khởi tạo ngoài vòng lặp
    print("-> Đã nạp thành công YOLOv8 INT8 & MoveNet INT8.")

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Lỗi: Không thể mở file video '{video_path}'!")
        return

    orig_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_video = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"-> Thông tin Video gốc: {orig_w}x{orig_h} | {fps_video:.1f} FPS | {total_frames} frames")

    writer = None
    if output_path:
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, fps_video, (orig_w, orig_h))
        print(f"-> Đã bật ghi file kết quả tại: {output_path}")

    print("\nBắt đầu xử lý... [Phím 'q': Thoát | Phím 'Space': Tạm dừng]\n")

    frame_count = 0
    total_processing_time = 0.0
    fps_ema, yolo_ms_ema, movenet_ms_ema = 0.0, 0.0, 0.0
    alpha_ema = 0.15
    paused = False

    scale = min(MAX_DISPLAY_WIDTH / orig_w, MAX_DISPLAY_HEIGHT / orig_h)
    gui_w = int(orig_w * scale)
    gui_h = int(orig_h * scale)

    while True:
        if not paused:
            t_frame_start = time.perf_counter()

            ret, frame = cap.read()
            if not ret:
                print("\n-> Đã xử lý xong toàn bộ video!")
                break

            frame_count += 1

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
                rx1, ry1, rx2, ry2 = get_square_crop_coords(x, y, w, h, orig_w, orig_h, margin_factor=1.35)

                crop_img = frame[ry1:ry2, rx1:rx2]
                if crop_img.size == 0:
                    continue

                crop_padded, move_ratio, (move_dw, move_dh) = letterbox(crop_img, new_shape=MOVENET_INPUT_SIZE)
                crop_rgb = cv2.cvtColor(crop_padded, cv2.COLOR_BGR2RGB)

                keypoints = movenet.estimate_pose(crop_rgb)
                keypoints = smoother.smooth(keypoints)

                # Đánh giá nguy cơ (Truyền BBox & orig_shape để check viền dưới)
                risk_status, _ = RiskAnalyzer.evaluate_water_risk(
                    keypoints, person_box=(x, y, w, h), orig_shape=(orig_h, orig_w)
                )

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
            movenet_ms = (t_movenet_start - t_movenet_start) * 1000.0 if len(detections) > 0 else 0.0

            # ================= TÍNH TOÁN FPS & HIỂN THỊ =================
            t_frame_end = time.perf_counter()
            frame_time = t_frame_end - t_frame_start
            total_processing_time += frame_time
            current_fps = 1.0 / frame_time if frame_time > 0 else 0.0

            fps_ema = (1 - alpha_ema) * fps_ema + alpha_ema * current_fps if fps_ema > 0 else current_fps
            yolo_ms_ema = (1 - alpha_ema) * yolo_ms_ema + alpha_ema * yolo_ms if yolo_ms_ema > 0 else yolo_ms
            movenet_ms_ema = (1 - alpha_ema) * movenet_ms_ema + alpha_ema * movenet_ms if movenet_ms_ema > 0 else movenet_ms

            draw_fps_info(frame, fps_ema, yolo_ms_ema, movenet_ms_ema)

            if writer:
                writer.write(frame)

            progress = (frame_count / total_frames) * 100 if total_frames > 0 else 0
            print(f"\rProgress: [{frame_count}/{total_frames}] {progress:.1f}% | Process FPS: {fps_ema:.1f}", end="")

        if display:
            gui_frame = cv2.resize(frame, (gui_w, gui_h))
            cv2.imshow("Rescue Detection System - Video Test", gui_frame)

            key = cv2.waitKey(1 if not paused else 0) & 0xFF
            if key == ord('q'):
                print("\n-> Đã dừng chương trình bằng phím 'q'.")
                break
            elif key == ord(' '):
                paused = not paused
                print("\n[PAUSED]" if paused else "\n[RESUMED]")

    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()

    if frame_count > 0:
        avg_fps = frame_count / total_processing_time
        print("\n==================================================")
        print(f"  Thống kê hoàn tất: {frame_count} frames")
        print(f"  FPS xử lý trung bình: {avg_fps:.2f} FPS")
        print("==================================================")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Chạy kiểm thử hệ thống cứu nạn trên File Video")
    parser.add_argument("--input", type=str, default=DEFAULT_INPUT_VIDEO, help="Đường dẫn file video đầu vào")
    parser.add_argument("--output", type=str, default=DEFAULT_OUTPUT_VIDEO, help="Đường dẫn lưu file video đầu ra")
    parser.add_argument("--no-display", action="store_true", help="Tắt cửa sổ GUI")

    args = parser.parse_args()
    
    out_file = None if args.no_display and not args.output else args.output
    process_video(args.input, output_path=out_file, display=not args.no_display)
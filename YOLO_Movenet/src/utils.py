import cv2
import numpy as np

# Bảng tên Class (0: Person, 1: Boat)
CLASS_NAMES = {0: "Person", 1: "Boat"}

def letterbox(img, new_shape=(320, 320), color=(114, 114, 114)):
    """Đệm ảnh (padding) giữ nguyên tỷ lệ không bị móp méo"""
    shape = img.shape[:2]
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    
    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    
    img_padded = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img_padded, r, (dw, dh)


def merge_overlapping_boxes(detections, iou_thresh=0.05, overlap_thresh=0.15, margin=15):
    """
    Gộp các Bounding Box CÙNG CLASS bị tách nhỏ (thân + 2 tay) thành 1 Box duy nhất.
    - iou_thresh=0.05: Giảm xuống cực thấp, chỉ cần giao nhẹ là gộp.
    - overlap_thresh=0.15: Chỉ cần 15% diện tích box nhỏ nằm sát box lớn là gộp.
    - margin=15: Nới rộng viền 15px để 'hút' các box tay/chân tách rời kề bên.
    """
    if not detections:
        return []

    # Chuyển đổi về dạng [x1, y1, x2, y2, conf, cls_id]
    current_boxes = []
    for box, conf, cls_id in detections:
        x, y, w, h = box
        current_boxes.append([x, y, x + w, y + h, float(conf), int(cls_id)])

    # Gộp đa vòng (Multi-pass) cho đến khi không còn box nào có thể gộp thêm
    has_merged = True
    while has_merged:
        has_merged = False
        new_boxes = []
        visited = [False] * len(current_boxes)

        for i in range(len(current_boxes)):
            if visited[i]:
                continue

            b1 = current_boxes[i]
            group = [b1]
            visited[i] = True

            for j in range(i + 1, len(current_boxes)):
                if visited[j]:
                    continue

                b2 = current_boxes[j]

                # BẮT BUỘC CÙNG CLASS (Person với Person, Boat với Boat)
                if b1[5] != b2[5]:
                    continue

                # Mở rộng tạm thời viền của 2 box theo margin để quét các chi tiết nằm sát kề
                e1 = [b1[0] - margin, b1[1] - margin, b1[2] + margin, b1[3] + margin]
                e2 = [b2[0] - margin, b2[1] - margin, b2[2] + margin, b2[3] + margin]

                # Tính diện tích giao nhau của 2 vùng mở rộng
                xx1 = max(e1[0], e2[0])
                yy1 = max(e1[1], e2[1])
                xx2 = min(e1[2], e2[2])
                yy2 = min(e1[3], e2[3])

                w_inter = max(0, xx2 - xx1)
                h_inter = max(0, yy2 - yy1)
                area_inter = w_inter * h_inter

                # Diện tích gốc (không margin)
                area_b1 = max(1, (b1[2] - b1[0]) * (b1[3] - b1[1]))
                area_b2 = max(1, (b2[2] - b2[0]) * (b2[3] - b2[1]))
                area_union = area_b1 + area_b2 - area_inter

                iou = area_inter / area_union if area_union > 0 else 0
                overlap_ratio = area_inter / min(area_b1, area_b2)

                # Nếu chạm nhẹ hoặc nằm trong khoảng margin 15px -> Gộp group
                if iou > iou_thresh or overlap_ratio > overlap_thresh:
                    group.append(b2)
                    visited[j] = True
                    has_merged = True

            # Gom group thành 1 Box duy nhất bao trọn từ đầu đến tay/chân
            gx1 = min(b[0] for b in group)
            gy1 = min(b[1] for b in group)
            gx2 = max(b[2] for b in group)
            gy2 = max(b[3] for b in group)
            max_conf = max(b[4] for b in group)
            cls_id = group[0][5]

            new_boxes.append([gx1, gy1, gx2, gy2, max_conf, cls_id])

        current_boxes = new_boxes

    # Trả về kết quả chuẩn dạng [ [x, y, w, h], conf, cls_id ]
    results = []
    for b in current_boxes:
        w = b[2] - b[0]
        h = b[3] - b[1]
        results.append(([b[0], b[1], w, h], b[4], b[5]))

    return results


def draw_detection_results(frame, x, y, w, h, conf, cls_id, risk_status=None, kpts_absolute=None, edges=None, kpt_thresh=0.25):
    """Vẽ Bounding Box, Khung xương và nhãn cảnh báo phân màu theo trạng thái."""
    orig_h = frame.shape[0]
    
    cls_id = int(cls_id)
    class_name = CLASS_NAMES.get(cls_id, f"Class {cls_id}")

    # ==================== 1. TRƯỜNG HỢP: THUYỀN (BOAT) ====================
    if cls_id != 0:
        label_text = f"{class_name}: {float(conf):.2f}"
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 165, 0), 2)  # Màu Cam
        cv2.putText(frame, label_text, (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 165, 0), 2)
        return

    # ==================== 2. TRƯỜNG HỢP: NGƯỜI (PERSON) ====================
    # Gán màu sắc Bounding Box và Khung xương tương ứng với vị trí / rủi ro
    if risk_status == "ON_BOAT_SAFE":
        box_color = (0, 255, 0)         # Xanh lá - Trên thuyền
        label_text = f"Person (ON BOAT): {float(conf):.2f}"
        skeleton_color = (0, 255, 0)
    elif risk_status == "IN_WATER_HIGH_RISK":
        box_color = (0, 0, 255)         # Đỏ - Dưới nước nguy hiểm
        label_text = f"IN WATER: {float(conf):.2f}"
        skeleton_color = (0, 0, 255)
    elif risk_status == "NEAR_CAMERA_SAFE":
        box_color = (0, 255, 255)       # Vàng - Đứng cận cảnh
        label_text = f"Person (NEAR): {float(conf):.2f}"
        skeleton_color = (0, 255, 255)
    else:  # NORMAL_VISIBLE hoặc mặc định trên bờ
        box_color = (255, 255, 0)       # Cyan/Xanh lơ - Trên bờ
        label_text = f"Person (ON SHORE): {float(conf):.2f}"
        skeleton_color = (255, 255, 0)

    # Vẽ Bounding Box với màu tương ứng
    cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
    cv2.putText(frame, label_text, (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

    # Cảnh báo dòng chữ đỏ bên dưới BBox nếu nạn nhân gặp rủi ro dưới nước
    if risk_status == "IN_WATER_HIGH_RISK":
        text_y = y + h + 25 if (y + h + 30) < orig_h else max(30, y - 30)
        cv2.putText(frame, "IN WATER / NO LEGS", (x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Vẽ Keypoints & Khung xương MoveNet
    if kpts_absolute:
        for abs_x, abs_y, score in kpts_absolute:
            if score > kpt_thresh:
                cv2.circle(frame, (abs_x, abs_y), 4, (0, 255, 255), -1)

        if edges:
            for p1_idx, p2_idx in edges:
                x1, y1, s1 = kpts_absolute[p1_idx]
                x2, y2, s2 = kpts_absolute[p2_idx]

                if s1 > kpt_thresh and s2 > kpt_thresh:
                    cv2.line(frame, (x1, y1), (x2, y2), skeleton_color, 2)


def draw_fps_info(frame, fps, yolo_ms=0.0, movenet_ms=0.0):
    """Vẽ bảng thông số FPS và Latency chi tiết lên góc trên bên trái khung hình"""
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (210, 85), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    cv2.rectangle(frame, (10, 10), (210, 85), (255, 255, 255), 1)

    fps_color = (0, 255, 0) if fps >= 15 else (0, 165, 255) if fps >= 8 else (0, 0, 255)
    cv2.putText(frame, f"FPS: {fps:.1f}", (20, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, fps_color, 2)

    cv2.putText(frame, f"YOLO   : {yolo_ms:.1f} ms", (20, 54),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(frame, f"MoveNet: {movenet_ms:.1f} ms", (20, 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
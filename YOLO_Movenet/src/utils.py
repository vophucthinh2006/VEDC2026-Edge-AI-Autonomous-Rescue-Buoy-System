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


def draw_detection_results(frame, x, y, w, h, conf, cls_id, risk_status=None, kpts_absolute=None, edges=None, kpt_thresh=0.25):
    """
    Vẽ Bounding Box & Khung xương.
    - Nếu là Boat (cls_id != 0): Chỉ vẽ Bounding Box màu cam, bỏ qua khung xương và cảnh báo.
    - Nếu là Person (cls_id == 0): 
        + Nếu ON_BOAT_SAFE: Vẽ Bounding Box xanh lá Person (SAFE), không cảnh báo.
        + Nếu dưới nước: Vẽ Bounding Box xanh lá + Khung xương + Cảnh báo nguy cơ nếu chìm chân.
    """
    orig_h = frame.shape[0]
    
    cls_id = int(cls_id)
    class_name = CLASS_NAMES.get(cls_id, f"Class {cls_id}")
    label_text = f"{class_name}: {float(conf):.2f}"

    # ==================== TRƯỜNG HỢP: THUYỀN (BOAT) ====================
    if cls_id != 0:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 165, 0), 2)
        cv2.putText(frame, label_text, (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 165, 0), 2)
        return  # Kết thúc sớm đối với Thuyền

    # ==================== TRƯỜNG HỢP: NGƯỜI (PERSON) ====================
    # 1. Người đã an toàn trên thuyền
    if risk_status == "ON_BOAT_SAFE":
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f"Person (SAFE): {float(conf):.2f}", (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        return

    # 2. Người dưới nước / Kiểm tra Pose
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.putText(frame, label_text, (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    # Định màu sắc khung xương dựa trên trạng thái rủi ro
    skeleton_color = (0, 0, 255) if risk_status == "IN_WATER_HIGH_RISK" else (0, 255, 255)

    if risk_status == "IN_WATER_HIGH_RISK":
        text_y = y + h + 25 if (y + h + 30) < orig_h else max(30, y - 30)
        cv2.putText(frame, "IN WATER / NO LEGS", (x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Vẽ điểm khớp
    if kpts_absolute:
        for abs_x, abs_y, score in kpts_absolute:
            if score > kpt_thresh:
                cv2.circle(frame, (abs_x, abs_y), 4, (0, 255, 255), -1)

        # Vẽ các đoạn xương nối
        if edges:
            for p1_idx, p2_idx in edges:
                x1, y1, s1 = kpts_absolute[p1_idx]
                x2, y2, s2 = kpts_absolute[p2_idx]

                if s1 > kpt_thresh and s2 > kpt_thresh:
                    cv2.line(frame, (x1, y1), (x2, y2), skeleton_color, 2)


def merge_overlapping_boxes(detections, iou_thresh=0.2, distance_ratio=0.15):
    """
    Gộp các Bounding Box CÙNG CLASS đè lên nhau.
    detections: danh sách tuple [(box, conf, cls_id), ...]
    """
    if not detections:
        return []

    boxes = []
    for box, conf, cls_id in detections:
        x, y, w, h = box
        boxes.append([x, y, x + w, y + h, conf, int(cls_id)])

    merged_results = []
    visited = [False] * len(boxes)

    for i in range(len(boxes)):
        if visited[i]:
            continue

        group = [boxes[i]]
        visited[i] = True

        for j in range(i + 1, len(boxes)):
            if visited[j]:
                continue

            b1, b2 = boxes[i], boxes[j]
            
            # CHỈ GỘP NẾU CÙNG CLASS
            if b1[5] != b2[5]:
                continue

            xx1 = max(b1[0], b2[0])
            yy1 = max(b1[1], b2[1])
            xx2 = min(b1[2], b2[2])
            yy2 = min(b1[3], b2[3])

            w_inter = max(0, xx2 - xx1)
            h_inter = max(0, yy2 - yy1)
            area_inter = w_inter * h_inter

            area_b1 = (b1[2] - b1[0]) * (b1[3] - b1[1])
            area_b2 = (b2[2] - b2[0]) * (b2[3] - b2[1])
            area_union = area_b1 + area_b2 - area_inter

            iou = area_inter / area_union if area_union > 0 else 0
            overlap_ratio = area_inter / min(area_b1, area_b2) if min(area_b1, area_b2) > 0 else 0

            if iou > iou_thresh or overlap_ratio > 0.4:
                group.append(boxes[j])
                visited[j] = True

        gx1 = min(b[0] for b in group)
        gy1 = min(b[1] for b in group)
        gx2 = max(b[2] for b in group)
        gy2 = max(b[3] for b in group)
        max_conf = max(b[4] for b in group)
        cls_id = group[0][5]

        merged_results.append(([gx1, gy1, gx2 - gx1, gy2 - gy1], max_conf, cls_id))

    return merged_results


def crop_person_square(frame, x, y, w, h, pad_ratio=0.3):
    """Tạo ROI hình vuông quanh đối tượng kèm padding mở rộng."""
    orig_h, orig_w = frame.shape[:2]
    cx, cy = x + w / 2, y + h / 2
    max_side = max(w, h) * (1.0 + pad_ratio)

    rx1 = int(max(0, cx - max_side / 2))
    ry1 = int(max(0, cy - max_side / 2))
    rx2 = int(min(orig_w, cx + max_side / 2))
    ry2 = int(min(orig_h, cy + max_side / 2))

    crop = frame[ry1:ry2, rx1:rx2]
    ch, cw = crop.shape[:2]
    max_c = max(ch, cw)
    square_img = np.zeros((max_c, max_c, 3), dtype=np.uint8)
    
    dx = (max_c - cw) // 2
    dy = (max_c - ch) // 2
    square_img[dy:dy+ch, dx:dx+cw] = crop

    return square_img, (rx1 - dx, ry1 - dy, max_c, max_c)


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
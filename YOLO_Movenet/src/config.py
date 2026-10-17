import os

# ==================== CẤU HÌNH ĐƯỜNG DẪN (PATHS) ====================
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEIGHTS_DIR = os.path.join(BASE_DIR, "weights")

# Đường dẫn mô hình
YOLO_MODEL_PATH = os.path.join(WEIGHTS_DIR, "yolov8n_640_int8.tflite")
MOVENET_MODEL_PATH = os.path.join(WEIGHTS_DIR, "movenet_lighting_int8.tflite")

# ==================== KÍCH THƯỚC ĐẦU VÀO (INPUT SIZES) ====================
YOLO_INPUT_SIZE = (640, 640)
MOVENET_INPUT_SIZE = (192, 192)

# ==================== NGƯỠNG LỌC (THRESHOLDS) ====================
# Stage 1: YOLO
CONF_THRESH_YOLO = 0.25      # Ngưỡng nhận diện đối tượng (BBox)
NMS_THRESH_YOLO = 0.40       # Ngưỡng đè Bounding Box (NMS)

# Stage 2: MoveNet & Risk Assessment
KEYPOINT_THRESH = 0.25           # Ngưỡng độ tin cậy tối thiểu để vẽ từng khớp
LOWER_BODY_WATER_THRESH = 0.25   # Ngưỡng tin cậy trung bình thân dưới (nhỏ hơn -> chìm dưới nước)

# ==================== CẤU TRÚC KHỚP XƯƠNG (KEYPOINTS) ====================
# 17 khớp chuẩn COCO của MoveNet
KEYPOINT_EDGES = [
    (0, 1), (0, 2), (1, 3), (2, 4),        # Đầu / Mặt
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10), # Thân trên & Tay
    (5, 11), (6, 12), (11, 12),            # Thân giữa
    (11, 13), (13, 15), (12, 14), (14, 16)   # Thân dưới & Chân
]

LOWER_BODY_INDICES = [11, 12, 13, 14, 15, 16] # Hông, Gối, Cổ chân
UPPER_BODY_INDICES = [5, 6, 7, 8, 9, 10]     # Vai, Khuỷu tay, Cổ tay
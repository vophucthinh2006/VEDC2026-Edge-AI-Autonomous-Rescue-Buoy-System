# RedgeSCUE Raspberry Pi runtime

## Cài đặt trên Raspberry Pi 4

```bash
cd Rasp_Pi
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python3 main.py --no-camera
```

Model nhận diện là **SSD MobileNet v1 COCO (quantized)** của TensorFlow, không lưu trong git (thư mục
`models/` nằm trong `.gitignore`). Tải và giải nén vào `models/`, được `detect.tflite` và `labelmap.txt`
(lớp `person` là lớp số 0):

```bash
mkdir -p models && cd models
curl -LO https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip && cd ..
```

Trỏ `camera.model_path` trong `config/settings.yaml` tới file đó. Trên Pi dùng `tflite-runtime`; trên
Python 3.12 (giả lập trong WSL) thư viện này không có bản cài, `ai_vision.py` tự chuyển sang
`ai-edge-litert` (`pip install ai-edge-litert`).

Sau khi có model, bỏ `--no-camera` để bật AI. Chạy lần đầu trên giá treo, không nối motor; STM32 phải có watchdog `NAV` TTL trước khi thử nước.

## Hai chương trình chính

| | `main.py` | `rescue_main.py` |
|---|---|---|
| Bộ điều khiển | STM32F407 tự viết, giao thức `$NAV/$IMU` qua UART | **Pixhawk + ArduPilot Rover**, MAVLink qua TELEM2 |
| Pi làm gì | tự điều hướng và tránh vật cản, ra lệnh hướng + tốc độ liên tục | chỉ lo AI và quyết định cứu hộ; autopilot lái, điều hướng, giữ failsafe |

`rescue_main.py` (xem `modules/rescue.py`): khi autopilot đang chạy lộ trình quét (AUTO) và camera thấy người
đủ `rescue.confirm_s` giây, Pi chuyển GUIDED đưa phao tới cách người `target_standoff_m`, chuyển LOITER giữ
`rescue.hold_s` giây, báo về trạm bằng `STATUSTEXT`, rồi trả lại AUTO. Người đã tiếp cận được gửi cho
autopilot như một vùng cấm (`modules/obstacle_feed.py`, bản tin `OBSTACLE_DISTANCE`). Bất kỳ ai đổi chế độ
(tay điều khiển, trạm mặt đất) thì Pi nhả quyền ngay.

```bash
python3 rescue_main.py                              # trên phao: /dev/serial0, camera USB
python3 rescue_main.py --overlay config/sim.yaml    # trong giả lập (sim/run_boat.sh --pi làm sẵn việc này)
```

Chưa có trong `rescue_main.py`: đọc LiDAR (module LDS-008 dùng giao thức khác LD14, chưa viết bộ giải mã),
điều khiển servo camera và LoRa qua bo STM32F103.

## Kết nối

- STM32: `/dev/serial0`, 115200 8N1, GPIO14/15.
- LD14: `/dev/ttyAMA3`, 115200 8N1, chỉ nối LD14 TX -> Pi GPIO5. Bật `enable_uart=1` và `dtoverlay=uart3` trong boot configuration; tắt serial login console trên UART STM32.
- Hiệu chuẩn `vehicle.lidar_yaw_offset_deg` để góc 0 của LD14 đúng hướng mũi phao.

## Hành vi phát hiện người

AI chỉ tạo target khi lớp COCO `person` đạt confidence cấu hình **và** một cụm LiDAR ở cùng góc camera. Khi khóa được, Pi gửi `TXD,VICTIM_FOUND,lat,lon,confidence` (không ảnh) và điều hướng đến khoảng cách `target_standoff_m`; không ghép được LiDAR thì không tiến lại gần.

## Kiểm tra nhanh

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile main.py modules/*.py utils/*.py
```

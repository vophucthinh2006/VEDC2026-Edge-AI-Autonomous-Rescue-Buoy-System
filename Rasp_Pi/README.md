# RedgeSCUE Raspberry Pi runtime

## Cài đặt trên Raspberry Pi 4

```bash
cd Rasp_Pi
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python3 main.py --no-camera
```

Sau khi chép đúng `models/person_detector.tflite`, bỏ `--no-camera` để bật AI. Chạy lần đầu trên giá treo, không nối motor; STM32 phải có watchdog `NAV` TTL trước khi thử nước.

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

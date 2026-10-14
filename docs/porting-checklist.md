# Đưa kiến trúc `pixhawk` lên phao thật

Làm theo thứ tự. Mỗi bước có lệnh và **giá trị phải thấy**; lệch thì xử lý theo cột cuối rồi mới đi
tiếp. Tháo chân vịt cho tới hết bước 7.

Những thứ đã chạy trong giả lập với đúng đường dữ liệu của phao (`sim/tools/hw_path_test.sh`): Pi ↔ bo
phụ (lệnh servo, số người, vị trí, báo cáo pin/nút dừng), logic firmware của bo, LiDAR qua cổng nối tiếp
tới bộ né vật cản của autopilot, dòng LoRa tới dashboard. Những thứ **chỉ kiểm được trên phần cứng** được
đánh dấu ⚠.

## 0. Chuẩn bị trên máy tính

```
boat pixhawk test          # unit test Pi + test firmware F103, phải ra [TEST] OK
boat pixhawk build
copy boat.local.example.cmd boat.local.cmd     # rồi sửa PI_HOST, PI_DIR
```

## 1. Bo phụ STM32F103, chưa nối gì ngoài ST-LINK

Cấp nguồn bằng 3.3 V của ST-LINK (4 dây), **không** cắm đồng thời 5 V ngoài.

```
boat pixhawk flash
python STM32F103\Tools\helper_monitor.py
```

| Phải thấy | Nếu không |
|---|---|
| LED PC13 nháy nhanh (5 lần/giây: chưa có Pi) | LED đứng yên: firmware không chạy, nạp lại; kiểm tra thạch anh 8 MHz |
| `Pi SILENT (0 lines, 0 bad)` | — |
| `servo 1500 us` | — |
| `LoRa chip 0x00 INIT FAILED` khi chưa cắm RA-02 | — |
| `INA226 not found` khi chưa cắm INA226 | — |

## 2. Nối từng ngoại vi vào bo phụ, tắt nguồn trước khi cắm

Sau mỗi món, bật nguồn lại và xem `helper_monitor.py`.

| Nối | Phải thấy | Nếu không |
|---|---|---|
| RA-02 **đã gắn anten 433 MHz**: SCK PA5, MISO PA6, MOSI PA7, NSS PA4, RST PB0, 3.3 V, GND | `LoRa chip 0x12 idle`, số `sent` tăng 1 mỗi 5 giây, `failed` đứng yên | `0x00`/`0xFF`: sai dây SPI hoặc NSS, thiếu 3.3 V. `failed` tăng: RST hoặc nguồn yếu (RA-02 hút ~120 mA khi phát, đừng lấy từ ST-LINK) |
| ⚠ Trạm bờ ESP32 đang chạy | log trạm có `[NHAN] id=PHAO-01 khong co toa do hop le` (bo chưa có vị trí), dashboard hiện `PHAO-01` "Chưa có GPS" | không nhận: so tham số radio ở `STM32F103/App/app_config.h` với `esp32-lora-station/main/main.c` |
| ⚠ INA226: SCL PB6, SDA PB7, 3.3 V/5 V, GND; IN+ / IN− nối tiếp đường 12 V | `battery 12.xx V +x.xx A`, đúng với đồng hồ trong khoảng 0.1 V | `INA226 not found`: địa chỉ không phải 0x44 (xem chân A0/A1), thiếu pull-up, hoặc bus này đang cắm chung với Pixhawk — hai master trên một bus I2C sẽ tranh nhau, tách ra |
| ⚠ Servo camera: tín hiệu PA2, nguồn servo riêng 5 V, chung GND | servo về giữa | không nhúc nhích: thiếu GND chung |
| ⚠ Relay: IN1 PA1, IN2 PA0 | đèn **tắt** lúc bình thường | đèn sáng suốt: module relay của bạn kích mức cao, đổi `RELAY_ON_LEVEL` trong `app_config.h` |
| ⚠ Nút dừng: PB9 xuống GND | nhấn giữ: hiện `E-STOP`; thả: mất | luôn `E-STOP`: nút loại thường đóng, đổi `ESTOP_PRESSED_LEVEL` |
| ⚠ Receiver FS-iA6B cổng **i-BUS SENS** → PB10 | `handset polls` tăng; trên tay FS-i6 hiện thêm cảm biến điện áp (Ext.V) bằng áp pin | `polls` đứng yên: cắm nhầm cổng i-BUS SERVO, hoặc giao thức khác giả định (phần này viết theo mô tả phổ biến của FlySky, **chưa từng chạy với receiver thật**) — ghi lại và báo |

## 3. Raspberry Pi ↔ bo phụ

Mạch USB-to-TTL 3.3 V: TXD → PA10, RXD → PA9, GND chung. Trên Pi phải có `/dev/ttyUSB0`.

```
boat pixhawk deploy
boat pixhawk run --no-scan
```

Chưa có Pixhawk nên chương trình dừng ở `waiting for the autopilot`; vẫn đủ để thử đường này:

| Phải thấy | Nếu không |
|---|---|
| Pi log: `helper board UART open: /dev/ttyUSB0` | `helper board unavailable`: sai cổng, sửa `helper.port` trong `Rasp_Pi/config/boat_pixhawk.yaml` |
| `helper_monitor.py`: vẫn `Pi SILENT` (Pi chỉ gửi khi đã thấy autopilot) | `bad` tăng: đảo TXD/RXD hoặc sai baud |

## 4. LiDAR LDS-008 vào Pi ⚠

Dây theo `Rasp_Pi/README.md` (mục Kết nối), `dtoverlay=uart3`.

```
python3 tools/lidar_capture.py
```

| Phải thấy | Nếu không |
|---|---|
| `... decoded` khác 0 và `points with a range` khác 0 khi có vật trong vài mét | `0 decoded`: giao thức của LDS-008 khác model LDS-006 mà bộ giải mã dựa vào. Giữ file `lidar_capture.bin` và gửi lại để viết lại bộ giải mã. Tạm thời đặt `lidar.source: none` trong `boat_pixhawk.yaml` và nạp tham số không có `PRX1_TYPE` (phao sẽ **không né vật cản**) |

## 5. Pixhawk

1. Nạp ArduPilot Rover, nạp `pixhawk/params/boat.parm` (đọc phần chú thích đầu file trước).
2. Pi nối cổng TELEM ghi trong file tham số; khởi động lại Pixhawk.
3. `boat pixhawk run --viewer --log`, mở `http://<địa chỉ Pi>:8090/`.

| Phải thấy | Nếu không |
|---|---|
| Pi log: `autopilot heard: system 1` | không thấy: sai cổng/baud TELEM, hoặc `SERIALn_PROTOCOL` chưa là 2 |
| log 5 giây một lần: `... | board 12.3 V lora ok` | `board off`: quay lại bước 3 |
| `helper_monitor.py`: `Pi OK`, số `lines` tăng, `bad` = 0 | — |
| viewer: ảnh camera; đặt vật trước mũi phao 1–2 m thì chấm trắng nằm **phía trên** hình | lệch hướng: chỉnh `vehicle.lidar_yaw_offset_deg`; ngược chiều: `vehicle.lidar_anticlockwise: true` |
| Mission Planner không báo `PRX1: No Data` | báo: Pi không đọc được LiDAR, xem bước 4 |
| dashboard: `PHAO-01` có vị trí khi Pixhawk có GPS fix | — |

## 6. Kiểm tra chức năng, vẫn trên bàn

| Làm | Phải thấy | Nếu không |
|---|---|---|
| Arm, chuyển AUTO với một lộ trình bất kỳ; để trống trước camera | servo quét trái–phải | đứng yên: Pi chỉ quét khi **đã arm và ở AUTO** |
| Đứng trước camera 2–4 m, rồi bước ngang | viewer có khung xanh; servo quay **theo** người | quay ngược: đổi `CAM_PAN_DIR` trong `STM32F103/App/app_config.h`, build, nạp |
| Đứng yên trước camera hơn 1 giây khi đang AUTO | Messages: `VICTIM SEEN ...`; chế độ sang GUIDED; **đèn relay bật 30 giây**; dashboard báo `PHAO-01 phát hiện người` | không có `VICTIM SEEN`: xem khung trong viewer, điểm tin cậy dưới 0.60 thì thiếu sáng/quá xa |
| Nhấn nút dừng khẩn cấp | Messages: `ESTOP PRESSED`, chế độ sang HOLD | — |
| Rút dây LiDAR | Pi log `obstacle feed stopped`; Mission Planner báo lỗi proximity. Cắm lại thì hết | — |
| Rút USB của bo phụ | Pi vẫn chạy, log `board off`; cắm lại thì tự nối sau ≤ 3 giây | — |
| Gạt tay điều khiển sang chế độ khác lúc Pi đang GUIDED | Pi nhả quyền ngay (log `phase ... -> search`) | — |

## 7. Trước khi xuống nước

- Kiểm tra chiều quay motor và chiều bánh lái bằng tay điều khiển ở MANUAL, chân vịt vẫn tháo.
- Thử failsafe: tắt tay điều khiển, phao phải làm đúng điều đã cấu hình.
- **Chỉ chạy vùng nước trống.** Hai ESC sau một chiều: trong giả lập phao vẫn cứu hộ tốt trên hồ trống nhưng
  va vật cản trong phố hẹp (`sim/README.md`, mục "ESC sau một chiều"). `boat.parm` đặt sẵn 0.6 m/s.
- Lắp chân vịt. Lộ trình đầu tiên: vùng nước trống, vài waypoint, không có người — chỉ để xem phao bám
  tuyến và chỉnh `ATC_STR_RAT_*`, `CRUISE_*` (các số trong giả lập **không** dùng cho phao thật).
- Luôn chạy với `--log`: file `logs/run_<ngày>_<giờ>.csv` trên Pi ghi mỗi giây chế độ, vị trí, hướng,
  pha cứu hộ, người đang thấy, vật cản gần nhất, tuổi vòng quét LiDAR, pin và trạng thái bo phụ.

## Số liệu nên mang về sau buổi thử

| Số liệu | Lấy ở đâu | Dùng để |
|---|---|---|
| `lidar_capture.bin` | bước 4 | xác nhận / viết lại bộ giải mã LDS-008 |
| `logs/run_*.csv` | Pi | chỉnh tốc độ quét, ngưỡng, so với giả lập |
| `helper_monitor.py --csv` | máy tính, qua ST-LINK | tỉ lệ gói Pi hỏng, số beacon LoRa gửi/lỗi |
| tầm phát hiện người thật (mét) ở ngưỡng 0.60 | viewer | khoảng cách giữa các tuyến quét |
| RSSI/SNR theo khoảng cách | dashboard | tầm LoRa thực tế |
| tốc độ khung hình của bộ phát hiện | `tools/cam_view.py` (dòng `fps=`) | `camera.max_fps` của giả lập đang giả định 8 |

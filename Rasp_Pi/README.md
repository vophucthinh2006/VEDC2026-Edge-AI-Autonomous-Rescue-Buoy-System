# RedgeSCUE Raspberry Pi runtime

## Cài đặt trên Raspberry Pi 4

```bash
cd Rasp_Pi
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python3 main.py --no-camera
```

Model nhận diện là **SSD MobileNet v1 COCO (quantized)** của TensorFlow, đã có sẵn trong git ở
`models/detect.tflite` và `models/labelmap.txt` (lớp `person` là lớp số 0). Nguồn gốc, nếu cần tải lại:

```bash
mkdir -p models && cd models
curl -LO https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip && cd ..
```

`camera.model_path` trong `config/settings.yaml` đã trỏ sẵn tới `models/detect.tflite`. Trên Pi dùng `tflite-runtime`; trên
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

Hai bản dùng chung cổng `/dev/serial0` nên mỗi lúc chỉ chạy một bản, đúng với bộ điều khiển đang nối dây.
`run.sh` chọn bản, bật `.venv` nếu có và kiểm tra cổng trước khi chạy; tham số phía sau được chuyển nguyên
cho chương trình:

```bash
./run.sh pixhawk --viewer      # rescue_main.py
./run.sh stm32 --no-lidar      # main.py
./run.sh --viewer              # bản ghi trong .boat_profile (do boat.bat deploy trên máy tính ghi)
```

Cấu hình chung nằm ở `config/settings.yaml`; phần riêng của từng phao ở `config/boat_pixhawk.yaml` và
`config/boat_stm32.yaml`, `run.sh` tự nạp đúng file. Chạy thẳng `python3 rescue_main.py` thì phải tự thêm
`--overlay config/boat_pixhawk.yaml`.

### Bản Pixhawk trên phao thật (`rescue_main.py`)

- **LiDAR**: `lidar.source: serial` (đặt sẵn trong `boat_pixhawk.yaml`) đọc module `serial.lidar_model` ở
  `serial.lidar_port`, gửi từng tia cho bộ né vật cản của autopilot. Không có vòng quét mới trong
  `avoidance.lidar_stale_s` (1 giây) thì Pi **ngừng gửi** `OBSTACLE_DISTANCE`: autopilot báo cảm biến
  proximity lỗi và từ chối arm, thay vì coi im lặng là mặt nước trống. Không lắp LiDAR thì đặt
  `lidar.source: none`, khi đó chỉ có vùng cấm quanh người được gửi.
- **Bo phụ STM32F103** (`helper.port: /dev/ttyUSB0`, qua mạch USB-to-TTL; xem `STM32F103/README.md`):
  Pi gửi góc servo camera (`$CAM`), và mỗi giây gửi nhịp sống, số người đã tìm thấy và vị trí, tư thế
  của phao (`$HBT`, `$VIC`, `$POS`). Bo bật relay báo động khi số người tăng và phát LoRa về trạm bờ.
  Bo báo lại áp pin và nút dừng khẩn cấp: nhấn nút thì Pi yêu cầu autopilot vào `rescue.estop_mode`
  (HOLD). Rút bo ra chương trình vẫn chạy, cổng được mở lại mỗi 3 giây.
- **Servo camera** nằm trên bo phụ. Camera quay ngược phía người thì đổi `CAM_PAN_DIR` trong
  `STM32F103/App/app_config.h`. (Không có bo phụ, `helper.port: none`: servo có thể cắm vào một cổng ra
  của Pixhawk, đặt `camera.pan_output` bằng số `SERVOn`.)
- **Số người lên dashboard qua MAVLink**: mỗi giây Pi cũng phát `NAMED_VALUE_INT "VICTIMS"` cho autopilot;
  cầu nối `esp32-lora-station/tools/mavlink_bridge.py` đọc nó khi phao nối trạm bờ bằng MAVLink.

Thêm `--log` để ghi mỗi giây một dòng CSV vào `logs/run_<ngày>_<giờ>.csv` (chế độ, vị trí, hướng, pha cứu hộ,
người đang thấy, vật cản gần nhất, tuổi vòng quét LiDAR, pin và trạng thái bo phụ): `./run.sh pixhawk --viewer --log`.

Trình tự đầy đủ khi đưa lên phao nằm ở `docs/porting-checklist.md`. Tóm tắt, **tháo chân vịt**:

1. `python3 tools/lidar_capture.py`: phải ra `valid packets` khác 0 (xem mục LiDAR bên dưới).
2. `./run.sh pixhawk --viewer`, mở `http://<địa chỉ Pi>:8090/` (trang có cả bản đồ quyết định, xem `sim/README.md`): thấy ảnh camera, điểm LiDAR đúng phía
   (đặt một vật trước mũi phao, chấm trắng phải nằm phía trên hình), khung quanh người.
3. Log mỗi 5 giây có `board 12.3 V lora ok` (bo phụ đang trả lời; `board off` là chưa thấy bo). Đứng
   trước camera rồi đi sang một bên: servo quay theo. Sai chiều thì đổi `CAM_PAN_DIR` trên bo phụ.
4. Rút dây LiDAR: log báo `obstacle feed stopped`, Mission Planner báo lỗi proximity. Cắm lại thì hết.
5. Arm, AUTO với một lộ trình ngắn: Messages của Mission Planner hiện `VICTIM SEEN` khi có người trước camera.

### Bản STM32 (`main.py`)

Bản trình diễn trên cạn. Pi gửi hướng và tốc độ (`$NAV`), nhịp sống (`$HBT`), góc camera (`$CAM`) và
sự kiện phát hiện người (`$TXD,VICTIM_FOUND`); STM32 đưa số người đã báo vào khung LoRa để trạm bờ và
dashboard hiện cảnh báo. Khi tay điều khiển đã arm và gạt sang AUTO mà chưa thấy ai, camera quét tìm theo
`camera.scan` (STM32 báo `armed`, `auto` trong gói `$SYS`).

LiDAR chọn bằng `serial.lidar_model` (`lds008`, `ld14`, `none`) hoặc `--no-lidar`. Với
`vehicle.lidar_required: false`, mất cổng LiDAR chỉ ghi cảnh báo và chạy tiếp, không né vật cản.
Module LDS-008 không có datasheet: `modules/lidar_lds008.py` viết theo model cùng họ LDS-006. Trước khi
tin bộ giải mã, ghi dữ liệu thô từ module thật:

```bash
python3 tools/lidar_capture.py        # /dev/ttyAMA3 @ 115200, 5 giây, ghi lidar_capture.bin
```

Nếu `valid packets` ra 0 điểm thì giao thức khác giả định, cần viết lại bộ giải mã theo file ghi được.
Sau đó chỉnh `vehicle.lidar_yaw_offset_deg` và `vehicle.lidar_anticlockwise` cho đúng hướng mũi phao.

Cả hai chương trình đọc LiDAR qua `modules/lidar_serial.py`: `main.py` gom điểm thành cụm cho trường thế
của Pi, `rescue_main.py` giữ từng tia cho autopilot.

## Kết nối

- STM32: `/dev/serial0`, 115200 8N1, GPIO14/15.
- LiDAR: `/dev/ttyAMA3`, 115200 8N1. LD14 chỉ cần TX -> Pi GPIO5; LDS-008 cần thêm Pi GPIO4 (TX) -> RX của module để gửi lệnh `startlds$`. Bật `enable_uart=1` và `dtoverlay=uart3` trong boot configuration; tắt serial login console trên UART STM32.
- Hiệu chuẩn `vehicle.lidar_yaw_offset_deg` để góc 0 của LD14 đúng hướng mũi phao.

## Hành vi phát hiện người

AI chỉ tạo target khi lớp COCO `person` đạt confidence cấu hình **và** một cụm LiDAR ở cùng góc camera. Khi khóa được, Pi gửi `TXD,VICTIM_FOUND,lat,lon,confidence` (không ảnh) và điều hướng đến khoảng cách `target_standoff_m`; không ghép được LiDAR thì không tiến lại gần.

## Kiểm tra nhanh

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile main.py modules/*.py utils/*.py
```

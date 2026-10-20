# RedgeSCUE Raspberry Pi runtime

## Cài đặt trên Raspberry Pi 4

```bash
cd Rasp_Pi
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python3 main.py --no-camera
```

Model nhận diện là **model edge của đội** (`../YOLO_Movenet/weights`, chọn bằng `camera.model: yolo_movenet`),
nối vào `modules/edge_model.py`:

- **YOLOv8n** (`yolov8n_640_int8.tflite`: int8 bên trong, vào/ra float), đầu vào 640 × 640, hai lớp `person` / `boat`.
  Trên Pi 4 khoảng 0.8 s một khung (bản 320 px trước đó: 0.24 s) nhưng tìm được người nhỏ và ở xa tốt hơn. Điểm số
  luôn ra 0.50, nên `camera.confidence_threshold` chỉ tách có / không có người.
  Người có khung nằm phần lớn trong khung thuyền (`camera.on_boat_overlap`) là người **trên thuyền**: mức `SAFE`,
  vẫn được báo về bờ một lần nhưng không bao giờ bị bám theo.
- **MoveNet Lightning** đọc tư thế từng người còn lại (tối đa `camera.max_poses`). Hông, gối, cổ chân có điểm trung
  bình dưới `camera.water_lower_body_score` thì người đó **dưới nước** (`IN WATER` trên viewer, khung xương đỏ).
- Hướng, khoảng cách, xác nhận và quét camera giữ nguyên như trước: chỉ phần "thấy người ở đâu trong khung hình" đổi.

Model cũ **SSD MobileNet v1 COCO (quantized)** vẫn dùng được với `camera.model: ssd` và
`camera.model_path: models/detect.tflite` (lớp `person` là lớp số 0). Nguồn gốc, nếu cần tải lại:

```bash
mkdir -p models && cd models
curl -LO https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip && cd ..
```

Cả hai model chạy bằng TFLite. Trên Pi dùng `tflite-runtime`; trên
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

Pi + STM32F407, **không có tay điều khiển**: phao chỉ chạy lộ trình admin gửi từ dashboard qua LoRa
([docs/lora-route-downlink.md](../docs/lora-route-downlink.md)). Pi gửi hướng và tốc độ (`$NAV`), nhịp sống
(`$HBT`), góc camera (`$CAM`) và báo người (`$TXD,VICTIM_FOUND`); STM32 giữ hướng bằng PID và đưa báo cáo vào
khung LoRa cho trạm bờ và dashboard. Đèn và relay chỉ do dashboard bật, Pi không đụng tới (`$SOS` không dùng).

**Trạng thái** (`modules/mission.py`, hiện trên viewer và trong log):

| Trạng thái | Phao làm gì | Chuyển sang |
|---|---|---|
| `IDLE` | đứng yên (chưa có lộ trình, nhận lộ trình rỗng, hoặc đã đi hết), camera nhìn thẳng | `PATROL` khi có lộ trình |
| `PATROL` | đi lần lượt các waypoint, né vật cản (BAPF), camera quét ±45° | `FOLLOW` khi xác nhận được người; `RETURN` khi mất liên lạc bờ |
| `FOLLOW` | tới người có mức rủi ro cao nhất (cùng mức thì người gần nhất), dừng cách `target_standoff_m` | `PATROL` khi người đó nhấn nút xác nhận (đã lấy phao/đồ, Pi bỏ qua người này `follow.ignore_s` giây) hoặc mất dấu `follow.lost_s` giây; `RETURN` khi mất liên lạc bờ |
| `RETURN` | về nhà, không bám người nhưng vẫn phát hiện và báo người | `HOME_HOLD` khi cách nhà dưới `return.arrive_radius_m` |
| `HOME_HOLD` | đứng yên ở nhà | — |

Khi liên lạc bờ trở lại, `RETURN` / `HOME_HOLD` quay về `PATROL` **đúng waypoint đang đi dở** (hoặc `IDLE` nếu
không còn lộ trình). Bộ an toàn (`modules/safety.py`: IMU, lật, GPS, geofence, LiDAR) luôn đứng trên mọi trạng thái.

- **Mất liên lạc bờ**: F407 báo trong `$LNK` (không nghe trạm 60 s thì `shore_ok` = 0). **Nhà** là vị trí GPS của
  trạm bờ (cũng trong `$LNK`); trạm nằm ngoài geofence thì nhà được kéo vào trong hàng rào `return.fence_margin_m`
  mét. Chưa từng nhận vị trí trạm thì nhà là chỗ phao đứng khi nhận lộ trình hiện tại.
- **Mức rủi ro** (`modules/edge_model.py`, ngưỡng ở `camera.risk`): người trên thuyền `SAFE` (xanh lá, không bao giờ
  bám theo); thấy phần thân dưới `MEDIUM` (vàng); dưới nước `HIGH` (cam); dưới nước có dấu hiệu đuối nước
  `CRITICAL` (đỏ): hai cổ tay cao hơn vai, hoặc chỉ còn thấy đầu (không thấy vai). Người ngoài `camera.max_poses`
  người gần nhất không được đọc tư thế nên tính `MEDIUM`.
- **Báo về bờ** `$TXD,seq,VICTIM_FOUND,lat,lon,conf,risk` (`risk` 0..3; `lat,lon` = 0,0 khi chưa có GPS fix):
  mỗi người khi được xác nhận lần đầu (cả người `SAFE`; cùng vị trí ước tính trong 5 m thì không báo lại); khi
  đang bám mà mức rủi ro của người đó đổi (thấy 2 lần liên tiếp); và mỗi `follow.report_every_s` (20 s) nếu vị
  trí ước tính của người đó đã dịch hơn 3 m.
- **Loa**: "phao cứu hộ" (`sounds/rescue_boat.wav`) mỗi `audio.periodic_s` (100 s), lần đầu ngay khi chạy; nhấn nút
  xác nhận thì phát `supplies_confirmed` (không chen tiếng định kỳ vào lúc đó).

Muốn chạy / dừng phao: xem `STM32F4/README.md` mục 13. Trên giá (`config/bench_demo.yaml`,
`follow.only_on_route: false`) phao bám người cả khi chưa có lộ trình.

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

Phao STM32 (`config/boat_stm32.yaml`), gọi cổng theo **ổ USB** (`/dev/serial/by-path/...`), không theo
`/dev/ttyUSBn` vì số này đổi giữa các lần khởi động; `tools/port_probe.py` cho biết tên sau khi cắm lại:

- STM32F407: UART4 qua mạch USB-to-TTL ở ổ `usb-0:1.2`, 115200 8N1.
- LiDAR LD14: mạch USB-to-TTL ở ổ `usb-0:1.1`, 115200 8N1 (LD14P: 230400, đặt `serial.lidar_baud`).
- Camera: ổ `usb-0:1.3` (trên Pi 4 8 GB chỉ ổ này chạy ổn định webcam), gọi theo số serial `/dev/v4l/by-id/...`.
- Hiệu chuẩn `vehicle.lidar_yaw_offset_deg` để góc 0 của LD14 đúng hướng mũi phao.

`settings.yaml` vẫn để mặc định cũ (`/dev/serial0` GPIO14/15 cho STM32, `/dev/ttyAMA3` cho LiDAR với
`dtoverlay=uart3`) cho cách nối dây qua UART của Pi.

Giao thức với F407 ở `STM32F4/README.md` mục 11. Pi đọc thêm:

- `$LNK,seq,shore_ok,heard_ago_s,home_valid,home_lat,home_lon` mỗi giây: `shore_ok` 0/1, `heard_ago_s` số giây từ
  lần cuối nghe trạm bờ (-1: chưa nghe), `home_valid` 0/1 và vị trí trạm (6 chữ số thập phân, có thể để trống khi
  `home_valid` = 0). Firmware chưa có `$LNK`: Pi coi liên lạc bờ luôn tốt và không có nhà.
- `$SYS` trường thứ 9 `battery_pct` (0..100, -1 không rõ); trường 1 `battery` là volt.

## Hành vi phát hiện người

Mỗi người có hướng (camera + góc servo), khoảng cách (chiều cao khung, `camera.distance_source`) và mức rủi ro.
Pi bám theo người có rủi ro cao nhất, không bao giờ bám người trên thuyền, và báo về bờ
`TXD,VICTIM_FOUND,lat,lon,confidence,risk` (không gửi ảnh) như mô tả ở mục Bản STM32.

## Kiểm tra nhanh

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile main.py modules/*.py utils/*.py
```

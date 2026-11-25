# Phao 2 — prototype camera / LoRa / GPS, không điều khiển thuyền

Thư mục này là phần bổ sung độc lập trong repository VEDC. Không sửa firmware phao 1.

## Phạm vi demo

- Pi 4 nhận camera CSI qua Picamera2 (mặc định), hoặc camera USB qua OpenCV.
- Nhận diện người; cần 3 khung hình liên tiếp có người trước khi báo.
- Pi nối USB–UART với USART1 STM32F103C8T6. STM32 điều khiển Ra-02 bằng SPI1, nhận GPS DroneCAN qua CAN (PA11/PA12).
- Báo người bằng đúng khung `ev=PHAO-02,...,V,...` của hệ thống hiện tại.
- Trạm bờ ESP32 phân xử: phao báo trước giữ điểm cứu; ACK 11 trường báo tọa độ cần bỏ qua cho phao khác. Pi 2 nhận ACK và dừng báo lại điểm đó trong 120 s (được gia hạn khi trạm nhắc lại).
- `REPORT_ACKED` chỉ có nghĩa báo cáo đã được xác nhận; giao thức ACK không chứa ID người sở hữu. Xem chủ điểm cứu trên dashboard.
- Chuyển tiếp nguyên gói telemetry/event của phao khác và ACK về phao khác. Có bộ nhớ chống lặp và hàng đợi có giới hạn.
- Không xuất tín hiệu động cơ, PWM servo, relay, lái tự động hoặc lệnh điều khiển phao 1.

**Đây là relay một trung gian tương thích giao thức cũ, chưa phải mesh định tuyến nhiều chặng.**
Không có khám phá đường đi hoặc hop count. Chỉ bật một relay trong thí nghiệm này. Firmware phao 1 có cửa sổ nhận ACK khoảng 1,5 s; truyền đi + trả về qua relay có thể vượt cửa sổ đó, đặc biệt với khung dài. Cần đo trên bàn thử trước khi kết luận kết nối hai chiều qua relay ổn định. Chưa chuyển tiếp lộ trình RTE hay gói chẩn đoán. Một demo phân xử thành công khi cả hai phao liên lạc trực tiếp với trạm **không chứng minh** đường relay hoạt động.

## Cấu trúc

| Thư mục | Nội dung |
|---|---|
| `stm32f1/` | Firmware cầu nối, driver RA-02 TX/RX, CMake build |
| `pi4/` | Camera, nhận diện, GPS NMEA, báo cáo và relay, log JSONL |
| `tests/` | Kiểm thử giao thức và tích hợp với mã C thật của trạm bờ |

Firmware tham chiếu HAL/CMSIS, startup và linker của `../STM32F103`; cần giữ checkout repository đầy đủ khi build. Driver LoRa lấy từ `../STM32F4/Modules/LoRa` và đổi header HAL sang F1. Không dùng driver F103 cũ vì bản đó chỉ phát.

## Đấu dây

| STM32F103 | Kết nối |
|---|---|
| PA4 | Ra-02 NSS |
| PA5 | Ra-02 SCK |
| PA6 | Ra-02 MISO |
| PA7 | Ra-02 MOSI |
| PB0 | Ra-02 RESET |
| PB1 | Có thể nối DIO0 theo sơ đồ cũ; firmware đọc IRQ qua SPI, không cần ngắt DIO0 |
| PA9 TX | RX của USB–UART |
| PA10 RX | TX của USB–UART |
| PA11 CAN_RX | RXD (CRX) của module thu phát CAN |
| PA12 CAN_TX | TXD (CTX) của module thu phát CAN |
| PA13 / PA14 | SWDIO / SWCLK để nạp ST-Link |
| GND | GND chung STM32, Ra-02, USB–UART, module CAN, GPS |

USB–UART cắm vào USB Pi 4. **Không đấu PA9/PA10 tới GPIO8/10 của Pi trong cấu hình này.** Chọn adapter UART TTL **3,3 V**, không phải RS232. Chân VCC adapter không cần nối nếu STM32 đã có nguồn riêng. Ra-02 dùng nguồn 3,3 V đủ dòng, tụ 100 nF + 10–47 µF gần module; anten phù hợp 433 MHz. BOOT0=0. Giữ PA13/PA14 cho SWD.

## GPS CUAV NEO 3X qua CAN

Tài liệu hãng: https://doc.cuav.net/gps/neo-series-gnss/en/neo-3x.html#connect

NEO 3X dùng DroneCAN, nguồn 4,7–5,2 V. CANH/CANL của GPS nối vào **module thu phát CAN** (TJA1050, SN65HVD230...), TXD/RXD của module vào PA12/PA11. Nguồn module theo loại chip. Giữa CANH và CANL (tắt nguồn) nên đo được khoảng 60 Ω.

- `dronecan.c` (không phụ thuộc HAL, có host test): cấp node ID 50 cho GPS nếu nó xin (F103 là node 100), phát NodeStatus mỗi giây, giải mã `Fix2` (1063). Chỉ một thiết bị DroneCAN được cấp ID.
- CAN 1 Mbit/s, lấy mẫu 81 %. Firmware dùng thạch anh 8 MHz; nếu không có thì về HSI (sai số 1 %, CAN có thể lỗi).
- Gửi lên Pi tối đa 1 lần/s dưới dạng câu GGA trong dòng `GPSHEX,<hex>`; Pi chỉ nhận RMC/GGA đúng checksum, fix quá 3 s thành `NO_FIX`. GGA không có giờ và độ cao. Pi vẫn ghi vào `logs/gps_uart.bin`.
- Dòng `STATUS` (mỗi 5 s): `can_rx` tổng số khung CAN, `can_lost` khung mất, `can_esr` thanh ghi lỗi (REC, TEC), `gps_node`, `gps_fix` (0 không, 2 = 2D, 3 = 3D), `gps_sats`, `gps_msgs`, `gps_bad` (sai CRC). `can_rx=0` là không có tín hiệu CAN: kiểm tra dây, nguồn GPS, điện trở đầu cuối.

## Vị trí người trong demo

Camera một mắt không tự cung cấp tọa độ GPS của người. Bản prototype này dùng **một điểm mục tiêu cố định do người vận hành cấu hình**, chỉ phát sự kiện khi camera thực sự phát hiện người. Đây là giả định demo được ghi trong log `position_source=configured_demo_target`, không phải định vị người bằng AI.

Đặt `demo_target` thành `[latitude, longitude]` của người/điểm cứu. Nếu để `null`, camera vẫn phát hiện nhưng trạng thái là `DETECTED_NO_TARGET_POSITION` và không phát báo cáo người có tọa độ giả. GPS của phao chỉ dùng cho beacon, không bị dùng thay vị trí người.

Mặc định `detector: yolo_movenet` dùng **đúng model và cấu hình của phao 1**: YOLOv8n 640 int8 + MoveNet qua `Rasp_Pi/modules/edge_model.py`, ngưỡng và đường dẫn model lấy từ `Rasp_Pi/config/settings.yaml` + `boat_stm32.yaml`. Đổi model phao 1 thì phao 2 đổi theo khi `camera.rasp_pi` trỏ tới bản `Rasp_Pi` đã cập nhật. Người ở trên thuyền không được báo; risk 1–3 do YOLO/MoveNet suy ra như phao 1, confidence là điểm YOLO. `hog` vẫn dùng được để thử nhanh: chỉ có/không, gửi confidence bằng `camera.confidence` và risk 2 ước lệ. Không dùng kết quả này làm đánh giá cứu nạn thực tế.

## Build và nạp STM32

Cần CMake, Ninja, ARM GNU Toolchain trong PATH. Chạy tại `Phao_2/stm32f1`:

```powershell
cmake -S . -B build -G Ninja "-DCMAKE_TOOLCHAIN_FILE=toolchain.cmake"
cmake --build build
```

Kết quả dự kiến `build/Phao2.elf`, `build/Phao2.hex`, `build/Phao2.bin`. Chip giới hạn Flash 64 KB/RAM 20 KB theo linker gốc. Firmware chạy 64 MHz từ thạch anh 8 MHz, không có thì từ HSI. Nạp HEX bằng STM32CubeProgrammer/ST-Link; không dùng file của phao 1. Chưa cung cấp binary đã kiểm chứng vì ARM GCC hiện không chạy được trên máy tạo mã.

## Cài và chạy trên Pi 4

Dùng Raspberry Pi OS 64-bit có Picamera2; chạy từ `Github/Phao_2/pi4`:

```bash
sudo apt update
sudo apt install python3-venv python3-opencv python3-picamera2
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
cp config.example.json config.json
ls -l /dev/serial/by-id/
```

Sửa `config.json`:

1. `serial_port`: đường dẫn đúng của USB–UART trong `/dev/serial/by-id/`.
2. `demo_target`: tọa độ mục tiêu thực nghiệm. Không giữ một tọa độ ví dụ làm tọa độ thật.
3. `camera.backend`: `picamera2` với CSI, hoặc `opencv` với USB/V4L2; `device` là chỉ số hoặc `/dev/video0`.
4. `camera.detector`: `yolo_movenet` (giống phao 1) hoặc `hog`. `camera.rasp_pi`: thư mục `Rasp_Pi` (trên Pi: `/home/khanh/vedc/Rasp_Pi`), cạnh nó phải có `YOLO_Movenet/weights`; chạy bằng Python của `.venv` trong thư mục đó (có tflite-runtime). Pi 4 đo được khoảng 2,7 fps với model 640. Inference chạy thread riêng để không giữ vòng xử lý radio.

```bash
python main.py --config config.json
# Chỉ thử GPS/relay, không tạo phát hiện giả:
python main.py --config config.json --no-camera
```

Nếu không có quyền cổng serial: thêm user vào group `dialout`, đăng xuất/đăng nhập lại. Không mở cùng cổng bằng terminal serial và chương trình cùng lúc.

Radio khớp phao 1: **433 MHz, SF9, BW125 kHz, CR4/5, sync 0xF3, CRC bật, 17 dBm**. Chương trình thử lại báo cáo tối đa 8 lần, có jitter; mất ACK dẫn tới `UNCONFIRMED`, không tự nhận là chủ. Đây không phải TDMA đồng bộ; hai phao vẫn có thể va chạm sóng.

Log `events.jsonl`: `DETECTION`, `REPORT_TX`, `RADIO_RX`, `REPORT_ACKED`, `SUPPRESSED`, `RELAY_TX`, `REPORT_UNCONFIRMED`, trạng thái GPS/serial. Sequence sự kiện được lưu `state.json` để không lặp ngay sau restart. Một phiên demo tương ứng một điểm cứu; dùng thư mục log mới khi đổi mục tiêu và xóa/release claim cũ tại dashboard nếu cần.

## Kịch bản kiểm chứng với phao 1

**A — Phân xử hai phao phát hiện cùng người:**

1. Trạm bờ phải chạy firmware có `victim_claims.c` và ACK 11 trường. Phao 1 cần firmware F4/Pi có xử lý `$SKP` như `docs/victim-claim.md`.
2. Đặt cả hai phao và trạm trong tầm thu tốt; ban đầu đặt `relay=false` trên phao 2 để tách bài kiểm tra phân xử khỏi đường relay.
3. Cho phao 1 phát hiện trước. Lấy tọa độ điểm cứu được báo trên dashboard làm `demo_target` của phao 2 (hoặc dùng điểm đã đo sao cho hai ước lượng cách nhau dưới 6 m).
4. Cho cùng người vào hình camera Pi 4. Chờ `DETECTION` → `REPORT_TX` → ACK → `SUPPRESSED`. Dashboard phải giữ chủ PHAO-01 và ghi nhận cả hai phao thấy cùng điểm.
5. Giữ người trong hình: phao 2 không gửi lại V trong thời hạn claim. Không coi việc ngừng gửi do lỗi camera/mất serial là thành công; kiểm tra có ACK/`SUPPRESSED` thực sự.
6. Release claim trên dashboard, chờ/xóa phiên demo phù hợp, làm ngược thứ tự. Phao 2 báo trước; sau đó phao 1 phải nhận `$SKP` và bỏ FOLLOW. Phao 2 vẫn đứng yên, không thực hiện cứu hộ/di chuyển.

**B — Chuyển tiếp một trung gian:**

1. Bật `relay=true`. Đặt phao 2 nghe được cả phao 1 và trạm, phao 1 không nghe trực tiếp trạm (bố trí khoảng cách/che chắn thích hợp).
2. Chứng minh bằng log phao 2 có `RADIO_RX` từ PHAO-01 → `RELAY_TX`; trạm nhận đúng ID/sequence gốc. Không đổi ID gói thành PHAO-02.
3. Kiểm tra chiều ACK: phao 2 nhận `ACK,PHAO-01,...`, chuyển tiếp, phao 1 thực sự tăng bộ đếm ACK. Nếu chỉ trạm nhận uplink nhưng phao 1 không nhận ACK thì mới chứng minh chuyển tiếp một chiều.
4. Tắt tiến trình relay phao 2 và so sánh: trạm phải mất báo cáo mới của phao 1; bật lại phải có báo cáo mới. Phân biệt dữ liệu cũ trên dashboard với gói mới.
5. Nếu ACK về muộn, cần thay cửa sổ RX/lịch radio của phao 1 trong một thay đổi riêng có kiểm thử. Bản này không tự sửa timing của thuyền đang vận hành.

## Kiểm thử và trạng thái bàn giao

Từ thư mục `Phao_2`:

```bash
python -m unittest discover -s tests -v
```

28 test đã chạy đạt trên máy phát triển, trong đó 2 test biên dịch và chạy **parser, bộ phân xử và bộ tạo ACK thật của trạm bờ** bằng host GCC. Nếu không có host GCC, hai test tích hợp được đánh dấu skip.

ARM firmware build/link đạt, không cảnh báo, với Arm GNU Toolchain 14.2 rel1 (FLASH 14,5 kB/64 kB, RAM 4,6 kB/20 kB). Đã chạy trên phần cứng (24–25/11/2026): NEO 3X nhận node ID, gửi Fix2 5 Hz và có fix thật; RA-02 thu PHAO-01, phát được trạm nhận và ACK; Pi 4 + webcam + YOLO 640 chạy 2,3 fps, báo cáo được ACK trong khoảng 0,7–2,5 s. Chưa đo: tầm nhận diện 10 m, relay một chặng, hướng (F103 chưa đọc la bàn, hướng phao lấy từ `assumed_heading_deg`).

**SPI của RA-02:** 31 kHz với sườn xung chậm (`GPIO_SPEED_FREQ_LOW`) trên NSS, SCK, MOSI. Với dây cắm hiện tại, sườn nhanh (nhất là NSS) làm mất lệnh ghi: radio bị khởi tạo lại mỗi giây dù đọc mã chip vẫn đúng.

## Demo trong phòng (GPS giả định)

Không có GPS thật thì người được đặt theo `assumed_position` + `assumed_heading_deg` và camera, trong `max_range_m`; người bị khung hình cắt (không có khoảng cách) thì dùng `demo_target`. Có GPS thật thì dùng GPS thật, trừ khi công tắc bật: nút trên dashboard (`demo_url`, Pi hỏi mỗi 5 s, tự tắt sau 3 giờ) hoặc file `demo_flag` trên Pi. Gói LoRa vẫn báo `NO_FIX` khi không có fix. Khung chẩn đoán `dg=` mỗi 10 s mang fps của AI.

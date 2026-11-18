# Phao 2 — prototype camera / LoRa / GPS, không điều khiển thuyền

Thư mục này là phần bổ sung độc lập trong repository VEDC. Không sửa firmware phao 1.

## Phạm vi demo

- Pi 4 nhận camera CSI qua Picamera2 (mặc định), hoặc camera USB qua OpenCV.
- Nhận diện người; cần 3 khung hình liên tiếp có người trước khi báo.
- Pi nối USB–UART với USART1 STM32F103C8T6. STM32 điều khiển Ra-02 bằng SPI1, nhận UART GPS qua USART2.
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
| PA3 RX | TX UART của bộ chuyển CAN–UART |
| PA2 TX | RX UART của bộ chuyển CAN–UART, để dự phòng; firmware hiện chỉ đọc GPS |
| PA13 / PA14 | SWDIO / SWCLK để nạp ST-Link |
| GND | GND chung STM32, Ra-02, USB–UART, CAN–UART |

USB–UART cắm vào USB Pi 4. **Không đấu PA9/PA10 tới GPIO8/10 của Pi trong cấu hình này.** Chọn adapter UART TTL **3,3 V**, không phải RS232. Chân VCC adapter không cần nối nếu STM32 đã có nguồn riêng. Ra-02 dùng nguồn 3,3 V đủ dòng, tụ 100 nF + 10–47 µF gần module; anten phù hợp 433 MHz. BOOT0=0. Giữ PA13/PA14 cho SWD.

## GPS CUAV NEO 3X và bộ CAN–UART

Tài liệu hãng: https://doc.cuav.net/gps/neo-series-gnss/en/neo-3x.html#connect

NEO 3X dùng DroneCAN và nguồn vào 4,7–5,2 V. Không nối CAN-H/CAN-L trực tiếp vào UART. Nguồn GPS và mức logic UART của bộ chuyển là hai thông số khác nhau.

**Thông tin còn thiếu: model bộ CAN–UART, baud UART và một mẫu dữ liệu đầu ra.** Bộ chuyển CAN thô sang serial không tự biến DroneCAN thành NMEA. Bản này không giả định bộ chuyển có bộ giải mã DroneCAN.

- STM32 nhận byte UART và gửi lên Pi dưới dạng `GPSHEX,<hex>`; Pi lưu nguyên byte vào `logs/gps_uart.bin` để kiểm tra. Có thống kê `gps_lost` trong log nếu dữ liệu tràn bộ đệm.
- Pi chỉ chấp nhận RMC/GGA NMEA có checksum đúng và fix hợp lệ; fix quá 3 s thành `NO_FIX`.
- Nếu bộ chuyển xuất SLCAN, khung nhị phân riêng hoặc DroneCAN thô, GPS vẫn được ghi raw nhưng **chưa có tọa độ**. Cần bổ sung decoder đúng loại bộ chuyển sau khi biết model/mẫu dữ liệu.
- Mặc định UART GPS 115200 baud; thay tham số CMake `GPS_BAUD` theo thực tế.
- GPS luồng liên tục tốc độ cao có thể vượt băng thông do hex làm tăng gấp đôi dung lượng; giảm tốc độ bản tin của bộ chuyển và kiểm tra `gps_lost=0`.

## Vị trí người trong demo

Camera một mắt không tự cung cấp tọa độ GPS của người. Bản prototype này dùng **một điểm mục tiêu cố định do người vận hành cấu hình**, chỉ phát sự kiện khi camera thực sự phát hiện người. Đây là giả định demo được ghi trong log `position_source=configured_demo_target`, không phải định vị người bằng AI.

Đặt `demo_target` thành `[latitude, longitude]` của người/điểm cứu. Nếu để `null`, camera vẫn phát hiện nhưng trạng thái là `DETECTED_NO_TARGET_POSITION` và không phát báo cáo người có tọa độ giả. GPS của phao chỉ dùng cho beacon, không bị dùng thay vị trí người.

HOG là lựa chọn nhẹ sẵn có để thử người đứng nhìn thấy toàn thân; không phải mô hình phát hiện người chìm/nạn nhân. Tùy chọn YOLO dùng model COCO có class 0 là person. Không có phân loại đuối nước. Giao thức cũ bắt buộc trường risk: giá trị `0` trong demo là giá trị cố định (dashboard hiểu là safe), **không phải kết luận an toàn từ AI**; HOG gửi confidence=0 vì trọng số HOG không phải xác suất. Không dùng kết quả này làm đánh giá cứu nạn thực tế.

## Build và nạp STM32

Cần CMake, Ninja, ARM GNU Toolchain trong PATH. Chạy tại `Phao_2/stm32f1`:

```powershell
cmake -S . -B build -G Ninja "-DCMAKE_TOOLCHAIN_FILE=toolchain.cmake" "-DGPS_BAUD=115200"
cmake --build build
```

Kết quả dự kiến `build/Phao2.elf`, `build/Phao2.hex`, `build/Phao2.bin`. Chip giới hạn Flash 64 KB/RAM 20 KB theo linker gốc. Firmware dùng HSI + PLL 64 MHz, không phụ thuộc thạch anh ngoài. Nạp HEX bằng STM32CubeProgrammer/ST-Link; không dùng file của phao 1. Chưa cung cấp binary đã kiểm chứng vì ARM GCC hiện không chạy được trên máy tạo mã.

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
4. `camera.detector`: `hog` để thử nhanh; `yolo` nếu đã cài `ultralytics` và có file model COCO phù hợp tại `camera.model`. Ứng dụng không tự tải model. Pi 4 chạy 320 px; tốc độ thực tế cần đo. Inference chạy thread riêng để không giữ vòng xử lý radio.

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

12 test đã chạy đạt trên máy phát triển, trong đó 2 test biên dịch và chạy **parser, bộ phân xử và bộ tạo ACK thật của trạm bờ** bằng host GCC. Nếu không có host GCC, hai test tích hợp được đánh dấu skip.

ARM firmware chưa build/link thành công: `arm-none-eabi-gcc.exe` 14.2 trên máy hiện báo `Illegal System DLL Relocation`, kể cả khi chỉ gọi `--version`. Chưa nạp STM32, chưa chạy camera Pi 4, chưa đo RF/độ trễ ACK. GPS DroneCAN qua bộ chuyển chưa thể chốt decoder khi chưa biết model và dữ liệu UART. Đây là các phần cần xác minh phần cứng, không được suy ra từ host test.

# ESP32-S3: Ngắt GPIO → Đọc GPS ATK-S1216 → Gửi qua LoRa SX1278

Khi có tín hiệu trên 1 chân GPIO (dùng ngắt phần cứng, không polling), chương
trình đọc tọa độ GPS mới nhất từ module ATK-S1216 (qua UART1) và gửi ngay qua
LoRa SX1278 (module RA-02).

## Cây thư mục

```
gpio_trigger_gps_lora_idf/
├── CMakeLists.txt
├── components/sx127x/       (driver LoRa dùng chung, đã dùng ở các project trước)
└── main/
    ├── CMakeLists.txt
    ├── main.c
    ├── nmea_gps.c
    └── nmea_gps.h            (parser NMEA, chỉ đọc câu $GPGGA/$GNGGA)
```

## Sơ đồ nối dây (ESP32-S3)

```
LoRa SX1278 (RA-02)          GPS ATK-S1216 (UART1)
SCK  --> GPIO12               TX(GPS) --> GPIO18 (RX1 ESP32)
MISO --> GPIO13               RX(GPS) <-- GPIO17 (TX1 ESP32, baud 38400)
MOSI --> GPIO11
CS   --> GPIO10                Chân TRIGGER
RST  --> GPIO9                 --> GPIO4 (đổi tùy ý, xem bên dưới)
DIO0 --> GPIO8
```

## Cách hoạt động

1. **Task đọc GPS chạy nền liên tục** — luôn cập nhật tọa độ mới nhất, không
   cần chờ tới lúc có trigger mới bắt đầu đọc GPS (giúp gửi đi ngay lập tức,
   không mất thời gian chờ GPS phản hồi sau khi có tín hiệu).
2. **Chân trigger dùng ngắt phần cứng** (`gpio_isr_handler_add`) thay vì
   polling liên tục — phản ứng ngay khi có tín hiệu, tiết kiệm CPU.
3. Trong ISR chỉ đẩy 1 giá trị vào FreeRTOS Queue (đúng khuyến nghị ESP-IDF -
   không làm việc nặng/blocking trong ngắt); 1 task riêng (`task_xu_ly_trigger`)
   nhận từ Queue, khử rung phím (debounce 300ms), rồi gửi tọa độ qua LoRa.

## Tùy chỉnh chân trigger / kiểu tín hiệu

Trong `main/main.c`, sửa các dòng sau cho phù hợp phần cứng của bạn:

```c
#define TRIGGER_GPIO       GPIO_NUM_4   // đổi thành chân bạn muốn dùng
#define TRIGGER_DEBOUNCE_MS 300         // tăng/giảm tùy độ "sạch" của tín hiệu
```

Và trong hàm `khoi_tao_chan_trigger()`:
- Nếu tín hiệu kích hoạt là **mức THẤP** (ví dụ nút nhấn kéo xuống GND, đang
  dùng `pull_up_en` + `GPIO_INTR_NEGEDGE` mặc định trong code) → giữ nguyên.
- Nếu tín hiệu kích hoạt là **mức CAO** (ví dụ cảm biến xuất ra 3.3V khi kích
  hoạt) → đổi `.pull_up_en = GPIO_PULLUP_ENABLE` thành
  `.pull_down_en = GPIO_PULLDOWN_ENABLE` (và tắt `pull_up_en`), đổi
  `.intr_type = GPIO_INTR_NEGEDGE` thành `GPIO_INTR_POSEDGE`.

## Định dạng dữ liệu gửi qua LoRa

Bản tin text đơn giản (không dùng cJSON để tránh phụ thuộc ngoài):
- Có fix GPS: `TRIGGER,<so_lan_kich_hoat>,<lat>,<lon>,sats=<n>,hdop=<x>`
  Ví dụ: `TRIGGER,3,10.762622,106.660172,sats=8,hdop=0.9`
- Chưa fix GPS: `TRIGGER,<so_lan_kich_hoat>,NO_FIX`

## Build & nạp

```powershell
idf.py set-target esp32s3
idf.py -p COMx build flash monitor
```

## Lưu ý

- Cấu hình LoRa (433MHz, SF9, BW125kHz, sync word `0xF3`) đặt giống với project
  `phao_cuu_ho_idf` — nếu bên nhận là trạm bờ trong project đó, 2 bên sẽ nói
  chuyện được với nhau. Đổi cho khớp nếu bên nhận dùng cấu hình khác.
- ATK-S1216 trong phần cứng thực tế của bạn xuất NMEA ở baud **38400** (đã xác nhận
  qua test) — nếu bạn dùng module ATK-S1216 khác đang ở baud mặc định 9600, hoặc đã
  cấu hình lại module
  qua phần mềm ATK-S1216 Config Tool sang baud khác, nhớ sửa `GPS_BAUDRATE`
  trong code cho khớp.
- Nếu gặp lỗi `Khong tim thay SX127x`, xem lại checklist khắc phục đã trao đổi
  ở các project trước (dây RST/CS, nguồn 3.3V, mối hàn, dây MISO/MOSI không
  bị đảo...).

# Web Dashboard GPS qua LoRa — Bản ESP32-S3

Chuyển từ project `web_dashboard_esp32devkit_idf` sang chạy trên **ESP32-S3**.
Toàn bộ logic (WiFi Station, HTTP server, WebSocket, driver LoRa, nhận diện
tọa độ linh hoạt) giữ nguyên 100% — chỉ đổi sơ đồ chân LoRa cho phù hợp với
bộ chân an toàn của ESP32-S3.

## Sơ đồ nối dây — ESP32-S3 + LoRa RA-02 (SX1278)

```
VCC(3.3V)->3V3   SCK->GPIO12   MISO->GPIO13
GND->GND         MOSI->GPIO11  NSS->GPIO10
RST->GPIO9       DIO0->GPIO8
```

(Khác với bộ chân ESP32 DevKit trước đây vì nhiều module ESP32-S3 dùng
GPIO26-37 cho SPI Flash/PSRAM nội bộ và GPIO19/20 cho USB gốc — cần tránh.)

## Build & nạp

```powershell
idf.py set-target esp32s3
idf.py -p COMx build flash monitor
```

## Lưu ý

- WiFi SSID/mật khẩu đang để theo đúng giá trị bạn đang dùng
  (`PTNVT for Teacher` / `88888888`) — sửa lại trong `main/main.c` nếu đổi mạng.
- Cấu hình LoRa (433MHz, SF9, BW125kHz, sync word `0xF3`) phải khớp với máy
  phát — không đổi gì so với bản DevKit.
- `dashboard.html`, driver `sx127x`, và toàn bộ file CMake được tái sử dụng
  nguyên vẹn từ bản DevKit, không cần chỉnh sửa gì thêm.

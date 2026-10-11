# RedgeSCUE — Firmware STM32F103 (bo phụ của phao Pixhawk)

Trên phao Pixhawk, autopilot lái phao. Bo STM32F103C8 (Blue Pill) này lo những thứ treo quanh
Raspberry Pi: servo camera, relay báo động, nút dừng khẩn cấp, đo pin, phát LoRa về trạm bờ và gửi
áp pin lên tay cầm FS-i6. Nó **không** điều khiển motor.

```
Raspberry Pi ──USB-TTL── USART1 ─┐
                                 ├─ STM32F103 ─► servo camera, 2 relay, LoRa RA-02
FS-iA6B (i-BUS SENS) ── USART3 ──┤            ◄─ nút dừng khẩn cấp, INA226
```

- **MCU:** STM32F103C8T6, thạch anh 8 MHz, chạy 72 MHz
- **Cấu hình:** `STM32F103.ioc` là nguồn sự thật; không sửa tay `Core/` ngoài các block `USER CODE`
- **Build / nạp:** `build.bat [Debug|Release] [clean]`, `flash.bat [Debug|Release]`
  (hoặc từ gốc repo: `boat pixhawk build`, `boat pixhawk flash`)

## Pinout

| Chức năng | Ngoại vi | Chân | Ghi chú |
|---|---|---|---|
| Raspberry Pi (USB-to-TTL, `/dev/ttyUSB0` trên Pi) | USART1 | PA9 TX, PA10 RX | 115200 8N1; module 3.3 V, chỉ nối TXD/RXD/GND |
| Sensor iBus của FS-iA6B | USART3 | PB10 | một dây, half-duplex, 115200; cổng **i-BUS SENS** của receiver |
| LoRa RA-02 (SX1278) | SPI1 | PA5 SCK, PA6 MISO, PA7 MOSI | mode 0, 2.25 Mbit/s; NSS PA4, RST PB0, DIO0 PB1 (chưa dùng) |
| INA226 | I2C1 | PB6 SCL, PB7 SDA | 100 kHz, địa chỉ 0x44, shunt 5 mΩ |
| Servo camera | TIM2_CH3 | PA2 | 50 Hz, 1 tick = 1 µs, giữa 1500 µs |
| Relay 1, 2 | GPIO | PA1, PA0 | mức cao = đèn bật (module kích mức thấp, đèn ở tiếp điểm NC) |
| Nút dừng khẩn cấp | GPIO | PB9 | pull-up nội, nhấn = nối GND |
| LED | GPIO | PC13 | nháy 1 lần/giây khi Pi đang nói chuyện, 5 lần/giây khi không |
| Debug | SWD | PA13, PA14 | ST-LINK |

**Gắn anten 433 MHz vào RA-02 trước khi cấp nguồn**: phát không anten có thể làm hỏng tầng công suất.
Chưa lắp module nào thì đặt cờ tương ứng trong `App/app_config.h` về 0 (`LORA_ENABLED`, `INA226_ENABLED`);
để nguyên cũng không sao, firmware báo thiếu và chạy tiếp.

## Cấu trúc

```
main.c  →  App/  →  Modules/  →  HAL
```

| File | Việc |
|---|---|
| `App/app.c` | phần dính phần cứng: đọc chân và cảm biến, ghi servo/relay/LED, UART, phát LoRa |
| `App/helper_logic.c` | mọi quyết định của bo (servo, báo động, nút dừng, nội dung beacon và báo cáo), không phụ thuộc HAL |
| `App/app_config.h` | mọi hằng số chỉnh được: chiều servo, thời gian báo động, tham số LoRa |
| `App/lora_frame.c` | dựng dòng beacon, cùng định dạng với phao STM32F4 |
| `Modules/PiLink` | giao thức dòng với Pi (`$...*CS`), không phụ thuộc HAL |
| `Modules/IbusSensor` | trả lời receiver như một cảm biến điện áp, không phụ thuộc HAL |
| `Modules/LoRa` | driver SX127x, giống `STM32F4/Modules/LoRa` |
| `Modules/INA226` | đọc áp và dòng |

## Giao thức với Raspberry Pi

Cùng khung với phao STM32F4: `$BODY*CS\n`, CS là XOR các ký tự của BODY (`Rasp_Pi/utils/nmea_packet.py`).

Pi → bo (`Rasp_Pi/modules/helper_board.py`):

```
$HBT,seq,state                 Pi còn sống (mỗi giây)
$CAM,seq,pan_deg               góc servo camera tính từ mũi phao, dương là sang phải
$VIC,seq,count                 số người đã tìm thấy (số tuyệt đối, lặp lại mỗi giây)
$POS,seq,fix,lat,lon,roll,pitch,yaw,target,mode,imu_ok,calib
```

Bo → Pi, mỗi 0.5 giây:

```
$AUX,seq,battery_mv,current_ma,estop,lora_ok,alarm
```

Hành vi:

- **Servo camera** theo `$CAM`; quá 2 giây không có gói thì nhìn thẳng. Quay ngược phía người: đổi
  `CAM_PAN_DIR`.
- **Relay** bật cả hai trong `ALARM_HOLD_MS` (30 giây) mỗi khi `count` tăng. Số giảm (Pi khởi động lại)
  không báo động.
- **Nút dừng khẩn cấp**: bo chỉ báo `estop=1` cho Pi, Pi yêu cầu autopilot vào HOLD
  (`rescue.estop_mode`). Đứt dây thì đọc như chưa nhấn: đây là nút yêu cầu, không phải mạch an toàn.
  Muốn dừng chắc chắn thì dùng công tắc trên tay điều khiển hoặc cắt nguồn motor.
- **LoRa** phát mỗi 5 giây dòng beacon dựng từ gói `$POS` gần nhất; `count` tăng thì phát ngay. Quá
  3 giây không có `$POS` thì phát `NO_FIX` để trạm bờ vẫn thấy bo và số người.
- **Tay cầm FS-i6** hiện áp pin từ INA226 (cảm biến "Ext.V", địa chỉ 1).

## Kiểm tra

Test trên máy tính cho phần không dính HAL (cần gcc):

```
powershell -File Tools\run_host_tests.ps1
```

Xem trạng thái bo đang chạy qua ST-LINK, không dừng chương trình:

```
python Tools\helper_monitor.py                # cập nhật liên tục
python Tools\helper_monitor.py --csv run.csv  # ghi thêm ra file
```

Nó đọc biến `helper_debug` (`App/app.h`): Pi có đang nói chuyện không và bao nhiêu dòng hỏng, xung
servo, số người và báo động, nút dừng, áp và dòng pin, chip LoRa (phải là `0x12`) cùng số beacon đã
gửi/lỗi, số lần receiver hỏi cảm biến. Trình tự đưa lên phao: `docs/porting-checklist.md`.

## Trong giả lập

`Tests/helper_sim.c` biên dịch chính `helper_logic.c`, `pi_link.c`, `lora_frame.c` cho máy tính;
`sim/tools/helper_board_sim.py` đặt nó sau một cổng nối tiếp ảo để code Pi nói chuyện với nó như với
bo thật, và đưa xung servo vào khớp camera của Gazebo (`sim/run_boat.sh --pi --hw`).

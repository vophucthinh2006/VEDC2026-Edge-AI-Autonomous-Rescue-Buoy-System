# RedgeSCUE — STM32F407 firmware

STM32 là lớp điều khiển thời gian thực. Firmware nhận mục tiêu `NAV` từ Raspberry Pi,
đọc BNO055/GPS/RC receiver, áp dụng watchdog và chỉ sau đó mới phát PWM tới ESC/servo.

## Cấu trúc

```
STM32F4/
├── STM32F4.ioc          cấu hình CubeMX (nguồn sự thật cho pinout)
├── Core/                code CubeMX sinh — không sửa ngoài block USER CODE
├── Drivers/             CMSIS + STM32F4xx HAL
├── Modules/             mỗi thư mục con là một thiết bị ngoại vi
│   ├── Actuators/       ESC + servo qua TIM PWM
│   ├── BNO055/          driver BNO055 (I2C) + calibration profile
│   ├── GPS/             bộ phân tích NMEA
│   ├── IBUS/            bộ phân tích khung iBUS của FS-iA6B
│   └── PiLink/          mã hoá/giải mã khung với Raspberry Pi
├── App/                 tầng ứng dụng
│   ├── control.c        vòng điều khiển + watchdog an toàn
│   ├── imu.c            lớp đệm trên driver BNO055
│   └── app_config.h     ngưỡng, timeout, hằng số PWM
├── Tools/               dump calibration, dashboard STM32CubeMonitor
├── build.bat            cmake + ninja → build/Debug/STM32F4.elf
└── flash.bat            nạp qua ST-LINK
```

Phân tầng: `main.c` chỉ gọi API của `App/`, `App/` gọi xuống `Modules/`, và
`Modules/` không biết gì về tầng ứng dụng — trừ `app_config.h`, xem ghi chú dưới.

`App/imu.c` là lớp đệm giữa driver BNO055 và vòng điều khiển: nó giữ
`bno055_euler_t` phẳng mà `control.c` dùng, và map `yaw` của cảm biến thành
`heading_deg` của bộ điều khiển. Nhờ vậy `Modules/BNO055/` vẫn là driver thuần,
bê sang dự án khác được ngay.

> `App/app_config.h` hiện được `Modules/Actuators` và `Modules/PiLink` include
> (hằng số `ESC_*`, `SERVO_*`, `NAV_TIMEOUT_MS`), tức là có một mũi tên ngược từ
> Modules lên App. Chấp nhận được ở quy mô này; nếu sau cần tách sạch thì đưa
> hằng số phần cứng của từng module về chính thư mục module đó.

## Pinout

Board: **STM32F407G-Discovery** (STM32F407VGTx, LQFP100).

| Chức năng | Peripheral / chân | Nối tới |
|---|---|---|
| UART Raspberry | USART1: PA9 TX, PA10 RX, 115200 8N1 | Pi GPIO15/RXD0 ← PA9; Pi GPIO14/TXD0 → PA10; GND chung |
| BNO055 | I2C1: **PB8 SCL, PB9 SDA**, 400 kHz | SCL/SDA, 3.3 V, GND; pull-up 4.7 kΩ lên 3.3 V nếu module chưa có |
| BNO055 reset | PB5 output, mức cao khi chạy | chân nRESET của module |
| GPS Holybro M10 V2 | USART3: PB10 TX, PB11 RX, 9600 | GPS TX → PB11; GPS RX ← PB10; 3.3 V/GND |
| FS-iA6B iBUS | USART6: PC6 TX, PC7 RX, 115200 | iBUS → PC7; GND chung |
| ESC trái/phải/trước | TIM3 CH1 PA6 / CH2 PA7 / CH3 PB0 | chỉ chân signal + GND với ESC |
| Servo 1/2/3 | TIM3 CH4 PB1 / TIM4 CH1 PB6 / CH2 PB7 | signal servo; cấp nguồn servo độc lập |
| E-stop ngoài | **PA2** input pull-up, tiếp điểm **thường đóng** xuống GND | nút NC → GND |
| SOS/relay test | PD13 output (LED cam trên Discovery) | qua transistor/driver, không nối tải trực tiếp |
| Accelerometer CS | PE3 output, giữ mức cao | không nối gì — xem ghi chú bên dưới |
| Battery sense | ADC1 IN4 PA4 — mới đặt chân, chưa dùng trong code | qua cầu chia áp tính đúng điện áp pin |
| Debug | SWD: PA13 SWDIO, PA14 SWCLK | ST-LINK trên board (CN1) |

TIM3/TIM4 chạy 1 MHz, period `19999` → PWM 50 Hz; giá trị compare là microsecond.
USART1/3/6 bật RX interrupt. Địa chỉ I2C của BNO055 là **0x29** (ADR thả nổi).

### Ba ràng buộc của board Discovery

- **PE3 phải luôn ở mức cao.** Cảm biến LIS3DSH trên board dùng chung SPI1 với
  PA6/PA7. PA6 là MISO — chân *xuất* của cảm biến. Nếu PE3 không được giữ cao,
  cảm biến được chọn và sẽ đẩy tín hiệu chống lại PWM của TIM3 CH1. `MX_GPIO_Init()`
  đặt PE3 lên cao trước khi timer chạy. Không bật SPI1.
- **Không cắm gì vào cổng USB OTG (CN5).** PA9/PA10 là VBUS/ID của cổng đó;
  VBUS 5 V sẽ chống lại chân USART1_TX. Chỉ dùng CN1 (ST-LINK) để nạp và cấp nguồn.
- **PB9 đã có pull-up sẵn** (bus I2C của codec CS43L22 trên board), PB8 thì không.
  Codec nằm cùng bus ở địa chỉ 0x4A, bị giữ reset bởi PD4 nên không quấy bus.
  Không bật I2S2/I2S3 vì PC7 và PB10 dính vào codec/micro.

### E-stop: đấu dây thường đóng

PA2 bật pull-up nội và firmware coi **mức cao là E-stop tác động**. Nút phải là
loại **thường đóng (NC)** nối PA2 xuống GND:

| Tình huống | PA2 | Kết quả |
|---|---|---|
| Bình thường, nút đóng | thấp | chạy |
| Nhấn nút, tiếp điểm mở | cao | dừng |
| **Đứt dây / tuột jack** | cao | **dừng** |

Đấu ngược lại (thường mở lên 3.3 V) sẽ khiến dây đứt bị hiểu là "không nhấn"
và chân vịt vẫn quay.

### Clock

Hiện chạy **HSI 16 MHz** → PLL → 168 MHz. Đủ cho test bàn, nhưng HSI trôi theo
nhiệt độ và ăn vào ngân sách sai lệch baud ở 115200. **Chuyển sang HSE 8 MHz
(PLLM=8, PLLN=336, PLLP=2, PLLQ=7) trước khi thử ngoài nước.** Nếu HSE timeout
thì đổi RCC sang `BYPASS Clock Source`, vẫn 8 MHz.

## Nguồn và an toàn phần cứng

- GY-BNO055 nhận 3–5 V nhưng bus I2C của STM32 là 3.3 V: cấp module ở 3.3 V là
  phương án an toàn; không để pull-up SDA/SCL lên 5 V.
- Không cấp DS51150 12 V hoặc ESC từ chân Discovery. UBEC/power board phải cấp
  đúng áp và đủ dòng; tất cả thiết bị tín hiệu cần chung GND.
- Pixhawk 4 không được nối đồng thời các PWM điều khiển với STM32. Firmware này
  giả định STM32 là **duy nhất** phát PWM. Giữ Pixhawk tách khỏi đường signal
  hoặc chuyển toàn bộ quyền điều khiển sang Pixhawk; không chạy hai autopilot song song.
- Mặc định `ACTUATORS_ENABLED` bằng 0. Sau khi xác nhận chiều motor/servo trên
  giá đỡ, đổi sang 1 trong `App/app_config.h`, build và hiệu chuẩn ESC.
  Không tháo chân vịt khi kiểm thử bàn.

## Giao thức khớp Raspberry Pi

Raspberry → STM32: `$NAV,seq,AUTO|STOP|MANUAL,speed_mps,heading_deg,ttl_ms*CS\n`,
`$HBT,seq,state*CS\n`, `$SOS,seq,0|1*CS\n`, `$TXD,seq,event,lat,lon,confidence*CS\n`.

STM32 → Raspberry: `$IMU,seq,pitch,roll,yaw,imu_ok,overturned*CS\n`,
`$GPS,seq,lat,lon,fix,hdop,speed,course*CS\n`,
`$SYS,seq,battery,motor_fault,estop,link_ok*CS\n`.

Checksum là XOR ASCII giữa `$` và `*`, giống `Rasp_Pi/utils/nmea_packet.py`.
`NAV` và `HBT` phải mới hơn 300 ms; nếu lỗi checksum, E-stop, BNO055 lỗi, nghiêng
quá ngưỡng hoặc receiver mất link, output ESC lập tức về stop.

## Build và nạp

```
build.bat            # → build\Debug\STM32F4.elf
flash.bat            # nạp qua ST-LINK
```

Sau khi hiệu chuẩn BNO055 tới 3/3/3/3, chạy `Tools\bno055_dump_calib.bat` để
sinh lại `Modules/BNO055/bno055_calib_profile.h` và commit profile mới.

## Cấu hình lại pinout

Sửa `STM32F4.ioc` trong STM32CubeMX rồi Generate Code — đừng sửa tay `Core/`.
Code trong `Modules/` và trong các block `USER CODE` của `Core/Src/main.c` được giữ nguyên.

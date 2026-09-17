# RedgeSCUE STM32F407G-Discovery firmware

STM32 là lớp điều khiển thời gian thực. Firmware này nhận mục tiêu `NAV` từ Raspberry Pi, đọc BNO055/GPS/RC receiver, áp dụng watchdog và chỉ sau đó mới phát PWM tới ESC/servo.

## Cấu hình CubeMX bắt buộc

Tạo dự án STM32CubeIDE cho **STM32F407VGTx**, HAL, SYS timebase SysTick, clock 168 MHz. CubeMX phải tạo `main.c`, `main.h`, các `MX_*_Init()` và các handle với đúng tên dưới đây. Sau đó thay `Core/Src/main.c` và thêm các file trong thư mục này.

| Chức năng | Peripheral / chân STM32 | Nối tới |
|---|---|---|
| UART Raspberry | USART1: PA9 TX, PA10 RX, 115200 8N1 | Pi GPIO15/RXD0 <- PA9; Pi GPIO14/TXD0 -> PA10; GND chung |
| BNO055 | I2C1: PB8 SCL, PB9 SDA, 400 kHz | SCL/SDA, 3.3 V, GND; pull-up 4.7 kOhm tới 3.3 V nếu module chưa có |
| GPS Holybro M10 V2 | USART3: PB10 TX, PB11 RX, 9600 hoặc đúng baud module | GPS TX -> PB11; GPS RX <- PB10; 3.3 V/GND |
| FS-iA6B iBUS | USART6: PC6 TX, PC7 RX, 115200 | iBUS -> PC7; GND chung |
| ESC trái/phải/trước | TIM3 CH1 PA6 / CH2 PA7 / CH3 PB0 | Chỉ chân signal + GND với ESC |
| Servo 1/2/3 | TIM3 CH4 PB1 / TIM4 CH1 PB6 / CH2 PB7 | Signal servo; cấp nguồn servo độc lập |
| E-stop ngoài | PA0 input pull-up, active-low | Nút -> GND; PA0 cũng là user button của Discovery |
| SOS/relay test | PD13 output | Qua transistor/driver, không nối tải trực tiếp |
| Battery sense (tùy chọn) | ADC1 IN4 PA4 | Qua cầu chia áp có tính toán đúng điện áp pin |

TIM3/TIM4 phải chạy **1 MHz**, period `19999` để tạo PWM 50 Hz; giá trị compare là microsecond. USART1/3/6 bật RX interrupt. I2C address BNO055 mặc định 0x28 (ADR=GND).

## Nguồn và an toàn phần cứng

- GY-BNO055 nhận 3-5 V nhưng bus I2C của STM32 là 3.3 V: cấp module ở 3.3 V là phương án an toàn; không để pull-up SDA/SCL lên 5 V.
- Không cấp DS51150 12 V hoặc ESC từ chân Discovery. UBEC/power board phải cấp đúng áp và đủ dòng; tất cả thiết bị tín hiệu cần chung GND.
- Pixhawk 4 không được nối đồng thời các PWM điều khiển với STM32. Firmware này giả định STM32 là **duy nhất** phát PWM. Giữ Pixhawk tách khỏi đường signal hoặc chuyển toàn bộ quyền điều khiển sang Pixhawk; không chạy hai autopilot song song.
- Mặc định `ACTUATORS_ENABLED` bằng 0. Sau khi xác nhận chiều motor/servo trên giá đỡ, đổi sang 1 trong `app_config.h`, build và hiệu chuẩn ESC. Không tháo chân vịt khi kiểm thử bàn.

## Giao thức khớp Raspberry Pi

Raspberry -> STM32: `$NAV,seq,AUTO|STOP|MANUAL,speed_mps,heading_deg,ttl_ms*CS\n`, `$HBT,seq,state*CS\n`, `$SOS,seq,0|1*CS\n`, `$TXD,seq,event,lat,lon,confidence*CS\n`.

STM32 -> Raspberry: `$IMU,seq,pitch,roll,yaw,imu_ok,overturned*CS\n`, `$GPS,seq,lat,lon,fix,hdop,speed,course*CS\n`, `$SYS,seq,battery,motor_fault,estop,link_ok*CS\n`.

Checksum là XOR ASCII giữa `$` và `*`, giống `Rasp_Pi/utils/nmea_packet.py`. `NAV` và `HBT` phải mới hơn 300 ms; nếu lỗi checksum, E-stop, BNO055 lỗi, nghiêng quá ngưỡng hoặc receiver mất link, output ESC lập tức về stop.

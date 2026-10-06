---
name: stm32-cubemx-first
description: Quy tắc bắt buộc trước khi sửa code STM32 (thư mục STM32F4, file .ioc, Core/, Drivers/, HAL, clock, GPIO, timer, UART, I2C, SPI, ADC, DMA, NVIC, PWM, ngắt). Dùng mỗi khi người dùng yêu cầu thêm tính năng, sửa lỗi hoặc thay đổi code liên quan STM32: phải yêu cầu người dùng cấu hình trong STM32CubeMX (.ioc) và Generate code trước, chỉ viết code sau khi người dùng xác nhận.
---

# STM32: cấu hình CubeMX trước, viết code sau

Project STM32 được sinh bởi STM32CubeMX từ `STM32F4/STM32F4.ioc`. Nếu sửa tay phần cấu hình thì lần Generate sau sẽ bị ghi đè hoặc lệch với `.ioc`.

## Quy trình

1. **Phân tích yêu cầu**: đọc code và `STM32F4.ioc` để xác định phần nào cấu hình được trong CubeMX (clock, pin/GPIO, peripheral, timer/PWM, UART/I2C/SPI, ADC, DMA, NVIC/ưu tiên ngắt, middleware...).
2. **Nếu có phần cần cấu hình**: chưa sửa code. Gửi người dùng danh sách cụ thể cần chỉnh trong CubeMX, ví dụ:
   - Peripheral và chế độ (ví dụ `TIM3 → Channel 1: PWM Generation CH1`).
   - Chân (ví dụ `PA6 → TIM3_CH1`), label nếu cần.
   - Tham số (prescaler, period, baud rate, clock speed, mode...) kèm giá trị và lý do.
   - DMA, NVIC (bật ngắt, mức ưu tiên) nếu cần.
   Sau đó yêu cầu người dùng cấu hình, **Generate code**, rồi xác nhận lại.
3. **Chờ xác nhận**: chỉ bắt đầu viết code khi người dùng xác nhận đã Generate xong. Khi đó đọc lại `.ioc` và các file sinh ra (`Core/Src/main.c`, `*_it.c`, `stm32f4xx_hal_msp.c`...) để kiểm tra cấu hình đúng như đã yêu cầu; nếu lệch thì báo lại trước khi code.
4. **Viết code**:
   - Code trong file sinh ra chỉ đặt giữa các cặp `/* USER CODE BEGIN ... */` và `/* USER CODE END ... */`.
   - Ưu tiên đặt logic trong thư mục riêng của project (`App/`, `Modules/`) thay vì trong file CubeMX sinh ra.
   - Không sửa tay các hàm `MX_*_Init()` hoặc `HAL_*_MspInit()` để đổi cấu hình; mọi thay đổi cấu hình đi qua `.ioc`.

## Khi nào được bỏ qua bước CubeMX

Nếu thay đổi hoàn toàn không đụng tới cấu hình phần cứng (chỉ là logic ứng dụng, thuật toán, tài liệu, script trong `Tools/`), nói rõ với người dùng là không cần chỉnh CubeMX rồi làm luôn.

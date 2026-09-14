# AGENTS.md — STM32F4

## Quy tắc làm việc

### 1. Cấu hình STM32CubeMX trước, code sau
- Khi nhận yêu cầu chỉnh sửa, nếu phần việc có thể cấu hình được trong STM32CubeMX
  (clock, pin, peripheral, NVIC, DMA, middleware...), hãy yêu cầu người dùng cấu hình
  trong `STM32F4.ioc` trước. Nêu rõ cần chỉnh những gì (peripheral, chân, tham số).
- Chỉ bắt đầu viết code sau khi người dùng xác nhận đã cấu hình và generate code xong.

### 2. Tạo Pull Request
- Chia thay đổi thành nhiều commit hợp lý theo từng phần logic, không gộp thành một commit.
- Mỗi PR có Description ngắn gọn.

---
name: stm32f4-build-flash
description: Build và nạp firmware STM32F4 (CMake + Ninja + arm-none-eabi-gcc, nạp qua ST-LINK/OpenOCD). Dùng khi người dùng muốn build, compile, flash, nạp code, hoặc kiểm tra firmware STM32F4 có biên dịch được không.
---

# Build & flash firmware STM32F4

Firmware nằm trong `STM32F4/`. Dùng các script có sẵn, không tự gọi cmake/openocd thủ công trừ khi script lỗi.

## Build

```powershell
STM32F4\build.bat            # Debug (-O0 -g3), mặc định
STM32F4\build.bat Release    # Release (-Os)
STM32F4\build.bat Debug clean  # xóa build\Debug rồi build lại
```

- Output: `STM32F4/build/<type>/STM32F4.elf`, `.hex`, `.bin`.
- Toolchain mặc định: `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin`; đổi bằng biến `ARM_GCC_BIN`.
- Cần `cmake` và `ninja` trong PATH.
- Sau khi build xong, báo lại kích thước flash/RAM từ dòng `arm-none-eabi-size`.

## Flash

```powershell
STM32F4\flash.bat            # nạp build\Debug\STM32F4.elf
STM32F4\flash.bat Release
```

- Nạp qua ST-LINK + OpenOCD (`interface/stlink.cfg`, `target/stm32f4x.cfg`), có verify và reset.
- Đổi đường dẫn OpenOCD bằng biến `OPENOCD`.
- Flash là thao tác lên phần cứng thật: chỉ chạy khi người dùng yêu cầu nạp.

## Khi lỗi

- Lỗi compile: đọc lỗi đầu tiên trong output, sửa code, build lại.
- `[FLASH] FAILED`: nhắc người dùng kiểm tra kết nối ST-LINK, nguồn board và dây SWD.
- Thay đổi clock/pin/peripheral: theo skill `stm32-cubemx-first`, yêu cầu người dùng cấu hình trong `STM32F4.ioc` trước.

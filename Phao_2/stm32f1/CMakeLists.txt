cmake_minimum_required(VERSION 3.22)
project(Phao2 C ASM)
set(CMAKE_C_STANDARD 11)
set(BASE ${CMAKE_CURRENT_SOURCE_DIR}/../../STM32F103)
set(HAL ${BASE}/Drivers/STM32F1xx_HAL_Driver)
add_executable(Phao2 main.c sx127x.c
  ${BASE}/startup_stm32f103xb.s
  ${BASE}/Core/Src/system_stm32f1xx.c
  ${BASE}/Core/Src/syscalls.c ${BASE}/Core/Src/sysmem.c)
foreach(module hal hal_rcc hal_rcc_ex hal_gpio hal_gpio_ex hal_dma hal_cortex hal_pwr hal_flash hal_flash_ex hal_spi hal_uart)
  target_sources(Phao2 PRIVATE ${HAL}/Src/stm32f1xx_${module}.c)
endforeach()
target_include_directories(Phao2 PRIVATE . ${BASE}/Core/Inc ${HAL}/Inc
  ${BASE}/Drivers/CMSIS/Include ${BASE}/Drivers/CMSIS/Device/ST/STM32F1xx/Include)
set(GPS_BAUD 115200 CACHE STRING "Actual CAN-UART converter output baud rate")
target_compile_definitions(Phao2 PRIVATE USE_HAL_DRIVER STM32F103xB GPS_BAUD=${GPS_BAUD})
target_compile_options(Phao2 PRIVATE -mcpu=cortex-m3 -mthumb -Os -Wall -Wextra -ffunction-sections -fdata-sections)
target_link_options(Phao2 PRIVATE -mcpu=cortex-m3 -mthumb --specs=nano.specs --specs=nosys.specs
  -T${BASE}/STM32F103xx_FLASH.ld -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=Phao2.map)
target_link_libraries(Phao2 m)
set_target_properties(Phao2 PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET Phao2 POST_BUILD
  COMMAND arm-none-eabi-objcopy -O ihex $<TARGET_FILE:Phao2> Phao2.hex
  COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:Phao2> Phao2.bin)

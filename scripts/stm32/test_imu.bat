@echo off
rem ============================================================
rem  BNO055 on the STM32F407: heading with the bow as -x, and the overturn detection.
rem  Builds and flashes the firmware (skip with noflash), then shows the heading loop live
rem  over ST-LINK (STM32F4\Tools\heading_monitor.py).
rem
rem  Usage: test_imu [noflash]
rem
rem  What to do, what to see:
rem    1. point the bow (the BNO055's -x axis) north: yaw near 0 (compass app on a phone);
rem       east: near 90. Off by about 90 deg: IMU_MOUNTING_OFFSET_DEG in App\app_config.h
rem    2. turn the boat clockwise seen from above: yaw grows, yaw rate positive
rem    3. tilt it past 35 deg: arm_block 2 (overturned), the buzzer beeps twice; with the
rem       shore station on, the dashboard shows the overturn alert (LoRa event T)
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
if /I not "%~1"=="noflash" (
    call "%ROOT%STM32F4\build.bat" Debug || goto :fail
    call "%ROOT%STM32F4\flash.bat" Debug || goto :fail
)
pushd "%ROOT%STM32F4"
python Tools\heading_monitor.py
popd
exit /b 0
:fail
echo [FAILED]
exit /b 1

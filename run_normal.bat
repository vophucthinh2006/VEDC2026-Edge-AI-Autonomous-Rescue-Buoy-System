@echo off
rem ============================================================
rem  Back to the normal program of the STM32F407 boat on its stand (after motor_test.bat):
rem    1. builds and flashes the STM32F407's own firmware (STM32F4\build.bat, flash.bat: Debug)
rem    2. on the Pi, restarts main.py with the stand settings and shows the viewer full
rem       screen on the HDMI screen (Rasp_Pi/tools/bench_demo.sh)
rem
rem  Usage: run_normal [noflash]
rem    noflash : only step 2, the firmware on the board is left as it is
rem
rem  The Pi's address comes from boat.local.cmd. The firmware is built as App\app_config.h
rem  stands: committed for the water (RC arms, IMU required). On the stand with no RC and
rem  the BNO055 out, set BENCH_IGNORE_IMU and BENCH_PE4_ARM to 1 first (and ACTUATORS_ENABLED
rem  to 0 to keep the motors off); PE4 then arms. Set them back before the water.
rem ============================================================
setlocal EnableExtensions
set "ROOT=%~dp0"

if /I "%~1"=="noflash" goto :pi
call "%ROOT%STM32F4\build.bat" Debug
if errorlevel 1 goto :fail
call "%ROOT%STM32F4\flash.bat" Debug
if errorlevel 1 goto :fail

:pi
if not exist "%ROOT%boat.local.cmd" (
    echo [ERROR] boat.local.cmd not found: copy boat.local.example.cmd and fill it in
    exit /b 1
)
call "%ROOT%boat.local.cmd"
echo [PI] %PI_HOST%: main.py and the HDMI viewer
ssh %PI_HOST% "cd %PI_DIR% && bash tools/bench_demo.sh"
if errorlevel 1 goto :fail
echo [OK] normal program running. Arm with the RC (or PE4 with BENCH_PE4_ARM 1).
exit /b 0

:fail
echo [FAILED]
exit /b 1

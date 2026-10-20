@echo off
rem ============================================================
rem  INA226 on I2C1 (next to the BNO055, address 0x40), live over ST-LINK
rem  (STM32F4\Tools\battery_monitor.py). Flash the firmware first.
rem
rem  Usage: test_battery [once]
rem
rem  What to see: the pack voltage within 0.1 V of a multimeter, charge in %.
rem  "INA226 not found": wiring of SCL PB8 / SDA PB9, A0/A1 not on GND, no 3.3 V.
rem  The current is right only once INA226_SHUNT_OHM in App\app_config.h is your module's shunt.
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
pushd "%ROOT%STM32F4"
if /I "%~1"=="once" (python Tools\battery_monitor.py --once) else (python Tools\battery_monitor.py)
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%

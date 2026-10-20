@echo off
rem ============================================================
rem  LoRa between the STM32F407 and the shore station, live over ST-LINK
rem  (STM32F4\Tools\lora_monitor.py). Flash the firmware first (test_imu or STM32F4\flash.bat).
rem
rem  Usage: test_lora [once]
rem
rem  What to see, with the station running (test_station.bat):
rem    chip 0x12 OK, tx growing every 5 s, fail 0
rem    station OK, ACK growing with every beacon, home = the station's GPS once it has a fix
rem    press PE4: events 1 sent, 0 waiting a few seconds later (the station acknowledged it)
rem    switch the station off: station LOST after 60 s
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
pushd "%ROOT%STM32F4"
if /I "%~1"=="once" (python Tools\lora_monitor.py --once) else (python Tools\lora_monitor.py)
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%

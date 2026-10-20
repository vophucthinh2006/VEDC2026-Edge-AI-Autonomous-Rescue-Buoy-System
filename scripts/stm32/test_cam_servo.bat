@echo off
rem ============================================================
rem  The camera pan servo swept the way it is on a route: back and forth between -DEGREES and
rem  +DEGREES at RATE deg/s (30 like the Pi), for SECONDS, through the normal firmware
rem  (STM32F4\Tools\cam_servo_test.ps1). No motor turns. Stop main.py on the Pi first.
rem  Usage: test_cam_servo [DEGREES SECONDS [RATE]]     e.g. test_cam_servo 45 10
rem  Flashes the normal Debug firmware first if the board runs another image (e.g. HwTest).
rem ============================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\..\STM32F4\Tools\cam_servo_test.ps1" %*
exit /b %ERRORLEVEL%

@echo off
rem ============================================================
rem  The ESP32-S3 shore station: build, flash and watch its log (ESP-IDF).
rem  Needs IDF_EXPORT (ESP-IDF's export.bat) and ESP_PORT in boat.local.cmd, and
rem  esp32-lora-station\main\secrets.h (copy secrets.example.h).
rem
rem  Usage: test_station [build|flash|monitor]     default: build flash monitor
rem
rem  What to see in the log, with the boat on:
rem    [NHAN] id=PHAO-01 ...        a beacon heard
rem    [ACK] da phat -> PHAO-01     the answer that keeps the boat's link (and its lights)
rem    [SU KIEN] ...                a person, an overturn, the confirm button
rem    [CLOUD] da gui               posted to the Worker
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
if not defined IDF_EXPORT (
    echo [ERROR] IDF_EXPORT not set in boat.local.cmd, e.g. C:\Espressif\frameworks\esp-idf-v5.3\export.bat
    exit /b 1
)
if not exist "%ROOT%esp32-lora-station\main\secrets.h" (
    echo [ERROR] esp32-lora-station\main\secrets.h missing: copy secrets.example.h and fill it in
    exit /b 1
)
rem IDF_EXPORT_ARGS: for the Windows installer's idf_cmd_init.bat, the esp-idf id it needs.
call "%IDF_EXPORT%" %IDF_EXPORT_ARGS% >nul 2>&1 || exit /b 1
set "STEPS=%*"
if "%STEPS%"=="" set "STEPS=build flash monitor"
idf.py -C "%ROOT%esp32-lora-station" -p %ESP_PORT% %STEPS%
exit /b %ERRORLEVEL%

@echo off
rem ============================================================
rem  Shared by scripts\stm32\*.bat: the repository root and the local settings.
rem    call "%~dp0_env.cmd"        settings only
rem    call "%~dp0_env.cmd" pi     also fails when the Pi's address is not set
rem
rem  boat.local.cmd (copy of boat.local.example.cmd, not committed) gives:
rem    PI_HOST, PI_DIR      the Raspberry Pi (ssh)
rem    ESP_PORT, IDF_EXPORT the shore station's COM port, ESP-IDF's export.bat
rem    CLOUD_URL            the Worker (dashboard), for test_light.bat and run_all.bat
rem    ADMIN_API_TOKEN      the Worker's ADMIN_API_TOKEN secret, for test_light.bat
rem    AUDIO_DEVICE         the Pi's sound output for aplay
rem ============================================================
for %%I in ("%~dp0..\..") do set "ROOT=%%~fI\"
set "PI_DIR=vedc/Rasp_Pi"
set "ESP_PORT=COM5"
set "CLOUD_URL=http://127.0.0.1:8787"
set "AUDIO_DEVICE=plughw:CARD=Headphones,DEV=0"
set "BUOY_ID=PHAO-01"
if exist "%ROOT%boat.local.cmd" call "%ROOT%boat.local.cmd"
rem The Pi's Python, its .venv when there is one: prefix of a command run there over ssh.
set "PI_PY=PY=python3; [ -x .venv/bin/python3 ] && PY=.venv/bin/python3;"
if /I not "%~1"=="pi" exit /b 0
if not defined PI_HOST (
    echo [ERROR] PI_HOST not set: copy boat.local.example.cmd to boat.local.cmd and fill it in
    exit /b 1
)
exit /b 0

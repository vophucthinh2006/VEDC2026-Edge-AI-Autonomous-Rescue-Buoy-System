@echo off
rem ============================================================
rem  The Pi <-> STM32F407 UART: names what answers on each serial port of the Pi
rem  (tools/port_probe.py), then counts the F407's lines for 5 s on serial.stm32_port
rem  (tools/stm32_lines.py): $IMU, $GPS, $SYS 10 a second, $LNK once a second. Stops main.py first.
rem
rem  Usage: test_pi_link [noprobe]
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd" pi || exit /b 1
if /I not "%~1"=="noprobe" (
    ssh %PI_HOST% "cd %PI_DIR% && pkill -f '[m]ain.py'; %PI_PY% $PY tools/port_probe.py"
)
ssh %PI_HOST% "cd %PI_DIR% && pkill -f '[m]ain.py'; %PI_PY% $PY tools/stm32_lines.py"
exit /b %ERRORLEVEL%

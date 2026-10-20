@echo off
rem ============================================================
rem  Every test that runs on this computer, no hardware:
rem    1. Pi unit tests (Rasp_Pi\tests)
rem    2. C host tests: F407 heading, camera, button, route, shore link, events, with the
rem       station's parser and frame builders (STM32F4\Tools\run_host_tests.ps1)
rem    3. Worker tests (esp32-lora-station\cloud, npm test); skipped without Node.js
rem
rem  Usage: test_host
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
set "R1=FAIL" & set "R2=FAIL" & set "R3=FAIL"

echo [1/3] Pi unit tests
pushd "%ROOT%Rasp_Pi"
python -m unittest discover -s tests
if not errorlevel 1 set "R1=OK"
popd

echo [2/3] C host tests
powershell -NoProfile -ExecutionPolicy Bypass -File "%ROOT%STM32F4\Tools\run_host_tests.ps1"
if not errorlevel 1 set "R2=OK"

echo [3/3] Worker tests
where npm >nul 2>nul
if errorlevel 1 (
    set "R3=SKIP (no npm)"
) else (
    pushd "%ROOT%esp32-lora-station\cloud"
    call npm test
    if not errorlevel 1 set "R3=OK"
    popd
)

echo.
echo   Pi unit tests   %R1%
echo   C host tests    %R2%
echo   Worker tests    %R3%
if "%R1%%R2%"=="OKOK" if not "%R3%"=="FAIL" (
    echo [TEST] OK
    exit /b 0
)
echo [TEST] FAILED
exit /b 1

@echo off
rem ============================================================
rem  The two lights (RELAY1 PC3, RELAY2 PC2), the way the admin switches them: through the Worker
rem  (POST /api/lights), the shore station's ACK over LoRa, the F407. Watches the relays over
rem  ST-LINK until they match (STM32F4\Tools\lora_monitor.py --wait-lights).
rem
rem  Usage: test_light [MASK]
rem    test_light       1 (light 1), 2 (light 2), 3 (both), 0 (off), 20 s each
rem    test_light 1     only light 1 on
rem
rem  Needs CLOUD_URL and ADMIN_API_TOKEN in boat.local.cmd, the station running and on WiFi,
rem  the F407 flashed and on ST-LINK. Each switch takes 5 to 10 s (the station asks the Worker
rem  every 5 s, the boat beacons every 5 s).
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
if not defined ADMIN_API_TOKEN (
    echo [ERROR] ADMIN_API_TOKEN not set in boat.local.cmd ^(the Worker's secret, see esp32-lora-station\README.md^)
    exit /b 1
)
set "FAILS=0"
if not "%~1"=="" (
    call :switch %~1
    goto :end
)
for %%M in (1 2 3 0) do call :switch %%M
:end
if "%FAILS%"=="0" (echo [LIGHT] OK & exit /b 0)
echo [LIGHT] FAILED: %FAILS% switch^(es^)
exit /b 1

:switch
echo [LIGHT] %BUOY_ID%: lights %1
curl -s -f -A "vedc-test/1.0" -X POST -H "Authorization: Bearer %ADMIN_API_TOKEN%" -H "Content-Type: application/json" ^
     -d "{\"id\":\"%BUOY_ID%\",\"lights\":%1}" "%CLOUD_URL%/api/lights"
if errorlevel 1 (
    echo   the Worker refused or is not reachable: %CLOUD_URL%
    set /a FAILS+=1
    exit /b 0
)
echo.
pushd "%ROOT%STM32F4"
python Tools\lora_monitor.py --wait-lights %1 --timeout 30
if errorlevel 1 set /a FAILS+=1
popd
ping -n 3 127.0.0.1 >nul
exit /b 0

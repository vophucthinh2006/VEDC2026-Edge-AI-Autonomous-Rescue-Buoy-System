@echo off
rem ============================================================
rem  The STM32F407 boat (Pi + F407) in Gazebo, the central-Vietnam scene, driven from the dashboard on
rem  this computer: route from the web -> Worker -> simulated shore station -> LoRa -> F407 -> Pi.
rem    1. the Worker and dashboard (wrangler dev, port 8787) and the test sign-in for the admin
rem       (npm run dev:idp), each in a window of its own, unless they already run; a Worker started
rem       here starts without the data of earlier runs (only PHAO-01..03 on the dashboard)
rem    2. lifts a STOP and the lights an earlier run left on the buoy (the Worker keeps them)
rem    3. Gazebo with its window, the simulated F407, LD14 and shore station, main.py with the viewer
rem       (sim/tools/stm32_web_sim.sh in WSL); prints the dashboard's and the viewer's links.
rem       Three buoys, all on Ho Da 01: PHAO-01 the simulated boat, PHAO-02 and PHAO-03 standing
rem       still and only beaconing (sim/tools/beacon_buoy_sim.py)
rem
rem  Usage: sim_central [headless] [world]     world: central (default), lake, flood, village, two
rem  Needs WSL Ubuntu-24.04 with the simulator (sim/README.md), Node.js with "npm install" done in
rem  esp32-lora-station\cloud, and its .dev.vars (esp32-lora-station\README.md).
rem  Ctrl+C stops the simulator; close the wrangler and sign-in windows to stop those.
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
set "CLOUD=%ROOT%esp32-lora-station\cloud"
set "HUB_STATE=%ROOT%esp32-lora-station\cloud\.wrangler\state\v3\do\mliot-redgescue-Hub"
set "WEB=http://127.0.0.1:8787"
set "VIEWER=http://127.0.0.1:8090"
set "SIM_ARGS=" & set "WORLD=central"
for %%A in (%*) do (
    if /I "%%~A"=="headless" (set "SIM_ARGS=--headless") else set "WORLD=%%~A"
)
if not exist "%CLOUD%\.dev.vars" (
    echo [ERROR] %CLOUD%\.dev.vars missing: see esp32-lora-station\README.md, local Worker
    exit /b 1
)
where npx >nul 2>nul || (echo [ERROR] Node.js not found: npx is needed for wrangler dev & exit /b 1)
set "INGEST_TOKEN=" & set "ADMIN_TOKEN="
for /f "usebackq tokens=1,* delims==" %%A in ("%CLOUD%\.dev.vars") do (
    if "%%A"=="INGEST_TOKEN" set "INGEST_TOKEN=%%B"
    if "%%A"=="ADMIN_API_TOKEN" set "ADMIN_TOKEN=%%B"
)
if not defined INGEST_TOKEN (echo [ERROR] INGEST_TOKEN not set in .dev.vars & exit /b 1)

echo ===== 1. Worker and dashboard
rem A fresh Worker starts clean: the buoys, people, events, route, lights and STOP of earlier runs
rem live in the local Hub's storage, and the dashboard would list every test buoy ever seen.
call :up "%WEB%/api/config"
if errorlevel 1 (
    if exist "%HUB_STATE%" (
        echo   clearing the local Worker's data of earlier runs
        rmdir /s /q "%HUB_STATE%"
    )
    pushd "%CLOUD%"
    start "wrangler dev" cmd /k npx wrangler dev --port 8787 --ip 127.0.0.1
    popd
) else echo   already running: its data of earlier runs stays ^(close its window and run this again to clear it^)
call :up "http://127.0.0.1:8799/jwks.json"
if errorlevel 1 (
    pushd "%CLOUD%"
    start "dev sign-in" cmd /k npm run dev:idp
    popd
)
call :wait "%WEB%/api/config" 90
if errorlevel 1 (echo [ERROR] the Worker did not answer on %WEB%: see its window & exit /b 1)

echo ===== 2. %BUOY_ID%: no STOP, lights off
if defined ADMIN_TOKEN (
    curl -s -o nul -X POST -H "Authorization: Bearer %ADMIN_TOKEN%" -H "Content-Type: application/json" -d "{\"id\":\"%BUOY_ID%\",\"hold\":false}" "%WEB%/api/hold"
    curl -s -o nul -X POST -H "Authorization: Bearer %ADMIN_TOKEN%" -H "Content-Type: application/json" -d "{\"id\":\"%BUOY_ID%\",\"lights\":0}" "%WEB%/api/lights"
) else echo   ADMIN_API_TOKEN not in .dev.vars: a STOP left from an earlier run stays

echo ===== 3. Gazebo (%WORLD%), the F407, the shore station and the Pi
echo.
echo   Relatives : %WEB%/        (the guest page: no sign-in)
echo   Admin     : %WEB%/admin   (the console)
echo   Viewer    : %VIEWER%/   (once the Pi is up, in about a minute)
echo   Sign-in   : on /admin, "Dang nhap admin (thu nghiem)", not Google's button
echo               (the test sign-in; the email must be in ADMIN_EMAILS of .dev.vars)
echo.
wsl -d Ubuntu-24.04 --cd "%ROOT%." -- env VEDC_INGEST_TOKEN=%INGEST_TOKEN% VEDC_CLOUD_URL=%WEB% bash sim/tools/stm32_web_sim.sh --world=%WORLD% %SIM_ARGS%
exit /b %ERRORLEVEL%

:up
powershell -NoProfile -Command "try { Invoke-WebRequest -UseBasicParsing -TimeoutSec 2 '%~1' | Out-Null; exit 0 } catch { exit 1 }"
exit /b %ERRORLEVEL%

:wait
powershell -NoProfile -Command "for ($i = 0; $i -lt %~2; $i++) { try { Invoke-WebRequest -UseBasicParsing -TimeoutSec 2 '%~1' | Out-Null; exit 0 } catch { Start-Sleep -Seconds 1 } }; exit 1"
exit /b %ERRORLEVEL%

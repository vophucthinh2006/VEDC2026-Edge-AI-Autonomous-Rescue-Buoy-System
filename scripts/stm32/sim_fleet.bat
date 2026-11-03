@echo off
rem ============================================================
rem  A fleet of STM32F407 boats (Pi + F407) in Gazebo on one shore station, to drive by hand from the
rem  dashboard on this computer (docs/fleet-control.md, docs/victim-claim.md):
rem    1. the Worker and dashboard (wrangler dev, port 8787) and the test sign-in for the admin
rem       (npm run dev:idp), each in a window of its own, unless they already run; a Worker started
rem       here starts without the data of earlier runs
rem    2. a clean slate for the boats: no formation, no STOP, no RETURN, lights off, no route, no people
rem    3. Gazebo, and for each boat its simulated F407, LD14 and main.py; the simulated shore station on
rem       one shared LoRa channel with time slots (sim/tools/fleet_sim.sh in WSL); prints the links
rem  Then on /admin, tab "Doi tau": draw the area, "Chia vung" or "Doi hinh", preview, send to the fleet.
rem
rem  Usage: sim_fleet [1-4] [world] [headless] [viewer] [noclaims] [holdNN]
rem    1-4       how many boats, PHAO-01 .. PHAO-0n (default 3)
rem    world     wide (default: open water 60 x 36 m, 8 people), central (a flooded street), lake, flood, village
rem    headless  no Gazebo window (a slow computer)
rem    viewer    main.py's viewer for each boat, http://127.0.0.1:8090 + n - 1
rem    noclaims  no victim claims: every boat stays with every person it sees (to compare)
rem    holdNN    seconds a boat stays with a person with no press of the button, e.g. hold60 (default 30;
rem              the real boat: 180)
rem  e.g.  sim_fleet            sim_fleet 3 central viewer            sim_fleet 2 wide noclaims hold20
rem
rem  Needs WSL Ubuntu-24.04 with the simulator (sim/README.md), Node.js with "npm install" done in
rem  esp32-lora-station\cloud, and its .dev.vars (esp32-lora-station\README.md).
rem  Ctrl+C stops the simulator; close the wrangler and sign-in windows to stop those.
rem ============================================================
setlocal EnableExtensions EnableDelayedExpansion
call "%~dp0_env.cmd"
set "CLOUD=%ROOT%esp32-lora-station\cloud"
set "HUB_STATE=%ROOT%esp32-lora-station\cloud\.wrangler\state\v3\do\mliot-redgescue-Hub"
set "WEB=http://127.0.0.1:8787"
set "BOATS=3" & set "WORLD=wide" & set "HOLD=30" & set "SIM_ARGS="
set "BAD=" & set "VIEWERS="
for %%A in (%*) do call :arg "%%~A"
if defined BAD exit /b 1
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
call :up "%WEB%/api/config"
if errorlevel 1 (
    if exist "%HUB_STATE%" (
        echo   clearing the local Worker's data of earlier runs
        rmdir /s /q "%HUB_STATE%"
    )
    pushd "%CLOUD%"
    start "wrangler dev" cmd /k npx wrangler dev --port 8787 --ip 127.0.0.1
    popd
) else echo   already running: its data of earlier runs stays, step 2 clears what gets in the way
call :up "http://127.0.0.1:8799/jwks.json"
if errorlevel 1 (
    pushd "%CLOUD%"
    start "dev sign-in" cmd /k npm run dev:idp
    popd
)
call :wait "%WEB%/api/config" 90
if errorlevel 1 (echo [ERROR] the Worker did not answer on %WEB%: see its window & exit /b 1)

echo ===== 2. A clean slate for PHAO-01 .. PHAO-0%BOATS%
rem As the admin would (Bearer ADMIN_API_TOKEN); the JSON built by PowerShell, cmd would split it at its commas
if defined ADMIN_TOKEN (
    powershell -NoProfile -Command "$h = @{ Authorization = 'Bearer %ADMIN_TOKEN%' }; function P($p, $b) { try { Invoke-RestMethod -Method Post -Uri ('%WEB%' + $p) -Headers $h -ContentType 'application/json' -Body (ConvertTo-Json $b -Compress) | Out-Null } catch { Write-Host ('  ' + $p + ': ' + $_.Exception.Message) } }; $ids = @(1..%BOATS% | ForEach-Object { 'PHAO-0' + $_ }); foreach ($i in $ids) { P '/api/hold' @{ id = $i; hold = $false }; P '/api/lights' @{ id = $i; lights = 0 }; P '/api/mission/send' @{ id = $i; clear = $true } }; P '/api/formation/end' @{}; P '/api/fleet' @{ action = 'resume'; ids = $ids }; P '/api/victims/clear' @{}"
) else echo   ADMIN_API_TOKEN not in .dev.vars: what an earlier run left (STOP, routes, people) stays

echo ===== 3. Gazebo (%WORLD%), %BOATS% boats, the shore station
echo.
echo   Admin     : %WEB%/admin   sign in with "Dang nhap admin (thu nghiem)", not Google's button
echo               (the email must be in ADMIN_EMAILS of .dev.vars)
echo   Relatives : %WEB%/
set /a "VIEW_LAST=8089 + BOATS"
if defined VIEWERS echo   Viewers   : http://127.0.0.1:8090 .. %VIEW_LAST% for PHAO-01 .. PHAO-0%BOATS%
echo.
echo   The boats show on the dashboard in a minute or two ("fleet up" below). Then:
echo     - tab "Doi tau": draw the area, pick "Chia vung" or "Doi hinh", "Xem truoc", "Gui cho ca doi"
echo     - "Nguoi can cuu": the boat that looks after each person; "+1": another boat saw them and went on
echo     - click a person: Da cuu / Nha / Giao; "VE BO TAT CA", "DUNG TAT CA", a boat's "Ve bo"
echo     - logs in WSL ~/vedc_sim/fleet_run: station_sim.log ("claims:"), boat_n/pi.log, boat_n/stm32_sim.log
echo.
wsl -d Ubuntu-24.04 --cd "%ROOT%." -- env VEDC_INGEST_TOKEN=%INGEST_TOKEN% VEDC_CLOUD_URL=%WEB% bash sim/tools/fleet_sim.sh --boats=%BOATS% --world=%WORLD% --hold=%HOLD% %SIM_ARGS%
exit /b %ERRORLEVEL%

rem One argument: a boat count, a world, or a flag
:arg
set "A=%~1"
if "%A%"=="1" (set "BOATS=1" & exit /b 0)
if "%A%"=="2" (set "BOATS=2" & exit /b 0)
if "%A%"=="3" (set "BOATS=3" & exit /b 0)
if "%A%"=="4" (set "BOATS=4" & exit /b 0)
if /I "%A%"=="headless" (set "SIM_ARGS=%SIM_ARGS% --headless" & exit /b 0)
if /I "%A%"=="viewer" (set "SIM_ARGS=%SIM_ARGS% --viewer" & set "VIEWERS=1" & exit /b 0)
if /I "%A%"=="noclaims" (set "SIM_ARGS=%SIM_ARGS% --no-claims" & exit /b 0)
if /I "%A:~0,4%"=="hold" (
    set "HOLD=%A:~4%"
    echo !HOLD!| findstr /r "^[0-9][0-9]*$" >nul || (echo [ERROR] %A%: hold and a number of seconds, e.g. hold60 & set "BAD=1")
    exit /b 0
)
for %%W in (wide central lake flood village) do if /I "%A%"=="%%W" (set "WORLD=%%W" & exit /b 0)
echo [ERROR] %A%: not a boat count 1-4, a world (wide central lake flood village), headless, viewer, noclaims or holdNN
set "BAD=1"
exit /b 0

:up
powershell -NoProfile -Command "try { Invoke-WebRequest -UseBasicParsing -TimeoutSec 2 '%~1' | Out-Null; exit 0 } catch { exit 1 }"
exit /b %ERRORLEVEL%

:wait
powershell -NoProfile -Command "for ($i = 0; $i -lt %~2; $i++) { try { Invoke-WebRequest -UseBasicParsing -TimeoutSec 2 '%~1' | Out-Null; exit 0 } catch { Start-Sleep -Seconds 1 } }; exit 1"
exit /b %ERRORLEVEL%

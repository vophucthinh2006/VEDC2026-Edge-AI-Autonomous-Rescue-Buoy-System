@echo off
rem ============================================================
rem  The whole STM32F407 boat system, in order:
rem    1. every test of this computer (test_host.bat); stops on a failure
rem    2. builds and flashes the STM32F407 firmware (Debug)
rem    3. builds and flashes the shore station (ESP-IDF), its log in a window of its own
rem    4. copies Rasp_Pi to the Pi (boat stm32 deploy)
rem    5. (re)starts main.py on the Pi, without the viewer (it costs the model fps), as the systemd
rem       service vedc-boat, which starts again by itself after a reboot; its output in a window of
rem       its own
rem    6. opens the dashboard
rem
rem  Usage: run_all [noflash] [nostation] [notest] [sim] [stand]
rem    noflash   : the F407 keeps its firmware (steps 2 skipped)
rem    nostation : the station keeps its firmware and runs on its own (step 3 skipped)
rem    notest    : step 1 skipped
rem    sim       : no hardware: the Worker on this computer and the whole mission in the simulator
rem                (test_cloud.bat, then test_sim.bat) instead of steps 2 to 6
rem    stand     : the boat on its stand in a small room for the judges (demo_stand.bat): the same
rem                chain, but main.py with config\stand_demo.yaml (the LiDAR is drawn, avoids
rem                nothing: walls and people close round the stand never stop the motors). Kept at
rem                every boot until a run_all without it. NEVER ON THE WATER.
rem  No viewer either way: every camera frame goes to the detector. To see the camera, run
rem  test_camera.bat (main.py by hand with the viewer) instead.
rem
rem  Then: draw a route on the dashboard and press "Gui toi phao": the boat runs it. An empty
rem  route stops it. The lights are the two switches of the boat's card (admin).
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
set "NOFLASH=" & set "NOSTATION=" & set "NOTEST=" & set "SIM=" & set "STAND="
for %%A in (%*) do (
    if /I "%%~A"=="noflash" set "NOFLASH=1"
    if /I "%%~A"=="nostation" set "NOSTATION=1"
    if /I "%%~A"=="notest" set "NOTEST=1"
    if /I "%%~A"=="sim" set "SIM=1"
    if /I "%%~A"=="stand" set "STAND=1"
)
rem What the service gives run.sh (Rasp_Pi\tools\install_service.sh; none: no viewer)
set "SVC_ARGS="
if defined STAND (
    set "SVC_ARGS=--overlay config/stand_demo.yaml"
    echo.
    echo   STAND DEMO: obstacle avoidance OFF. Never on the water.
    echo   Hands off the propellers once a route is sent; the front ESC is water-cooled.
    echo.
)

if not defined NOTEST (
    echo ===== 1. tests on this computer
    call "%~dp0test_host.bat" || goto :fail
)

if defined SIM (
    echo ===== simulator: the Worker, then the mission
    call "%~dp0test_cloud.bat" mock || goto :fail
    call "%~dp0test_sim.bat" || goto :fail
    goto :ok
)

call "%~dp0_env.cmd" pi || exit /b 1
if not defined NOFLASH (
    echo ===== 2. STM32F407 firmware
    call "%ROOT%STM32F4\build.bat" Debug || goto :fail
    call "%ROOT%STM32F4\flash.bat" Debug || goto :fail
)
if not defined NOSTATION (
    echo ===== 3. shore station
    rem The log window of an earlier run holds the station's COM port: stop its monitor first.
    rem By command line, not window title (in Windows Terminal the title is not the cmd's);
    rem [m]: the pattern must not match this very PowerShell.
    powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'esp_idf_[m]onitor|idf\.py.* [m]onitor' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"
    ping -n 3 127.0.0.1 >nul
    call "%~dp0test_station.bat" build flash || goto :fail
    start "shore station" cmd /k ""%~dp0test_station.bat" monitor"
)
echo ===== 4. Pi code
call "%ROOT%boat.bat" stm32 deploy || goto :fail
echo ===== 5. main.py on the Pi
rem main.py runs as the systemd service vedc-boat (Rasp_Pi\tools\install_service.sh): installed the
rem first time, restarted here on the code just copied, started again by itself at every boot and
rem 3 s after it stops. One main.py only: the script also stops one started by hand. The window
rem follows the service's output; closing it leaves main.py running. End the ssh of an earlier
rem run's window first ([s]/[v]: the patterns must not match the shell that runs them).
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'ssh.*(run\.sh [s]tm32|journalctl -u [v]edc-boat)' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"
ssh %PI_HOST% "cd %PI_DIR% && bash tools/install_service.sh --restart %SVC_ARGS%" || goto :fail
start "Pi main.py" cmd /k ssh -t %PI_HOST% "journalctl -u vedc-boat -f -n 40 -o cat"
ping -n 10 127.0.0.1 >nul
echo ===== 6. dashboard
start "" "%CLOUD_URL%/"

:ok
echo.
echo What to see:
echo   dashboard: PHAO-01 online, its battery, the bow (IMU) and course (GPS) arrows
echo   station window: [ACK] after every beacon
echo   Pi window: "waiting for a route from the dashboard"; the boat stands still (IDLE)
echo   send a route: the boat runs it, the camera sweeps +-45 deg, the speaker every 100 s
echo   a person in view: a coloured dot on the map with its confidence
echo   press PE4: "Da xac nhan lay do" on the speaker, a green alert on the dashboard
exit /b 0

:fail
echo [RUN_ALL] FAILED
exit /b 1

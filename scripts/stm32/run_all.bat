@echo off
rem ============================================================
rem  The whole STM32F407 boat system, in order:
rem    1. every test of this computer (test_host.bat); stops on a failure
rem    2. builds and flashes the STM32F407 firmware (Debug)
rem    3. builds and flashes the shore station (ESP-IDF), its log in a window of its own
rem    4. copies Rasp_Pi to the Pi (boat stm32 deploy)
rem    5. starts main.py on the Pi with the viewer, in a window of its own
rem    6. opens the dashboard and the viewer
rem
rem  Usage: run_all [noflash] [nostation] [notest] [sim]
rem    noflash   : the F407 keeps its firmware (steps 2 skipped)
rem    nostation : the station keeps its firmware and runs on its own (step 3 skipped)
rem    notest    : step 1 skipped
rem    sim       : no hardware: the Worker on this computer and the whole mission in the simulator
rem                (test_cloud.bat, then test_sim.bat) instead of steps 2 to 6
rem
rem  Then: draw a route on the dashboard and press "Gui toi phao": the boat runs it. An empty
rem  route stops it. The lights are the two switches of the boat's card (admin).
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
set "NOFLASH=" & set "NOSTATION=" & set "NOTEST=" & set "SIM="
for %%A in (%*) do (
    if /I "%%~A"=="noflash" set "NOFLASH=1"
    if /I "%%~A"=="nostation" set "NOSTATION=1"
    if /I "%%~A"=="notest" set "NOTEST=1"
    if /I "%%~A"=="sim" set "SIM=1"
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
rem One main.py only: two would fight over the F407's UART. End the ssh of an earlier run's window
rem and stop any main.py still running on the Pi ([m]/[s]: the patterns must not match the shell
rem that runs them).
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'ssh.*run\.sh [s]tm32' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"
ssh %PI_HOST% "pkill -INT -f '[m]ain.py --overlay'; sleep 2" >nul 2>&1
start "Pi main.py" cmd /k ssh -t %PI_HOST% "cd %PI_DIR% && ./run.sh stm32 --viewer"
ping -n 10 127.0.0.1 >nul
echo ===== 6. dashboard and viewer
start "" "%CLOUD_URL%/"
for /f "tokens=2 delims=@" %%H in ("x@%PI_HOST%") do start "" "http://%%H:8090/"

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

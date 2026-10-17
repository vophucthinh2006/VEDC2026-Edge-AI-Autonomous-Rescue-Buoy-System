@echo off
rem ============================================================
rem  One name picks everything of one of the two boats.
rem
rem  Usage: boat <pixhawk|stm32> <build|flash|deploy|run|test> [options]
rem
rem    pixhawk : Raspberry Pi + Pixhawk + STM32F103   (firmware in STM32F103\)
rem    stm32   : Raspberry Pi + STM32F407             (firmware in STM32F4\)
rem
rem    build   : that firmware's own build.bat   [Debug|Release] [clean]
rem    flash   : that firmware's own flash.bat   [Debug|Release]
rem    deploy  : copy Rasp_Pi\ to the Pi over SSH and make this boat the
rem              one its run.sh starts
rem    run     : start the Pi program of this boat over SSH; options go to
rem              run.sh, e.g.  boat pixhawk run --viewer
rem    test    : the Pi unit tests and that firmware's host tests,
rem              plus for "pixhawk test sim" the rescue scenario in the
rem              simulator (WSL, a few minutes)
rem
rem  The Pi's address comes from boat.local.cmd (copy boat.local.example.cmd).
rem ============================================================
setlocal EnableExtensions

set "ROOT=%~dp0"
set "BOAT=%~1"
set "ACTION=%~2"

if /I "%BOAT%"=="pixhawk" (set "FIRMWARE=STM32F103") else if /I "%BOAT%"=="stm32" (set "FIRMWARE=STM32F4") else goto :usage
if "%ACTION%"=="" goto :usage

rem Everything after the first two words goes on untouched.
set "REST="
:collect
if "%~3"=="" goto :collected
set "REST=%REST% %3"
shift /3
goto :collect
:collected

if /I "%ACTION%"=="build"  goto :build
if /I "%ACTION%"=="flash"  goto :flash
if /I "%ACTION%"=="deploy" goto :deploy
if /I "%ACTION%"=="run"    goto :run
if /I "%ACTION%"=="test"   goto :test
goto :usage

:build
call "%ROOT%%FIRMWARE%\build.bat"%REST%
exit /b %ERRORLEVEL%

:flash
call "%ROOT%%FIRMWARE%\flash.bat"%REST%
exit /b %ERRORLEVEL%

:deploy
call :pi_address || exit /b 1
echo [DEPLOY] Rasp_Pi -^> %PI_HOST%:%PI_DIR%  (boat: %BOAT%)
rem tar over ssh: one connection, and the Pi's .venv, logs and caches stay out of it.
tar -C "%ROOT%Rasp_Pi" --exclude=.venv --exclude=__pycache__ --exclude=.pytest_cache --exclude=logs --exclude=lidar_capture.bin -cf - . | ssh %PI_HOST% "mkdir -p %PI_DIR% && tar -C %PI_DIR% -xf - && echo %BOAT% > %PI_DIR%/.boat_profile && chmod +x %PI_DIR%/run.sh"
if errorlevel 1 goto :fail
rem The detector's models: camera.model_path points at ../YOLO_Movenet/weights, next to PI_DIR.
rem Only the TFLite files; the .onnx and .pt are for the PC.
echo [DEPLOY] YOLO_Movenet\weights\*.tflite -^> %PI_HOST%:%PI_DIR%/../YOLO_Movenet/weights
tar -C "%ROOT%YOLO_Movenet" --exclude=*.onnx --exclude=*.pt -cf - weights | ssh %PI_HOST% "mkdir -p %PI_DIR%/../YOLO_Movenet && tar -C %PI_DIR%/../YOLO_Movenet -xf -"
if errorlevel 1 goto :fail
echo [DEPLOY] OK: on the Pi, ./run.sh now starts the %BOAT% boat
exit /b 0

:run
call :pi_address || exit /b 1
ssh -t %PI_HOST% "cd %PI_DIR% && ./run.sh %BOAT%%REST%"
exit /b %ERRORLEVEL%

:test
echo [TEST] Pi unit tests
pushd "%ROOT%Rasp_Pi"
python -m unittest discover -s tests
set "RC=%ERRORLEVEL%"
popd
if not "%RC%"=="0" goto :fail
echo [TEST] %FIRMWARE% host tests
powershell -NoProfile -ExecutionPolicy Bypass -File "%ROOT%%FIRMWARE%\Tools\run_host_tests.ps1"
if errorlevel 1 goto :fail
if /I "%BOAT%"=="pixhawk" if /I "%REST%"==" sim" (
    echo [TEST] Rescue scenario in the simulator
    wsl -d Ubuntu-24.04 --cd "%ROOT%." -- bash sim/tools/rescue_test.sh simulated
    if errorlevel 1 goto :fail
)
echo [TEST] OK
exit /b 0

:pi_address
if exist "%ROOT%boat.local.cmd" call "%ROOT%boat.local.cmd"
if not defined PI_HOST (
    echo [ERROR] PI_HOST is not set. Copy boat.local.example.cmd to boat.local.cmd and fill it in.
    exit /b 1
)
if not defined PI_DIR set "PI_DIR=vedc/Rasp_Pi"
exit /b 0

:usage
echo Usage: boat ^<pixhawk^|stm32^> ^<build^|flash^|deploy^|run^|test^> [options]
echo   pixhawk = Raspberry Pi + Pixhawk + STM32F103      stm32 = Raspberry Pi + STM32F407
echo   boat pixhawk build Release      boat stm32 flash       boat pixhawk deploy
echo   boat pixhawk run --viewer       boat stm32 test        boat pixhawk test sim
exit /b 2

:fail
echo.
echo [%ACTION%] FAILED
exit /b 1

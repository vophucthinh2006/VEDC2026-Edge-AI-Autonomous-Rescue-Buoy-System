@echo off
rem ============================================================
rem  Camera, detector and camera servo: main.py on the Pi without the LiDAR, with the viewer.
rem  The F407 must be on its UART and armed (BNO055 read, not tilted): the Pi then sweeps the
rem  camera +-45 deg looking for people; somebody in front of it: the servo follows them and the
rem  viewer draws the box, the skeleton and the risk level. No route: the motors stay still.
rem
rem  Usage: test_camera
rem  Viewer: http://<PI_HOST>:8090/    Ctrl+C stops it.
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd" pi || exit /b 1
echo [CAMERA] viewer: http://%PI_HOST:*@=%:8090/
ssh -t %PI_HOST% "cd %PI_DIR% && ./run.sh stm32 --no-lidar --viewer"
exit /b %ERRORLEVEL%

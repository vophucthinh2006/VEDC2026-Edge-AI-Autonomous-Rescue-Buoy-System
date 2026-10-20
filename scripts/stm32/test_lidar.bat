@echo off
rem ============================================================
rem  LD14 LiDAR on the Pi (serial.lidar_port of config\boat_stm32.yaml): returns per degree near
rem  the boat (Rasp_Pi\tools\lidar_near.py). Stop main.py first.
rem
rem  Usage: test_lidar [PORT SECONDS MAX_M]     default: the configured port, 6 s, 1 m
rem  What to see: hold a hand 0.5 m in front of the bow: returns at the bow's angle.
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd" pi || exit /b 1
ssh %PI_HOST% "cd %PI_DIR% && pkill -f '[m]ain.py'; %PI_PY% $PY tools/lidar_near.py %*"
exit /b %ERRORLEVEL%

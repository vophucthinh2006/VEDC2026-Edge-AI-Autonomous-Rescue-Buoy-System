@echo off
rem ============================================================
rem  The whole mission in the simulator (WSL Ubuntu-24.04, Gazebo), no hardware:
rem  sim\tools\stm32_mission_test.sh - stand still without a route, route over LoRa, the person
rem  reported with a risk level, confirm button, light 1 from the "dashboard", station switched
rem  off: home after 60 s, station back. About 5 minutes.
rem
rem  Usage: test_sim [gui]
rem    gui : open the Gazebo window too
rem  Viewer while it runs: http://127.0.0.1:8090/
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
set "GUI_ENV="
if /I "%~1"=="gui" set "GUI_ENV=GUI=1"
wsl -d Ubuntu-24.04 --cd "%ROOT%." -- env %GUI_ENV% bash sim/tools/stm32_mission_test.sh
exit /b %ERRORLEVEL%

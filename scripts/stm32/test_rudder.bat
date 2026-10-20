@echo off
rem ============================================================
rem  The three rudder servos of the motor pods (front, rear right, rear left) on the stand,
rem  steered back and forth the way the heading controller does: right to PERCENT of full steer,
rem  through straight ahead, left, PERIOD seconds a cycle (4 by default), for SECONDS
rem  (STM32F4\Tools\rudder_test.ps1, HwTest firmware: no motor turns).
rem  Usage: test_rudder [PERCENT SECONDS [PERIOD]] [-Keep]     e.g. test_rudder 100 10
rem  The normal firmware is flashed back afterwards; -Keep leaves HwTest for another run.
rem ============================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\..\STM32F4\Tools\rudder_test.ps1" %*
exit /b %ERRORLEVEL%

@echo off
rem ============================================================
rem  Dump BNO055 calibration profile from the running target
rem  and regenerate Modules\BNO055\bno055_calib_profile.h
rem
rem  Usage: Tools\bno055_dump_calib.bat [Debug|Release]
rem  Close CubeMonitor / debugger first (ST-LINK must be free).
rem ============================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0bno055_dump_calib.ps1" %*
exit /b %errorlevel%

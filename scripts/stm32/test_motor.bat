@echo off
rem ============================================================
rem  The three motors on the stand, HwTest firmware (STM32F4\Tools\motor_test.ps1).
rem  Usage: test_motor [FRONT_US RIGHT_US LEFT_US SECONDS] [-SkipFlash]
rem    test_motor                     asks for each value
rem    test_motor 1700 1700 1700 5    all three at 1700 us for 5 s
rem    front : 1500 us stop, 1000..2000 (below pushes forward, above back)
rem    rear  : 1500 us stop and arm (like the front's neutral), throttle 1550..2000
rem    after a flash it waits 6 s: the front ESC arms 2-3 s after the rear ones
rem    time  : at most 60 s; the firmware stops the motors when it runs out
rem  Back to the normal firmware afterwards: STM32F4\flash.bat (or run_all.bat).
rem ============================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\..\STM32F4\Tools\motor_test.ps1" %*
exit /b %ERRORLEVEL%

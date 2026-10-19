@echo off
rem ============================================================
rem  Bench test of the three motors of the STM32F407 boat, on its stand.
rem
rem  Usage: motor_test [FRONT_US RIGHT_US LEFT_US SECONDS] [-SkipFlash]
rem    motor_test                     asks for each value
rem    motor_test 1700 1700 1700 5    all three at 1700 us for 5 s
rem    motor_test 1500 1600 1000 3 -SkipFlash
rem
rem    front : 1500 us stop, 1200..1800 (below pushes forward, above back)
rem    rear  : 1000 us stop, 1000..1800 (they turn from about 1500 us)
rem    time  : at most 60 s; the firmware stops the motors when it runs out
rem
rem  Flashes the HwTest firmware (the Pi is not listened to) unless -SkipFlash.
rem  Back to the normal program: run_normal.bat
rem  Details: STM32F4\Tools\motor_test.ps1
rem ============================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0STM32F4\Tools\motor_test.ps1" %*
exit /b %ERRORLEVEL%

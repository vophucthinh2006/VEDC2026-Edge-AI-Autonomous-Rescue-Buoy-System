@echo off
rem ============================================================
rem  The three motors on the stand: the repository's motor_test.bat (HwTest firmware).
rem  Usage: test_motor [FRONT_US RIGHT_US LEFT_US SECONDS] [-SkipFlash]
rem  Back to the normal firmware afterwards: STM32F4\flash.bat (or run_all.bat).
rem ============================================================
call "%~dp0..\..\motor_test.bat" %*
exit /b %ERRORLEVEL%

@echo off
rem ============================================================
rem  The STM32 boat on its stand in a small room for the judges: run_all.bat stand.
rem  The whole chain as run_all.bat (tests, the F407 and the station flashed, the Pi code, main.py as
rem  the service vedc-boat, the dashboard; the boat waits for a route sent from the dashboard), with
rem  config\stand_demo.yaml: the obstacle avoidance is off, so the walls and the people close round
rem  the stand never stop the motors, and no viewer, so the Pi's frames all go to the detector.
rem  The service keeps it at every boot until a run_all.bat without stand. NEVER ON THE WATER.
rem
rem  Usage: demo_stand [noflash] [nostation] [notest]     as run_all.bat
rem  Stop: DUNG PHAO or an empty route on the dashboard.
rem ============================================================
call "%~dp0run_all.bat" stand %*
exit /b %ERRORLEVEL%

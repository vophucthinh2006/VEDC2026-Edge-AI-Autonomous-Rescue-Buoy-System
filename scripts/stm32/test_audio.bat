@echo off
rem ============================================================
rem  The Pi's speaker: plays both announcements once over ssh, on AUDIO_DEVICE (boat.local.cmd,
rem  default plughw:0,0, the 3.5 mm jack; it must match audio.device of config\settings.yaml).
rem    sounds/rescue_boat.wav         "Day la tau cuu ho", every 100 s while main.py runs
rem    sounds/supplies_confirmed.wav  after the confirm button (PE4)
rem  Usage: test_audio
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd" pi || exit /b 1
ssh %PI_HOST% "cd %PI_DIR% && for f in sounds/rescue_boat.wav sounds/supplies_confirmed.wav; do echo playing $f; aplay -q -D %AUDIO_DEVICE% $f || exit 1; done"
if errorlevel 1 (
    echo [AUDIO] FAILED: aplay -l on the Pi lists the devices
    exit /b 1
)
echo [AUDIO] OK if you heard both
exit /b 0

@echo off
rem ============================================================
rem  The Worker and the dashboard on this computer: npm test, then wrangler dev on port 8787 in
rem  its own window and the dashboard in the browser. ?mock shows made-up boats, people and alerts
rem  without any hardware.
rem
rem  Usage: test_cloud [mock]
rem  Needs Node.js; the first time "npm install" in esp32-lora-station\cloud, and .dev.vars
rem  (esp32-lora-station\README.md).
rem ============================================================
setlocal EnableExtensions
call "%~dp0_env.cmd"
pushd "%ROOT%esp32-lora-station\cloud"
call npm test || (popd & echo [CLOUD] tests FAILED & exit /b 1)
start "wrangler dev" cmd /k npx wrangler dev --port 8787
popd
ping -n 8 127.0.0.1 >nul
if /I "%~1"=="mock" (start "" "http://127.0.0.1:8787/?mock") else (start "" "http://127.0.0.1:8787/")
echo [CLOUD] dashboard on http://127.0.0.1:8787/ (close the wrangler window to stop)
exit /b 0

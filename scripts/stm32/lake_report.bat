@echo off
rem ============================================================
rem  After a lake test (docs\lake-test.md): copies main.py's logs\nav_*.csv from the Pi to
rem  lake_logs\ (main.py writes one per start, every AUTO loop; run_all.bat needs nothing more), then
rem  makes the report of how the boat steered (Rasp_Pi\tools\lake_report.py) and opens it.
rem
rem  Usage: lake_report [all] [nocopy]
rem    (none) : the newest run only (the service restarted by run_all.bat is one run)
rem    all    : every run copied, read as one, oldest first
rem    nocopy : the Pi is out of reach: the CSVs already in lake_logs\
rem
rem  What to see: PASS/FAIL per metric in this window, the page with the track and the charts.
rem  Needs matplotlib on this computer (pip install matplotlib).
rem ============================================================
setlocal EnableExtensions EnableDelayedExpansion
call "%~dp0_env.cmd" || exit /b 1
set "ALL=" & set "NOCOPY="
for %%A in (%*) do (
    if /I "%%~A"=="all" set "ALL=1"
    if /I "%%~A"=="nocopy" set "NOCOPY=1"
)
set "DIR=%ROOT%lake_logs"
if not exist "%DIR%" mkdir "%DIR%"
if not defined NOCOPY (
    call "%~dp0_env.cmd" pi || exit /b 1
    scp "%PI_HOST%:%PI_DIR%/logs/nav_*.csv" "%DIR%\" || (echo [LAKE] copy from the Pi failed & exit /b 1)
)
set "FILES=" & set "NEWEST="
rem nav_YYYYmmdd_HHMMSS.csv: by name is by time
for /f "delims=" %%F in ('dir /b /on "%DIR%\nav_*.csv" 2^>nul') do (
    set "FILES=!FILES! "%DIR%\%%F""
    set "NEWEST=%DIR%\%%F"
)
if not defined NEWEST (echo [LAKE] no nav_*.csv in %DIR% & exit /b 1)
pushd "%ROOT%Rasp_Pi"
if defined ALL (
    set "OUT=%DIR%\report_all.html"
    python tools\lake_report.py !FILES! -o "!OUT!"
) else (
    for %%N in ("%NEWEST%") do set "OUT=%DIR%\%%~nN.html"
    python tools\lake_report.py "%NEWEST%" -o "!OUT!"
)
set "RC=%ERRORLEVEL%"
popd
if "%RC%"=="0" start "" "%OUT%"
exit /b %RC%

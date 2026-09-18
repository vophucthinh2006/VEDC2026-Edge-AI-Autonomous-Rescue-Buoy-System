@echo off
rem ============================================================
rem  Flash STM32F4 firmware via ST-LINK + OpenOCD
rem
rem  Usage: flash.bat [Debug|Release]
rem    Flashes build\<type>\STM32F4.elf (run build.bat first),
rem    verifies, then resets the MCU.
rem
rem  Override OpenOCD executable: set OPENOCD=<path to openocd.exe>
rem ============================================================
setlocal EnableExtensions

set "BUILD_TYPE=Debug"
if /I "%~1"=="Release" set "BUILD_TYPE=Release"

cd /d "%~dp0"

set "ELF=build\%BUILD_TYPE%\STM32F4.elf"
if not exist "%ELF%" goto :elf_missing

if defined OPENOCD goto :openocd_ok
where openocd >nul 2>&1 || goto :openocd_missing
set "OPENOCD=openocd"
:openocd_ok

rem OpenOCD (Tcl) needs forward slashes in file paths
set "ELF_TCL=%ELF:\=/%"

echo [FLASH] %ELF%
"%OPENOCD%" -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program %ELF_TCL% verify reset exit"
if errorlevel 1 goto :fail

echo.
echo [FLASH] OK
exit /b 0

:elf_missing
echo [ERROR] %ELF% not found. Run: build.bat %BUILD_TYPE%
exit /b 1

:openocd_missing
echo [ERROR] openocd not found in PATH. Set OPENOCD=^<path to openocd.exe^>
exit /b 1

:fail
echo.
echo [FLASH] FAILED - check ST-LINK connection, board power and SWD wiring
exit /b 1

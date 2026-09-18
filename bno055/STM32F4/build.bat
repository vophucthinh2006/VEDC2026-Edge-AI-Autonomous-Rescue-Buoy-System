@echo off
rem ============================================================
rem  Build STM32F4 firmware (CMake + Ninja + arm-none-eabi-gcc)
rem
rem  Usage: build.bat [Debug|Release] [clean]
rem    Debug   : -O0 -g3 (default)
rem    Release : -Os
rem    clean   : delete build\<type> before building
rem
rem  Output : build\<type>\STM32F4.elf / .hex / .bin
rem  Override toolchain path: set ARM_GCC_BIN=<path to bin>
rem ============================================================
setlocal EnableExtensions

set "BUILD_TYPE=Debug"
set "CLEAN="
for %%A in (%*) do (
    if /I "%%~A"=="Debug"   set "BUILD_TYPE=Debug"
    if /I "%%~A"=="Release" set "BUILD_TYPE=Release"
    if /I "%%~A"=="clean"   set "CLEAN=1"
)

if not defined ARM_GCC_BIN set "ARM_GCC_BIN=C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin"

cd /d "%~dp0"

rem --- Toolchain check (outside parentheses: PATH contains "(x86)") ---
where arm-none-eabi-gcc >nul 2>&1 && goto :gcc_ok
if not exist "%ARM_GCC_BIN%\arm-none-eabi-gcc.exe" goto :gcc_missing
set "PATH=%ARM_GCC_BIN%;%PATH%"
:gcc_ok

where cmake >nul 2>&1 || goto :cmake_missing
where ninja >nul 2>&1 || goto :ninja_missing

set "OUT_DIR=build\%BUILD_TYPE%"
set "ELF=%OUT_DIR%\STM32F4.elf"

if defined CLEAN if exist "%OUT_DIR%" (
    echo [BUILD] Cleaning %OUT_DIR%
    rmdir /s /q "%OUT_DIR%"
)

echo [BUILD] Configure (%BUILD_TYPE%)
cmake --preset %BUILD_TYPE%
if errorlevel 1 goto :fail

echo [BUILD] Compile (%BUILD_TYPE%)
cmake --build --preset %BUILD_TYPE%
if errorlevel 1 goto :fail

arm-none-eabi-objcopy -O ihex "%ELF%" "%OUT_DIR%\STM32F4.hex"
if errorlevel 1 goto :fail
arm-none-eabi-objcopy -O binary "%ELF%" "%OUT_DIR%\STM32F4.bin"
if errorlevel 1 goto :fail

echo.
arm-none-eabi-size "%ELF%"
echo.
echo [BUILD] OK: %ELF%
exit /b 0

:gcc_missing
echo [ERROR] arm-none-eabi-gcc not found in PATH or ARM_GCC_BIN="%ARM_GCC_BIN%"
exit /b 1

:cmake_missing
echo [ERROR] cmake not found in PATH
exit /b 1

:ninja_missing
echo [ERROR] ninja not found in PATH
exit /b 1

:fail
echo.
echo [BUILD] FAILED
exit /b 1

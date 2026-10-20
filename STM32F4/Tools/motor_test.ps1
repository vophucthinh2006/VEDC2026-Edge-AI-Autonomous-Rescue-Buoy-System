<#
Bench test of the three motors of the STM32F407 boat, on its stand, through the ST-LINK.

Builds and flashes the HwTest firmware (App/hw_test.c: the control loop is off, the Pi is not
listened to), waits for the ESCs to arm on the stop signal, drives the three ESCs with the pulses
given for the time given, then puts them back to stop. The firmware stops them by itself when the
time runs out, even if this script or the ST-LINK is cut off.

    motor_test.ps1 [-Front 1500] [-Right 1500] [-Left 1500] [-Seconds 5] [-SkipFlash]
    motor_test.ps1 1700 1700 1700 5            # positional: front, right, left, seconds
    motor_test.ps1                              # asks for each value

Pulses (App/app_config.h, fixed on the bench 2026-10-20): rear ESCs (one way) 1500 us stop and arm,
throttle 1550 us and up; front ESC (both ways) 1500 us stop, below pushes forward, above pushes back.
Allowed here: rear 1500 (stop) or 1550..2000 us, front 1000..2000 us (full range, the HwTest preset in
CMakePresets.json caps the same), at most 60 s.
After a flash the script waits for the front ESC too: it arms 2-3 s after the rear ones.
-SkipFlash: the HwTest firmware is already on the board (a run of this script left it there).
Back to the normal program: run_normal.bat.
#>
param(
    [Nullable[int]]$Front,
    [Nullable[int]]$Right,
    [Nullable[int]]$Left,
    [Nullable[double]]$Seconds,
    [switch]$SkipFlash
)
$ErrorActionPreference = 'Stop'
$fw = Split-Path -Parent $PSScriptRoot                     # STM32F4
# Rear: ESC_REAR_STOP_US, ESC_REAR_START_US (App/app_config.h), the HwTest cap HW_TEST_ESC_MAX_US
$REAR_STOP, $REAR_START, $REAR_MAX, $FRONT_MIN, $FRONT_MAX, $MAX_S = 1500, 1550, 2000, 1000, 2000, 60
function RearOk([int]$us) { $us -eq $REAR_STOP -or ($us -ge $REAR_START -and $us -le $REAR_MAX) }

function Ask([string]$label, [double]$default) {
    $answer = Read-Host "$label [$default]"
    if ([string]::IsNullOrWhiteSpace($answer)) { return $default }
    return [double]$answer
}
if ($null -eq $Front)   { $Front   = [int](Ask 'Front motor, us (1500 = stop)' 1500) }
if ($null -eq $Right)   { $Right   = [int](Ask 'Rear right motor, us (1500 = stop)' 1500) }
if ($null -eq $Left)    { $Left    = [int](Ask 'Rear left motor, us (1500 = stop)' 1500) }
if ($null -eq $Seconds) { $Seconds = Ask 'Run time, s' 5 }

if ($Front -lt $FRONT_MIN -or $Front -gt $FRONT_MAX) { throw "Front $Front us: allowed $FRONT_MIN..$FRONT_MAX" }
if (-not (RearOk $Right)) { throw "Right $Right us: allowed $REAR_STOP (stop) or $REAR_START..$REAR_MAX" }
if (-not (RearOk $Left)) { throw "Left $Left us: allowed $REAR_STOP (stop) or $REAR_START..$REAR_MAX" }
if ($Seconds -le 0 -or $Seconds -gt $MAX_S) { throw "Run time $Seconds s: allowed 0..$MAX_S" }
$runMs = [int]($Seconds * 1000)

if (-not $SkipFlash) {
    & cmd /c "`"$fw\build.bat`" HwTest"
    if ($LASTEXITCODE -ne 0) { throw 'HwTest build failed' }
    & cmd /c "`"$fw\flash.bat`" HwTest"
    if ($LASTEXITCODE -ne 0) { throw 'HwTest flash failed: check the ST-LINK and the board power' }
}

# Where hw_test (App/hw_test.h) sits in RAM: esc_us at +0x08 (front, right, left), esc_run_ms at +0x14.
$gccBin = if ($env:ARM_GCC_BIN) { $env:ARM_GCC_BIN } else { 'C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin' }
$elf = Join-Path $fw 'build\HwTest\STM32F4.elf'
if (-not (Test-Path $elf)) { throw "$elf not found: run once without -SkipFlash" }
$line = & (Join-Path $gccBin 'arm-none-eabi-nm.exe') $elf | Select-String ' hw_test$'
if (-not $line) { throw "hw_test not in $elf" }
$base = [Convert]::ToUInt32($line.ToString().Split(' ')[0], 16)
function Hex([uint32]$offset) { '0x{0:x8}' -f ($base + $offset) }

$openocd = if ($env:OPENOCD) { $env:OPENOCD } else { 'openocd' }
# After a flash the ESCs arm on the stop signal: the rear ones in about 3 s, the front ZTW Shark
# 2-3 s later (bench 2026-10-20); started sooner, the front motor lags the rear ones
$waitArm = if ($SkipFlash) { 0 } else { 6000 }
$hold = [Math]::Max(0, $runMs - 500)
Write-Host ("[MOTOR] front {0} us, rear right {1} us, rear left {2} us for {3} s" -f $Front, $Right, $Left, $Seconds)
# OpenOCD logs on stderr: Windows PowerShell would turn each line into an error and stop here.
$ErrorActionPreference = 'Continue'
$out = & $openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c 'init' -c "sleep $waitArm" `
    -c "mwh $(Hex 0x08) $Front" -c "mwh $(Hex 0x0A) $Right" -c "mwh $(Hex 0x0C) $Left" -c "mww $(Hex 0x14) $runMs" `
    -c 'sleep 500' -c 'mdw 0x40000034 4' -c "sleep $hold" `
    -c "mwh $(Hex 0x08) 1500" -c "mwh $(Hex 0x0A) 1500" -c "mwh $(Hex 0x0C) 1500" -c "mww $(Hex 0x14) 0" -c 'exit' 2>&1 |
    ForEach-Object { "$_" }
$ErrorActionPreference = 'Stop'
if ($LASTEXITCODE -ne 0) { $out | Select-Object -Last 5 | ForEach-Object { Write-Host $_ }; throw 'OpenOCD failed' }

# TIM2 CCR1..CCR4 while running: CH1 PA5 rear left, CH2 PA1 front, CH4 PA3 rear right.
$ccr = ($out | Select-String '^0x40000034:').ToString().Split(' ', [StringSplitOptions]::RemoveEmptyEntries)
Write-Host ("[MOTOR] pulses on the pins while running: front PA1 {0} us, rear right PA3 {1} us, rear left PA5 {2} us" -f `
    [Convert]::ToInt32($ccr[2], 16), [Convert]::ToInt32($ccr[4], 16), [Convert]::ToInt32($ccr[1], 16))
Write-Host '[MOTOR] stopped. Run again with -SkipFlash, or run_normal.bat for the normal program.'

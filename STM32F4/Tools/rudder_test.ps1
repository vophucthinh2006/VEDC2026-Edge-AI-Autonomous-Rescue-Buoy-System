<#
Bench test of the three rudder servos of the motor pods (front PA7, rear right PB1, rear left PB0;
TIM3 CH2/CH4/CH3), through the ST-LINK, on the HwTest firmware (App/hw_test.c: the control loop is
off, the Pi is not listened to, the ESCs stay silent: no motor turns).
Steers the boat smoothly back and forth the way the heading controller does: from straight ahead
to PERCENT of full steer to the right (bow right), back through straight ahead to the left, again
and again for SECONDS, PERIOD seconds a full cycle. Each servo gets the pulse the normal firmware
would give it (Modules/Actuators/actuators.c write_rudder): its trimmed centre + steer * direction
* RUDDER_SPAN_US, the front pod swinging the other way to the rear ones (four-wheel steering).
The pulses on the three pins are read back 5 times a second.
    rudder_test.ps1 [-Percent 100] [-Seconds 10] [-Period 4] [-Keep]
    rudder_test.ps1 100 10                        # positional: percent, seconds, period
Afterwards the normal Debug firmware is flashed back, ready for run_all.bat; -Keep leaves HwTest
on the board for another run.
#>
param(
    [double]$Percent = 100,
    [double]$Seconds = 10,
    [double]$Period = 4,
    [switch]$Keep
)
$ErrorActionPreference = 'Stop'
$fw = Split-Path -Parent $PSScriptRoot                     # STM32F4
# Must match App/app_config.h: RUDDER_SPAN_US, RUDDER_DIR_FRONT, RUDDER_DIR_REAR, RUDDER_FRONT_GAIN
$SPAN_US, $DIR_FRONT, $DIR_REAR, $FRONT_GAIN, $TICK_MS = 250, -1, 1, 1.0, 50
if ($Percent -le 0 -or $Percent -gt 100) { throw "Percent ${Percent}: allowed 0..100 (of full steer)" }
if ($Seconds -le 0 -or $Seconds -gt 120) { throw "Seconds ${Seconds}: allowed 0..120" }
if ($Period -lt 0.5 -or $Period -gt 30) { throw "Period ${Period}: allowed 0.5..30 s" }

$gccBin = if ($env:ARM_GCC_BIN) { $env:ARM_GCC_BIN } else { 'C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin' }
# The executable itself: a bare 'openocd' would also name a function of that name (PowerShell is case-blind)
$openocd = if ($env:OPENOCD) { $env:OPENOCD } else { (Get-Command openocd -CommandType Application | Select-Object -First 1).Source }
$elf = Join-Path $fw 'build\HwTest\STM32F4.elf'
function Invoke-Ocd([string[]]$commands) {
    $ErrorActionPreference = 'Continue'   # OpenOCD logs on stderr
    $a = @('-f', 'interface/stlink.cfg', '-f', 'target/stm32f4x.cfg', '-c', 'init')
    foreach ($c in $commands) { $a += @('-c', $c) }
    $out = & $openocd @a -c 'exit' 2>&1 | ForEach-Object { "$_" }
    if ($LASTEXITCODE -ne 0) { $out | Select-Object -Last 5 | ForEach-Object { Write-Host $_ }; throw 'OpenOCD failed: check the ST-LINK and the board power' }
    return $out
}
function Words([string[]]$lines, [string]$prefix) {
    @($lines | Where-Object { $_ -match "^$prefix" } | ForEach-Object { $_.Split(' ', [StringSplitOptions]::RemoveEmptyEntries) | Select-Object -Skip 1 })
}

# HwTest on the board? Build it (incremental), then compare its vector table with the flash, read
# without halting; flash it when they differ.
& cmd /c "`"$fw\build.bat`" HwTest" | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'HwTest build failed' }
$vec = Join-Path $env:TEMP 'rudder_test_vectors.bin'
& (Join-Path $gccBin 'arm-none-eabi-objcopy.exe') -O binary -j .isr_vector $elf $vec
$bytes = [IO.File]::ReadAllBytes($vec)
$want = @(0..15 | ForEach-Object { '{0:x8}' -f [BitConverter]::ToUInt32($bytes, $_ * 4) })
$seen = Words (Invoke-Ocd @('mdw 0x08000000 16')) '0x080000[0-3][0-9a-f]:'   # 8 words a line
if (($seen -join ',') -ne ($want -join ',')) {
    Write-Host '[RUDDER] flashing the HwTest firmware (no motor turns with it)'
    & cmd /c "`"$fw\flash.bat`" HwTest" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'HwTest flash failed: check the ST-LINK and the board power' }
    Start-Sleep -Seconds 2
}

# hw_test in RAM and the trimmed centres, from the ELF (gdb warns on stderr about the code page)
$ErrorActionPreference = 'Continue'
$gdb = & (Join-Path $gccBin 'arm-none-eabi-gdb.exe') -batch -ex 'print/x &hw_test' -ex 'print actuator_servo_center' $elf 2>&1 | ForEach-Object { "$_" }
$ErrorActionPreference = 'Stop'
$base = [Convert]::ToUInt32((($gdb | Select-String '= 0x([0-9a-f]+)').Matches[0].Groups[1].Value), 16)
$centres = @(($gdb | Select-String '= \{([0-9, ]+)\}').Matches[0].Groups[1].Value.Split(',') | ForEach-Object { [int]$_.Trim() })
if ($centres.Count -ne 4) { throw "actuator_servo_center not found in $elf" }
function Hex([uint32]$offset) { '0x{0:x8}' -f ($base + $offset) }
# servo_us[]: 0 camera, 1 front, 2 right, 3 left (servo_id_t); servo_run_ms at +0x10
$SERVOS = @(@{ name = 'front'; idx = 1; dir = $DIR_FRONT * $FRONT_GAIN }, @{ name = 'right'; idx = 2; dir = $DIR_REAR }, @{ name = 'left '; idx = 3; dir = $DIR_REAR })
function PulseFor([hashtable]$sv, [double]$steer) { [int][Math]::Round($centres[$sv.idx] + $steer * $sv.dir * $SPAN_US) }

# Triangle of steer: 0 -> +p -> 0 -> -p -> 0 ..., right first, a full cycle every $Period seconds
$p = $Percent / 100.0
$n = [int][Math]::Ceiling($Seconds * 1000 / $TICK_MS)
$tcl = New-Object System.Collections.Generic.List[string]
$tcl.Add('set t0 [ms]')
for ($k = 1; $k -le $n; $k++) {
    $phase = (($k * $TICK_MS / 1000.0) / $Period) % 1.0      # 0..1 along one cycle
    $steer = if ($phase -lt 0.25) { 4 * $phase } elseif ($phase -lt 0.75) { 2 - 4 * $phase } else { 4 * $phase - 4 }
    $steer *= $p
    $tcl.Add("set w [expr {`$t0 + $($k * $TICK_MS) - [ms]}]; if {`$w > 0} { sleep `$w }")
    foreach ($sv in $SERVOS) { $tcl.Add("mwh $(Hex (2 * $sv.idx)) $(PulseFor $sv $steer)") }
    $tcl.Add("mww $(Hex 0x10) 1000")                            # keeps the servos on servo_us
    if ($k % 4 -eq 0) { $tcl.Add('echo [mdw 0x40000438 3]') }   # TIM3 CCR2 front, CCR3 left, CCR4 right
}
foreach ($sv in $SERVOS) { $tcl.Add("mwh $(Hex (2 * $sv.idx)) $($centres[$sv.idx])") }
$tcl.Add("mww $(Hex 0x10) 0")                                   # back to the trimmed centres
$tcl.Add('sleep 300')
$tcl.Add('echo [mdw 0x40000438 3]')
$tclFile = Join-Path $env:TEMP 'rudder_test.tcl'
[IO.File]::WriteAllLines($tclFile, $tcl)

Write-Host ("[RUDDER] steering +-{0}% of full ({1} us each way), {2} s a cycle, for {3} s (HwTest, no motor)" -f $Percent, [int]($p * $SPAN_US), $Period, $Seconds)
$out = Invoke-Ocd @("source {$($tclFile -replace '\\', '/')}")
$rows = @($out | Where-Object { $_ -match '^0x40000438:' } | ForEach-Object {
    $w = $_.Split(' ', [StringSplitOptions]::RemoveEmptyEntries)
    @{ front = [Convert]::ToInt32($w[1], 16); left = [Convert]::ToInt32($w[2], 16); right = [Convert]::ToInt32($w[3], 16) } })
if ($rows.Count -lt 2) { throw 'no pulse read back from the rudder pins' }
$bad = 0
foreach ($sv in $SERVOS) {
    $key = $sv.name.Trim()
    $vals = @($rows[0..($rows.Count - 2)] | ForEach-Object { $_[$key] })
    $lo = ($vals | Measure-Object -Minimum).Minimum; $hi = ($vals | Measure-Object -Maximum).Maximum
    $wantLo = [Math]::Min((PulseFor $sv $p), (PulseFor $sv (-$p))); $wantHi = [Math]::Max((PulseFor $sv $p), (PulseFor $sv (-$p)))
    $reach = ($hi - $lo) -ge 0.7 * ($wantHi - $wantLo)
    if (-not $reach) { $bad++ }
    Write-Host ("[RUDDER] {0}  centre {1} us, swung {2}..{3} us (commanded {4}..{5}), back at {6} us{7}" -f `
        $sv.name, $centres[$sv.idx], $lo, $hi, $wantLo, $wantHi, $rows[-1][$key], $(if ($reach) { '' } else { '  <- did not swing' }))
}

if (-not $Keep) {
    Write-Host '[RUDDER] flashing the normal firmware back (Debug)'
    & cmd /c "`"$fw\build.bat`" Debug" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'Debug build failed: the board still runs HwTest' }
    & cmd /c "`"$fw\flash.bat`" Debug" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'Debug flash failed: the board still runs HwTest' }
    Write-Host '[RUDDER] normal firmware back on the board'
} else {
    Write-Host '[RUDDER] HwTest left on the board (-Keep): STM32F4\flash.bat or run_all.bat for the normal program'
}
if ($bad) { throw "$bad rudder pin(s) did not swing as commanded" }
Write-Host '[RUDDER] pulses swing as commanded. A servo that did not move: its 5 V, its signal wire, or the servo itself.'

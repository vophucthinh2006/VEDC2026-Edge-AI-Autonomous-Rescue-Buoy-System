<#
Bench test of the camera pan servo (PA6, TIM3_CH1) the way it moves when the boat runs a route: the
Pi's search sweep (Rasp_Pi/modules/perception.py SweepPattern: back and forth between -limit and
+limit at a constant rate, left first, an angle 20 times a second), fed to the NORMAL firmware
through the ST-LINK into the very field the Pi's CAM packet fills (pi_command.cam_pan_deg in
Core/Src/main.c). The F407 glides the servo there itself (App/camera_pan.c), as on the water.
No motor turns: there is no NAV from the Pi. Stop main.py on the Pi first, or it sends its own CAM.
    cam_servo_test.ps1 [-Degrees 45] [-Seconds 10] [-Rate 30]
    cam_servo_test.ps1 45 10                      # positional: degrees, seconds, rate in deg/s
If the board runs another image (the HwTest firmware of test_motor.bat, an older build), the normal
Debug firmware is built and flashed first, so the board is left ready for run_all.bat.
#>
param(
    [double]$Degrees = 45,
    [double]$Seconds = 10,
    [double]$Rate = 30
)
$ErrorActionPreference = 'Stop'
$fw = Split-Path -Parent $PSScriptRoot                     # STM32F4
$LIMIT_DEG, $TICK_MS = 80, 50                              # CAM_PAN_LIMIT_DEG; camera.pan_rate_hz 20
if ($Degrees -le 0 -or $Degrees -gt $LIMIT_DEG) { throw "Degrees ${Degrees}: allowed 0..$LIMIT_DEG" }
if ($Seconds -le 0 -or $Seconds -gt 120) { throw "Seconds ${Seconds}: allowed 0..120" }
if ($Rate -le 0 -or $Rate -gt 180) { throw "Rate ${Rate}: allowed 0..180 deg/s" }

$gccBin = if ($env:ARM_GCC_BIN) { $env:ARM_GCC_BIN } else { 'C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin' }
# The executable itself: a bare 'openocd' would also name the function below (PowerShell is case-blind)
$openocd = if ($env:OPENOCD) { $env:OPENOCD } else { (Get-Command openocd -CommandType Application | Select-Object -First 1).Source }
$elf = Join-Path $fw 'build\Debug\STM32F4.elf'
function Invoke-Ocd([string[]]$commands) {
    $ErrorActionPreference = 'Continue'   # OpenOCD logs on stderr
    $args_ = @('-f', 'interface/stlink.cfg', '-f', 'target/stm32f4x.cfg', '-c', 'init')
    foreach ($c in $commands) { $args_ += @('-c', $c) }
    $out = & $openocd @args_ -c 'exit' 2>&1 | ForEach-Object { "$_" }
    if ($LASTEXITCODE -ne 0) { $out | Select-Object -Last 5 | ForEach-Object { Write-Host $_ }; throw 'OpenOCD failed: check the ST-LINK and the board power' }
    return $out
}
function Words([string[]]$lines, [string]$prefix) {
    @($lines | Where-Object { $_ -match "^$prefix" } | ForEach-Object { $_.Split(' ', [StringSplitOptions]::RemoveEmptyEntries) | Select-Object -Skip 1 })
}

# Is the normal firmware of this ELF on the board? Its vector table (16 words) read without halting.
if (-not (Test-Path $elf)) { & cmd /c "`"$fw\build.bat`" Debug"; if ($LASTEXITCODE -ne 0) { throw 'Debug build failed' } }
$vec = Join-Path $env:TEMP 'cam_servo_test_vectors.bin'
& (Join-Path $gccBin 'arm-none-eabi-objcopy.exe') -O binary -j .isr_vector $elf $vec
$bytes = [IO.File]::ReadAllBytes($vec)
$want = @(0..15 | ForEach-Object { '{0:x8}' -f [BitConverter]::ToUInt32($bytes, $_ * 4) })
$seen = Words (Invoke-Ocd @('mdw 0x08000000 16')) '0x080000[0-3][0-9a-f]:'   # 8 words a line
if (($seen -join ',') -ne ($want -join ',')) {
    Write-Host '[CAM] the board does not run the normal firmware: building and flashing it (Debug)'
    & cmd /c "`"$fw\build.bat`" Debug"; if ($LASTEXITCODE -ne 0) { throw 'Debug build failed' }
    & cmd /c "`"$fw\flash.bat`" Debug"; if ($LASTEXITCODE -ne 0) { throw 'Flash failed: check the ST-LINK and the board power' }
    Start-Sleep -Seconds 2
}

# Where the CAM packet lands in RAM, from the ELF's debug info
# gdb warns on stderr about the console code page; Windows PowerShell would stop on that line
$ErrorActionPreference = 'Continue'
$gdb = & (Join-Path $gccBin 'arm-none-eabi-gdb.exe') -batch -ex 'print/x &pi_command.cam_pan_deg' -ex 'print/x &pi_command.last_cam_ms' $elf 2>&1 | ForEach-Object { "$_" }
$ErrorActionPreference = 'Stop'
$addr = @($gdb | Select-String '= (0x[0-9a-f]+)' | ForEach-Object { $_.Matches[0].Groups[1].Value })
if ($addr.Count -ne 2) { throw "pi_command.cam_pan_deg / last_cam_ms not found in $elf" }
$panAddr, $camMsAddr = $addr

# The Pi's sweep, sample by sample (SweepPattern.frame_done): from 0, left first, bouncing at the ends
$angle, $dir = 0.0, -1.0
$step = $Rate * $TICK_MS / 1000.0
$n = [int][Math]::Ceiling($Seconds * 1000 / $TICK_MS)
$tcl = New-Object System.Collections.Generic.List[string]
$tcl.Add("mww $camMsAddr 1")                                # non-zero: the F407 follows cam_pan_deg
$tcl.Add('set t0 [ms]')
for ($k = 0; $k -lt $n; $k++) {
    $travel = $step
    while ($travel -gt 0) {
        $room = if ($dir -gt 0) { $Degrees - $angle } else { $angle + $Degrees }
        if ($travel -lt $room) { $angle += $dir * $travel; break }
        $angle = $Degrees * $dir; $dir = -$dir; $travel -= $room
    }
    $bits = [BitConverter]::ToUInt32([BitConverter]::GetBytes([single]$angle), 0)
    $tcl.Add("set w [expr {`$t0 + $(($k + 1) * $TICK_MS) - [ms]}]; if {`$w > 0} { sleep `$w }")
    $tcl.Add(('mww {0} 0x{1:x8}' -f $panAddr, $bits))
    if ($k % 4 -eq 3) { $tcl.Add('echo [mdw 0x40000434 1]') }   # TIM3 CCR1: the pulse on PA6, 5 times a second (a sourced script prints only what it echoes)
}
$tcl.Add("mww $panAddr 0x00000000")                         # back ahead, gliding
$tcl.Add('sleep 1500')
$tcl.Add('echo [mdw 0x40000434 1]')
$tcl.Add("mww $camMsAddr 0")                                # as if the Pi had never sent CAM
$tclFile = Join-Path $env:TEMP 'cam_servo_test.tcl'
[IO.File]::WriteAllLines($tclFile, $tcl)

Write-Host ("[CAM] sweeping +-{0} deg at {1} deg/s for {2} s, as on a route (normal firmware, no motor)" -f $Degrees, $Rate, $Seconds)
$out = Invoke-Ocd @("source {$($tclFile -replace '\\', '/')}")
$us = @(Words $out '0x40000434:' | ForEach-Object { [Convert]::ToInt32($_, 16) })
if ($us.Count -lt 2) { throw 'no pulse read back from PA6' }
$sweep = $us[0..($us.Count - 2)]
$min = ($sweep | Measure-Object -Minimum).Minimum; $max = ($sweep | Measure-Object -Maximum).Maximum
# App/app_config.h: 1500 us + CAM_PAN_DIR (-1) * 11 us per degree
$degOf = { param($p) (1500 - $p) / 11.0 }
Write-Host ("[CAM] PA6 swung {0}..{1} us, that is {2:+0.0;-0.0} .. {3:+0.0;-0.0} deg; back ahead at {4} us" -f `
    $min, $max, (& $degOf $max), (& $degOf $min), $us[-1])
if ((& $degOf $min) -lt $Degrees * 0.8 -or (& $degOf $max) -gt -$Degrees * 0.8) {
    Write-Host '[CAM] the pulse did not reach the ends: is main.py running on the Pi (its CAM overrides this)?'
}
Write-Host '[CAM] pulses on PA6 swing as commanded. If the camera did not turn: servo power (5 V), the signal wire on PA6, or the servo.'

<#
  Dump the BNO055 calibration profile from the running target over ST-LINK (OpenOCD)
  and regenerate Modules/BNO055/bno055_calib_profile.h.

  Usage : Tools\bno055_dump_calib.bat [Debug|Release]
  Needs : firmware built from the same ELF is running and CALIB_STAT reached 3/3/3/3.
          No other tool (CubeMonitor, debugger) may hold the ST-LINK.
  Env   : ARM_GCC_BIN = toolchain bin dir, OPENOCD = openocd.exe path (optional)
#>
param([string]$BuildType = 'Debug')

$ErrorActionPreference = 'Stop'
$ProfileSize = 22
$root   = Split-Path -Parent $PSScriptRoot
$elf    = Join-Path $root "build\$BuildType\STM32F4.elf"
$header = Join-Path $root 'Modules\BNO055\bno055_calib_profile.h'

if (-not (Test-Path $elf)) { throw "$elf not found. Run: build.bat $BuildType" }

# --- Tools -------------------------------------------------------------------
$gccBin = if ($env:ARM_GCC_BIN) { $env:ARM_GCC_BIN } else { 'C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin' }
$nmCmd  = Get-Command arm-none-eabi-nm -ErrorAction SilentlyContinue
$nm     = if ($nmCmd) { $nmCmd.Source } else { Join-Path $gccBin 'arm-none-eabi-nm.exe' }
if (-not (Test-Path $nm)) { throw 'arm-none-eabi-nm not found. Set ARM_GCC_BIN.' }

$ocdCmd  = Get-Command openocd -ErrorAction SilentlyContinue
$openocd = if ($env:OPENOCD) { $env:OPENOCD } elseif ($ocdCmd) { $ocdCmd.Source } else { $null }
if (-not $openocd) { throw 'openocd not found in PATH. Set OPENOCD.' }

# --- Symbol addresses from ELF -----------------------------------------------
$symbols = @{}
& $nm $elf | ForEach-Object {
  $p = $_ -split '\s+'
  if ($p.Count -eq 3) { $symbols[$p[2]] = [Convert]::ToUInt32($p[0], 16) }
}
foreach ($s in 'bno055_calib_captured', 'bno055_calib_captured_valid') {
  if (-not $symbols.ContainsKey($s)) { throw "Symbol '$s' not found in $elf" }
}
$addrData  = $symbols['bno055_calib_captured']
$addrValid = $symbols['bno055_calib_captured_valid']

# --- Read target memory (no halt, no reset) ----------------------------------
$ocdArgs = @(
  '-f', 'interface/stlink.cfg', '-f', 'target/stm32f4x.cfg',
  '-c', 'init',
  '-c', ('mdb 0x{0:x8} 1' -f $addrValid),
  '-c', ('mdb 0x{0:x8} {1}' -f $addrData, $ProfileSize),
  '-c', 'shutdown'
)
Write-Host "[CALIB] Reading target memory via OpenOCD ..."
$ErrorActionPreference = 'Continue'
$ocdOut = & $openocd @ocdArgs 2>&1 | ForEach-Object { "$_" }
$ocdExit = $LASTEXITCODE
$ErrorActionPreference = 'Stop'

$validByte = $null
$bytes = @()
foreach ($line in $ocdOut) {
  if ($line -match '^0x([0-9a-fA-F]{8}):\s+((?:[0-9a-fA-F]{2}\s*)+)$') {
    $addr = [Convert]::ToUInt32($Matches[1], 16)
    $vals = @($Matches[2].Trim() -split '\s+' | ForEach-Object { [Convert]::ToByte($_, 16) })
    if ($addr -eq $addrValid) { $validByte = $vals[0] }
    elseif ($addr -eq $addrData) { $bytes += $vals }
  }
}

if (($null -eq $validByte) -or ($bytes.Count -lt $ProfileSize)) {
  $ocdOut | ForEach-Object { Write-Host "  $_" }
  throw "Could not read target memory (openocd exit $ocdExit). Check ST-LINK connection and close CubeMonitor/debugger."
}
if ($validByte -ne 1) {
  throw 'Profile not captured yet: wait until CALIB_STAT = 3/3/3/3 (check CubeMonitor), and make sure the running firmware matches this ELF.'
}
$bytes = $bytes[0..($ProfileSize - 1)]

# --- Generate header ---------------------------------------------------------
function Get-S16([int]$i) {
  $v = [int]$bytes[$i] -bor ([int]$bytes[$i + 1] -shl 8)   # [byte] -shl 8 would overflow to 0
  if ($v -ge 0x8000) { $v - 0x10000 } else { $v }
}
# Data sheet ranges (3.6.4, Tables 3-16, 3-21, 3-24); firmware applies the same check
$checks = @(
  @{ n = 'ACC offset X'; i = 0;  min = -500;  max = 500 },  @{ n = 'ACC offset Y'; i = 2;  min = -500;  max = 500 },
  @{ n = 'ACC offset Z'; i = 4;  min = -500;  max = 500 },  @{ n = 'MAG offset X'; i = 6;  min = -6400; max = 6400 },
  @{ n = 'MAG offset Y'; i = 8;  min = -6400; max = 6400 }, @{ n = 'MAG offset Z'; i = 10; min = -6400; max = 6400 },
  @{ n = 'GYR offset X'; i = 12; min = -2000; max = 2000 }, @{ n = 'GYR offset Y'; i = 14; min = -2000; max = 2000 },
  @{ n = 'GYR offset Z'; i = 16; min = -2000; max = 2000 }, @{ n = 'ACC radius';   i = 18; min = -2048; max = 2048 },
  @{ n = 'MAG radius';   i = 20; min = 144;   max = 1280 }
)
$invalid = @($checks | Where-Object { $v = Get-S16 $_.i; ($v -lt $_.min) -or ($v -gt $_.max) })
if ($invalid.Count -gt 0) {
  $invalid | ForEach-Object { Write-Host ("  {0} = {1} (range {2}..{3})" -f $_.n, (Get-S16 $_.i), $_.min, $_.max) }
  throw ("Captured profile is outside data sheet ranges, header NOT written. Bytes: " + (($bytes | ForEach-Object { '{0:X2}' -f $_ }) -join ' '))
}

$hex = @($bytes | ForEach-Object { '0x{0:X2}' -f $_ })
$rows = for ($i = 0; $i -lt $ProfileSize; $i += 8) {
  $end  = [Math]::Min($i + 7, $ProfileSize - 1)
  $sep  = if ($end -lt $ProfileSize - 1) { ',' } else { ' ' }
  '  ' + ($hex[$i..$end] -join ', ') + $sep + ' \'
}
$date = Get-Date -Format 'yyyy-MM-dd HH:mm'

$text = @"
/**
  ******************************************************************************
  * @file    bno055_calib_profile.h
  * @brief   BNO055 calibration profile (offsets + radius, data sheet 3.11.5),
  *          shared through git so every STM32F4 kit loads the same profile.
  *
  *          The profile belongs to the BNO055 module and its mounting,
  *          not to the STM32 kit. Regenerate it after changing module/mounting:
  *            1. Flash firmware, calibrate until CALIB_STAT = 3/3/3/3.
  *            2. Run Tools\bno055_dump_calib.bat (target keeps running).
  *
  *          Generated by Tools/bno055_dump_calib.ps1 on $date
  *            ACC offset X/Y/Z : $(Get-S16 0) / $(Get-S16 2) / $(Get-S16 4) LSB (1 m/s^2 = 100 LSB)
  *            MAG offset X/Y/Z : $(Get-S16 6) / $(Get-S16 8) / $(Get-S16 10) LSB (1 uT = 16 LSB)
  *            GYR offset X/Y/Z : $(Get-S16 12) / $(Get-S16 14) / $(Get-S16 16) LSB (1 dps = 16 LSB)
  *            ACC radius       : $(Get-S16 18) LSB
  *            MAG radius       : $(Get-S16 20) LSB
  ******************************************************************************
  */

#ifndef BNO055_CALIB_PROFILE_H
#define BNO055_CALIB_PROFILE_H

#define BNO055_CALIB_PROFILE_VALID  1

/* ACC_OFFSET_X_LSB (0x55) .. MAG_RADIUS_MSB (0x6A) */
#define BNO055_CALIB_PROFILE_DATA \
{ \
$($rows -join "`n")
}

#endif /* BNO055_CALIB_PROFILE_H */
"@

[IO.File]::WriteAllText($header, ($text -replace "`r`n", "`n") + "`n", (New-Object Text.UTF8Encoding $false))
Write-Host "[CALIB] Profile: $($hex -join ' ')"
Write-Host "[CALIB] Written $header"
Write-Host '[CALIB] Rebuild and commit Modules/BNO055/bno055_calib_profile.h'

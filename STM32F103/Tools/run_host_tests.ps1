# PC tests of the firmware parts with no HAL in them. Needs gcc (MinGW) in PATH.
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$testDir = Join-Path ([System.IO.Path]::GetTempPath()) 'vedc-f103-tests'
New-Item -ItemType Directory -Force -Path $testDir | Out-Null

$exe = Join-Path $testDir 'helper_test.exe'
# The pin levels in app_config.h are HAL names; the logic under test never reads them.
& gcc -std=c11 -Wall -Wextra -Werror -DGPIO_PIN_SET=1 -DGPIO_PIN_RESET=0 `
  -I (Join-Path $project 'App') `
  -I (Join-Path $project 'Modules/PiLink') `
  -I (Join-Path $project 'Modules/IbusSensor') `
  (Join-Path $project 'Tests/helper_test.c') `
  (Join-Path $project 'App/lora_frame.c') `
  (Join-Path $project 'App/helper_logic.c') `
  (Join-Path $project 'Modules/PiLink/pi_link.c') `
  (Join-Path $project 'Modules/IbusSensor/ibus_sensor.c') `
  -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile helper_test' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'helper_test failed' }

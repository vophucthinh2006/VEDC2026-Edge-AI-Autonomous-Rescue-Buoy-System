$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$testDir = Join-Path ([System.IO.Path]::GetTempPath()) 'vedc-heading-tests'
New-Item -ItemType Directory -Force -Path $testDir | Out-Null

$headingExe = Join-Path $testDir 'heading_controller_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror -Wno-error=cpp `
  -I (Join-Path $repo 'STM32F4/App') `
  (Join-Path $repo 'STM32F4/Tests/heading_controller_test.c') `
  (Join-Path $repo 'STM32F4/App/heading_controller.c') `
  -lm -o $headingExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile heading_controller_test' }
& $headingExe
if ($LASTEXITCODE -ne 0) { throw 'heading_controller_test failed' }

$parserExe = Join-Path $testDir 'lora_payload_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror `
  -I (Join-Path $repo 'esp32-lora-station/main') `
  (Join-Path $repo 'esp32-lora-station/test/lora_payload_test.c') `
  (Join-Path $repo 'esp32-lora-station/main/lora_payload.c') `
  -lm -o $parserExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile lora_payload_test' }
& $parserExe
if ($LASTEXITCODE -ne 0) { throw 'lora_payload_test failed' }

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

$panExe = Join-Path $testDir 'camera_pan_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror -Wno-error=cpp `
  -I (Join-Path $repo 'STM32F4/App') `
  (Join-Path $repo 'STM32F4/Tests/camera_pan_test.c') `
  (Join-Path $repo 'STM32F4/App/camera_pan.c') `
  -lm -o $panExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile camera_pan_test' }
& $panExe
if ($LASTEXITCODE -ne 0) { throw 'camera_pan_test failed' }

$buttonExe = Join-Path $testDir 'confirm_button_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror -Wno-error=cpp `
  -I (Join-Path $repo 'STM32F4/App') `
  (Join-Path $repo 'STM32F4/Tests/confirm_button_test.c') `
  (Join-Path $repo 'STM32F4/App/confirm_button.c') `
  -o $buttonExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile confirm_button_test' }
& $buttonExe
if ($LASTEXITCODE -ne 0) { throw 'confirm_button_test failed' }

$parserExe = Join-Path $testDir 'lora_payload_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror `
  -I (Join-Path $repo 'esp32-lora-station/main') `
  (Join-Path $repo 'esp32-lora-station/test/lora_payload_test.c') `
  (Join-Path $repo 'esp32-lora-station/main/lora_payload.c') `
  -lm -o $parserExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile lora_payload_test' }
& $parserExe
if ($LASTEXITCODE -ne 0) { throw 'lora_payload_test failed' }

$routeExe = Join-Path $testDir 'route_link_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror `
  -I (Join-Path $repo 'STM32F4/App') `
  -I (Join-Path $repo 'esp32-lora-station/main') `
  (Join-Path $repo 'STM32F4/Tests/route_link_test.c') `
  (Join-Path $repo 'STM32F4/App/route_link.c') `
  (Join-Path $repo 'esp32-lora-station/main/route_downlink.c') `
  -lm -o $routeExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile route_link_test' }
& $routeExe
if ($LASTEXITCODE -ne 0) { throw 'route_link_test failed' }

$shoreExe = Join-Path $testDir 'shore_link_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror `
  -I (Join-Path $repo 'STM32F4/App') `
  -I (Join-Path $repo 'esp32-lora-station/main') `
  (Join-Path $repo 'STM32F4/Tests/shore_link_test.c') `
  (Join-Path $repo 'STM32F4/App/shore_link.c') `
  (Join-Path $repo 'esp32-lora-station/main/route_downlink.c') `
  -lm -o $shoreExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile shore_link_test' }
& $shoreExe
if ($LASTEXITCODE -ne 0) { throw 'shore_link_test failed' }

$eventExe = Join-Path $testDir 'event_queue_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror `
  -I (Join-Path $repo 'STM32F4/App') `
  -I (Join-Path $repo 'esp32-lora-station/main') `
  (Join-Path $repo 'STM32F4/Tests/event_queue_test.c') `
  (Join-Path $repo 'STM32F4/App/event_queue.c') `
  (Join-Path $repo 'esp32-lora-station/main/lora_payload.c') `
  -lm -o $eventExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile event_queue_test' }
& $eventExe
if ($LASTEXITCODE -ne 0) { throw 'event_queue_test failed' }

$gpsExe = Join-Path $testDir 'gps_nmea_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror `
  -I (Join-Path $repo 'STM32F4/Modules/GPS') `
  (Join-Path $repo 'STM32F4/Tests/gps_nmea_test.c') `
  (Join-Path $repo 'STM32F4/Modules/GPS/gps_nmea.c') `
  -lm -o $gpsExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile gps_nmea_test' }
& $gpsExe
if ($LASTEXITCODE -ne 0) { throw 'gps_nmea_test failed' }

$frontExe = Join-Path $testDir 'front_motor_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror -Wno-error=cpp `
  -I (Join-Path $repo 'STM32F4/App') `
  (Join-Path $repo 'STM32F4/Tests/front_motor_test.c') `
  (Join-Path $repo 'STM32F4/App/front_motor.c') `
  -lm -o $frontExe
if ($LASTEXITCODE -ne 0) { throw 'Failed to compile front_motor_test' }
& $frontExe
if ($LASTEXITCODE -ne 0) { throw 'front_motor_test failed' }

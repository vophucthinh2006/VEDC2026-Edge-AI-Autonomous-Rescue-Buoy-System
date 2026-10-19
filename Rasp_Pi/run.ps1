# run.ps1
param (
    [Parameter(Mandatory=$true, Position=0)]
    [string]$Mode,

    [ValueFromRemainingArguments($true)]
    [string[]]$ExtraArgs
)

# Kích hoạt venv nếu chưa bật
if (-not $env:VIRTUAL_ENV) {
    if (Test-Path ".\.venv\Scripts\Activate.ps1") {
        Write-Host "Activating .venv..." -ForegroundColor Green
        & .\.venv\Scripts\Activate.ps1
    } else {
        Write-Host "Warning: .venv not found. Running with global Python." -ForegroundColor Yellow
    }
}

# Luôn load overlay win.yaml nếu có
$WinOverlay = @()
if (Test-Path "config\win.yaml") {
    $WinOverlay = @("--overlay", "config\win.yaml")
}

switch ($Mode.ToLower()) {
    "pixhawk" {
        Write-Host "Starting rescue_main.py (Pixhawk Mode)..." -ForegroundColor Cyan
        python rescue_main.py @WinOverlay @ExtraArgs
    }
    "stm32" {
        Write-Host "Starting main.py (STM32 Mode)..." -ForegroundColor Cyan
        python main.py @WinOverlay @ExtraArgs
    }
    default {
        Write-Host "Usage: .\run.ps1 [pixhawk|stm32] [options]" -ForegroundColor Red
        Write-Host "Example: .\run.ps1 stm32 --no-lidar"
        Write-Host "Example: .\run.ps1 pixhawk --viewer"
    }
}
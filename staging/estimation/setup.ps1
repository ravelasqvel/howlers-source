# Setup script for Omnidirectional Robot Research Project
# Unified setup for all subprojects
# Author: Automated setup for Windows PowerShell

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "  Omnidirectional Robot Research - Project Setup" -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host ""

# Check Python version
Write-Host "[1/4] Checking Python installation..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "Python not found"
    }
    Write-Host "✓ Python found: $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "✗ ERROR: Python is not installed or not in PATH" -ForegroundColor Red
    Write-Host ""
    Write-Host "Please install Python 3.8 or higher from https://www.python.org" -ForegroundColor Yellow
    exit 1
}
Write-Host ""

# Create virtual environment if it doesn't exist
Write-Host "[2/4] Checking virtual environment..." -ForegroundColor Yellow
$venvPath = ".venv"

if (-not (Test-Path $venvPath)) {
    Write-Host "Creating virtual environment at E:\Project\.venv..." -ForegroundColor Cyan
    python -m venv $venvPath
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "✗ ERROR: Failed to create virtual environment" -ForegroundColor Red
        exit 1
    }
    Write-Host "✓ Virtual environment created successfully" -ForegroundColor Green
} else {
    Write-Host "✓ Virtual environment already exists" -ForegroundColor Green
}
Write-Host ""

# Activate virtual environment
Write-Host "[3/4] Activating virtual environment..." -ForegroundColor Yellow
$activateScript = Join-Path $venvPath "Scripts\Activate.ps1"

if (Test-Path $activateScript) {
    & $activateScript
    
    if ($env:VIRTUAL_ENV) {
        Write-Host "✓ Virtual environment activated: $env:VIRTUAL_ENV" -ForegroundColor Green
    } else {
        Write-Host "⚠ Warning: Virtual environment activation may have failed" -ForegroundColor Yellow
        Write-Host "  Continuing with installation..." -ForegroundColor Yellow
    }
} else {
    Write-Host "✗ ERROR: Activation script not found" -ForegroundColor Red
    exit 1
}
Write-Host ""

# Install dependencies
Write-Host "[4/4] Installing dependencies..." -ForegroundColor Yellow
$pythonExe = Join-Path $venvPath "Scripts\python.exe"

if (Test-Path "requirements.txt") {
    Write-Host "Upgrading pip..." -ForegroundColor Cyan
    & $pythonExe -m pip install --upgrade pip --quiet
    
    Write-Host "Installing packages from requirements.txt..." -ForegroundColor Cyan
    & $pythonExe -m pip install -r requirements.txt
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "✗ ERROR: Failed to install dependencies" -ForegroundColor Red
        exit 1
    }
    Write-Host "✓ All dependencies installed successfully" -ForegroundColor Green
} else {
    Write-Host "✗ ERROR: requirements.txt not found in project root" -ForegroundColor Red
    exit 1
}
Write-Host ""

# Clean up old virtual environments
Write-Host "Checking for old virtual environments..." -ForegroundColor Yellow
$oldVenvs = @(
    "Robot_Identification\venv",
    "State Estimation_Python\venv",
    "Video_Robot_Tracking\venv_tracking"
)

$foundOldVenv = $false
foreach ($oldVenv in $oldVenvs) {
    if (Test-Path $oldVenv) {
        $foundOldVenv = $true
        Write-Host "  Found old venv: $oldVenv" -ForegroundColor Yellow
    }
}

if ($foundOldVenv) {
    Write-Host ""
    Write-Host "⚠ Old virtual environments detected (no longer needed)" -ForegroundColor Yellow
    Write-Host "  The project now uses a single .venv at the root" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Do you want to remove old virtual environments? (Y/N): " -ForegroundColor Yellow -NoNewline
    $answer = Read-Host
    
    if ($answer -eq "Y" -or $answer -eq "y") {
        foreach ($oldVenv in $oldVenvs) {
            if (Test-Path $oldVenv) {
                Write-Host "  Removing $oldVenv..." -ForegroundColor Cyan
                Remove-Item -Recurse -Force $oldVenv
            }
        }
        Write-Host "✓ Old virtual environments removed" -ForegroundColor Green
    } else {
        Write-Host "  Skipped removal (you can delete them manually later)" -ForegroundColor Gray
    }
    Write-Host ""
}

# Summary
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "  ✓ SETUP COMPLETED SUCCESSFULLY" -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Environment Information:" -ForegroundColor White
Write-Host "  Python:        $pythonVersion" -ForegroundColor Cyan
Write-Host "  Virtual Env:   E:\Project\.venv" -ForegroundColor Cyan
Write-Host "  Requirements:  Installed from requirements.txt" -ForegroundColor Cyan
Write-Host ""
Write-Host "Available Commands:" -ForegroundColor White
Write-Host ""
Write-Host "  Robot Identification:" -ForegroundColor Yellow
Write-Host "    .\run.ps1 stage1              # Stage 1: Motor parameters" -ForegroundColor Cyan
Write-Host "    .\run.ps1 stage2              # Stage 2: Inertia identification" -ForegroundColor Cyan
Write-Host "    .\run.ps1 stage3              # Stage 3: Compensation factors" -ForegroundColor Cyan
Write-Host ""
Write-Host "  State Estimation:" -ForegroundColor Yellow
Write-Host "    .\run.ps1 ekf                 # Extended Kalman Filter" -ForegroundColor Cyan
Write-Host "    .\run.ps1 ukf                 # Unscented Kalman Filter" -ForegroundColor Cyan
Write-Host "    .\run.ps1 pf                  # Particle Filter" -ForegroundColor Cyan
Write-Host "    .\run.ps1 compare             # Compare all filters" -ForegroundColor Cyan
Write-Host ""
Write-Host "  Video Robot Tracking:" -ForegroundColor Yellow
Write-Host "    .\run.ps1 tracking            # Track robot in video" -ForegroundColor Cyan
Write-Host "    .\run.ps1 process_imu         # Fuse video + IMU data" -ForegroundColor Cyan
Write-Host ""
Write-Host "  Other:" -ForegroundColor Yellow
Write-Host "    .\run.ps1 -Help               # Show all available commands" -ForegroundColor Cyan
Write-Host ""
Write-Host "Note: run.ps1 automatically activates the virtual environment" -ForegroundColor Gray
Write-Host ""

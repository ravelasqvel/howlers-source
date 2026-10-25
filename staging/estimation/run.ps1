# Run script for Omnidirectional Robot Research Project
# Unified execution script for all subprojects
# Author: Automated execution with virtual environment management
#
# Usage:
#   .\run.ps1 <module>             # Run specific module
#   .\run.ps1 -List                # List all available modules
#   .\run.ps1 -Help                # Show detailed help

param(
    [Parameter(Position=0)]
    [string]$Module = "",
    
    [switch]$List,
    [switch]$Help
)

# Color scheme
$ColorTitle = "Cyan"
$ColorSuccess = "Green"
$ColorError = "Red"
$ColorWarning = "Yellow"
$ColorInfo = "White"
$ColorCommand = "Cyan"
$ColorDescription = "Gray"

# Module definitions: [DisplayName, ScriptPath, WorkingDirectory, Description]
$modules = @{
    # Robot Identification
    "stage1" = @("Stage 1: Motor Parameters", "src\examples\stage1_experimental.py", "Robot_Identification", "Identify motor resistance and constant from steady-state tests")
    "stage2" = @("Stage 2: Inertia Matrix", "src\examples\stage2_experimental.py", "Robot_Identification", "Identify robot inertia from rotation tests")
    "stage3" = @("Stage 3: Compensation", "src\examples\stage3_experimental_vid.py", "Robot_Identification", "Calculate compensation factors from video trajectories")
    "sim_test" = @("Simulation Test", "src\examples\test_simulation_vid4.py", "Robot_Identification", "Test robot model simulation vs experimental trajectory data")
    
    # State Estimation
    "ekf" = @("Extended Kalman Filter", "examples\ekf_omnidirectional.py", "State Estimation_Python", "EKF for omnidirectional robot state estimation")
    "ukf" = @("Unscented Kalman Filter", "examples\ukf_omnidirectional.py", "State Estimation_Python", "UKF for omnidirectional robot state estimation")
    "pf" = @("Particle Filter", "examples\pf_omnidirectional.py", "State Estimation_Python", "Particle filter for omnidirectional robot")
    "compare" = @("Compare Filters", "examples\compare_filters.py", "State Estimation_Python", "Compare EKF, UKF, and Particle Filter performance")
    "custom" = @("Custom Model", "examples\custom_model.py", "State Estimation_Python", "Differential drive robot with custom model")
    "compute_q" = @("Compute Process Noise Q", "examples\example_Q_from_identification.py", "State Estimation_Python", "Compute Q matrix from model identification and experimental data")
    
    # Video Robot Tracking
    "tracking" = @("Video Tracking", "track_simple_robust.py", "Video_Robot_Tracking", "Track robot position using optical flow")
    "process_imu" = @("IMU Data Fusion", "process_imu_data.py", "Video_Robot_Tracking", "Fuse video tracking with IMU sensor data")
}

function Show-Help {
    Write-Host ""
    Write-Host "========================================================" -ForegroundColor $ColorTitle
    Write-Host "  Omnidirectional Robot Research - Run Script" -ForegroundColor $ColorTitle
    Write-Host "========================================================" -ForegroundColor $ColorTitle
    Write-Host ""
    Write-Host "USAGE:" -ForegroundColor $ColorWarning
    Write-Host "  .\run.ps1 <module>              " -NoNewline -ForegroundColor $ColorCommand
    Write-Host "# Run specific module" -ForegroundColor $ColorDescription
    Write-Host "  .\run.ps1 -List                 " -NoNewline -ForegroundColor $ColorCommand
    Write-Host "# List all available modules" -ForegroundColor $ColorDescription
    Write-Host "  .\run.ps1 -Help                 " -NoNewline -ForegroundColor $ColorCommand
    Write-Host "# Show this help message" -ForegroundColor $ColorDescription
    Write-Host ""
    Write-Host "EXAMPLES:" -ForegroundColor $ColorWarning
    Write-Host "  .\run.ps1 stage1                " -NoNewline -ForegroundColor $ColorCommand
    Write-Host "# Run Stage 1 identification" -ForegroundColor $ColorDescription
    Write-Host "  .\run.ps1 ukf                   " -NoNewline -ForegroundColor $ColorCommand
    Write-Host "# Run UKF state estimation" -ForegroundColor $ColorDescription
    Write-Host "  .\run.ps1 compute_q             " -NoNewline -ForegroundColor $ColorCommand
    Write-Host "# Compute Q matrix from identification" -ForegroundColor $ColorDescription
    Write-Host "  .\run.ps1 tracking              " -NoNewline -ForegroundColor $ColorCommand
    Write-Host "# Run video tracking" -ForegroundColor $ColorDescription
    Write-Host ""
    Write-Host "AVAILABLE MODULES:" -ForegroundColor $ColorWarning
    Write-Host ""
    Write-Host "  Robot Identification:" -ForegroundColor $ColorInfo
    foreach ($key in @("stage1", "stage2", "stage3", "sim_test")) {
        $info = $modules[$key]
        Write-Host "    $key" -NoNewline -ForegroundColor $ColorCommand
        Write-Host (" " * (15 - $key.Length)) -NoNewline
        Write-Host "- $($info[3])" -ForegroundColor $ColorDescription
    }
    Write-Host ""
    Write-Host "  State Estimation:" -ForegroundColor $ColorInfo
    foreach ($key in @("ekf", "ukf", "pf", "compare", "custom", "compute_q")) {
        $info = $modules[$key]
        Write-Host "    $key" -NoNewline -ForegroundColor $ColorCommand
        Write-Host (" " * (15 - $key.Length)) -NoNewline
        Write-Host "- $($info[3])" -ForegroundColor $ColorDescription
    }
    Write-Host ""
    Write-Host "  Video Robot Tracking:" -ForegroundColor $ColorInfo
    foreach ($key in @("tracking", "process_imu")) {
        $info = $modules[$key]
        Write-Host "    $key" -NoNewline -ForegroundColor $ColorCommand
        Write-Host (" " * (20 - $key.Length)) -NoNewline
        Write-Host "- $($info[3])" -ForegroundColor $ColorDescription
    }
    Write-Host ""
    Write-Host "NOTES:" -ForegroundColor $ColorWarning
    Write-Host "  - Virtual environment is automatically activated" -ForegroundColor $ColorDescription
    Write-Host "  - Results are saved in results/ subdirectories" -ForegroundColor $ColorDescription
    Write-Host "  - Run setup.ps1 first if environment not configured" -ForegroundColor $ColorDescription
    Write-Host ""
}

function Show-List {
    Write-Host ""
    Write-Host "Available Modules:" -ForegroundColor $ColorTitle
    Write-Host ""
    
    Write-Host "Robot Identification:" -ForegroundColor $ColorWarning
    foreach ($key in @("stage1", "stage2", "stage3", "sim_test")) {
        $info = $modules[$key]
        Write-Host "  $key" -NoNewline -ForegroundColor $ColorCommand
        Write-Host (" " * (15 - $key.Length)) -NoNewline
        Write-Host "- $($info[0])" -ForegroundColor $ColorInfo
    }
    Write-Host ""
    
    Write-Host "State Estimation:" -ForegroundColor $ColorWarning
    foreach ($key in @("ekf", "ukf", "pf", "compare", "custom", "compute_q")) {
        $info = $modules[$key]
        Write-Host "  $key" -NoNewline -ForegroundColor $ColorCommand
        Write-Host (" " * (15 - $key.Length)) -NoNewline
        Write-Host "- $($info[0])" -ForegroundColor $ColorInfo
    }
    Write-Host ""
    
    Write-Host "Video Robot Tracking:" -ForegroundColor $ColorWarning
    foreach ($key in @("tracking", "process_imu")) {
        $info = $modules[$key]
        Write-Host "  $key" -NoNewline -ForegroundColor $ColorCommand
        Write-Host (" " * (20 - $key.Length)) -NoNewline
        Write-Host "- $($info[0])" -ForegroundColor $ColorInfo
    }
    Write-Host ""
}

# Show help if requested
if ($Help) {
    Show-Help
    exit 0
}

# Show list if requested
if ($List) {
    Show-List
    exit 0
}

# Show help if no module specified
if ([string]::IsNullOrWhiteSpace($Module)) {
    Write-Host ""
    Write-Host "ERROR: No module specified" -ForegroundColor $ColorError
    Write-Host ""
    Write-Host "Usage: .\run.ps1 <module>" -ForegroundColor $ColorWarning
    Write-Host "       .\run.ps1 -List      (to see available modules)" -ForegroundColor $ColorWarning
    Write-Host "       .\run.ps1 -Help      (for detailed help)" -ForegroundColor $ColorWarning
    Write-Host ""
    exit 1
}

# Validate module
if (-not $modules.ContainsKey($Module.ToLower())) {
    Write-Host ""
    Write-Host "ERROR: Unknown module '$Module'" -ForegroundColor $ColorError
    Write-Host ""
    Write-Host "Available modules:" -ForegroundColor $ColorWarning
    Show-List
    Write-Host "Use '.\run.ps1 -Help' for detailed information" -ForegroundColor $ColorInfo
    Write-Host ""
    exit 1
}

# Get module info
$moduleInfo = $modules[$Module.ToLower()]
$displayName = $moduleInfo[0]
$scriptPath = $moduleInfo[1]
$workingDir = $moduleInfo[2]
$description = $moduleInfo[3]

# Header
Write-Host ""
Write-Host "========================================================" -ForegroundColor $ColorTitle
Write-Host "  $displayName" -ForegroundColor $ColorTitle
Write-Host "========================================================" -ForegroundColor $ColorTitle
Write-Host ""
Write-Host "Description: $description" -ForegroundColor $ColorInfo
Write-Host ""

# Check virtual environment
Write-Host "Checking virtual environment..." -ForegroundColor $ColorWarning
$venvPath = Join-Path $PSScriptRoot ".venv"
$pythonExe = Join-Path $venvPath "Scripts\python.exe"

if (-not (Test-Path $venvPath)) {
    Write-Host "✗ ERROR: Virtual environment not found at E:\Project\.venv" -ForegroundColor $ColorError
    Write-Host ""
    Write-Host "Please run setup first:" -ForegroundColor $ColorWarning
    Write-Host "  .\setup.ps1" -ForegroundColor $ColorCommand
    Write-Host ""
    exit 1
}

if (-not (Test-Path $pythonExe)) {
    Write-Host "✗ ERROR: Python executable not found in virtual environment" -ForegroundColor $ColorError
    Write-Host ""
    Write-Host "Please run setup first:" -ForegroundColor $ColorWarning
    Write-Host "  .\setup.ps1" -ForegroundColor $ColorCommand
    Write-Host ""
    exit 1
}

Write-Host "✓ Virtual environment found: $venvPath" -ForegroundColor $ColorSuccess

# Activate virtual environment if not already active
if (-not $env:VIRTUAL_ENV) {
    Write-Host "Activating virtual environment..." -ForegroundColor $ColorWarning
    $activateScript = Join-Path $venvPath "Scripts\Activate.ps1"
    
    if (Test-Path $activateScript) {
        & $activateScript
        
        if ($env:VIRTUAL_ENV) {
            Write-Host "✓ Virtual environment activated" -ForegroundColor $ColorSuccess
        } else {
            Write-Host "⚠ Warning: Activation may have failed, continuing anyway..." -ForegroundColor $ColorWarning
        }
    }
} else {
    Write-Host "✓ Virtual environment already active" -ForegroundColor $ColorSuccess
}
Write-Host ""

# Change to working directory
$fullWorkingDir = Join-Path $PSScriptRoot $workingDir
$fullScriptPath = Join-Path $fullWorkingDir $scriptPath

if (-not (Test-Path $fullWorkingDir)) {
    Write-Host "✗ ERROR: Working directory not found: $fullWorkingDir" -ForegroundColor $ColorError
    exit 1
}

if (-not (Test-Path $fullScriptPath)) {
    Write-Host "✗ ERROR: Script not found: $fullScriptPath" -ForegroundColor $ColorError
    exit 1
}

Write-Host "Working Directory: $workingDir" -ForegroundColor $ColorInfo
Write-Host "Script:            $scriptPath" -ForegroundColor $ColorInfo
Write-Host ""
Write-Host "========================================================" -ForegroundColor $ColorTitle
Write-Host "  Executing..." -ForegroundColor $ColorTitle
Write-Host "========================================================" -ForegroundColor $ColorTitle
Write-Host ""

# Save current location and change to working directory
$originalLocation = Get-Location
Set-Location $fullWorkingDir

# Execute the script
& $pythonExe $scriptPath

# Capture exit code
$exitCode = $LASTEXITCODE

# Return to original location
Set-Location $originalLocation

# Show result
Write-Host ""
Write-Host "========================================================" -ForegroundColor $ColorTitle
if ($exitCode -eq 0) {
    Write-Host "  ✓ EXECUTION COMPLETED SUCCESSFULLY" -ForegroundColor $ColorSuccess
} else {
    Write-Host "  ✗ EXECUTION FAILED (Exit Code: $exitCode)" -ForegroundColor $ColorError
}
Write-Host "========================================================" -ForegroundColor $ColorTitle
Write-Host ""

exit $exitCode

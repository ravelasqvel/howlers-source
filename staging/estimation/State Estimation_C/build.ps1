# ============================================================================
# Build Script for State Estimation Examples
# Author: MaverickST
# Description: Unified build script for UKF, PF, and EKF examples
# ============================================================================

param(
    [Parameter(Mandatory=$false)]
    [ValidateSet("ukf", "pf", "ekf", "menu", "all", "help")]
    [string]$Target = "help"
)

# Colors for output
$ColorInfo = "Cyan"
$ColorSuccess = "Green"
$ColorError = "Red"
$ColorWarning = "Yellow"

# ============================================================================
# HELP FUNCTION
# ============================================================================

function Show-Help {
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host "State Estimation Build Script" -ForegroundColor $ColorInfo
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host ""
    Write-Host "Usage:" -ForegroundColor $ColorWarning
    Write-Host "  .\build.ps1 <target>" -ForegroundColor White
    Write-Host ""
    Write-Host "Targets:" -ForegroundColor $ColorWarning
    Write-Host "  menu   - Build unified menu (all examples in one executable)" -ForegroundColor White
    Write-Host "  ukf    - Build Unscented Kalman Filter example" -ForegroundColor White
    Write-Host "  pf     - Build Particle Filter example" -ForegroundColor White
    Write-Host "  ekf    - Build Extended Kalman Filter example" -ForegroundColor White
    Write-Host "  all    - Build all examples" -ForegroundColor White
    Write-Host "  help   - Show this help message" -ForegroundColor White
    Write-Host ""
    Write-Host "Examples:" -ForegroundColor $ColorWarning
    Write-Host "  .\build.ps1 menu   # Build menu.exe (interactive)" -ForegroundColor White
    Write-Host "  .\build.ps1 ukf    # Build ukf_demo.exe" -ForegroundColor White
    Write-Host "  .\build.ps1 pf     # Build pf_demo.exe" -ForegroundColor White
    Write-Host "  .\build.ps1 all    # Build all examples" -ForegroundColor White
    Write-Host ""
}

# ============================================================================
# CHECK GCC
# ============================================================================

function Test-GCC {
    try {
        $gccVersion = gcc --version 2>&1 | Select-Object -First 1
        Write-Host "[OK] GCC found: $gccVersion" -ForegroundColor $ColorSuccess
        return $true
    } catch {
        Write-Host "[ERROR] GCC not found! Please install MinGW or similar." -ForegroundColor $ColorError
        return $false
    }
}

# ============================================================================
# BUILD UKF
# ============================================================================

function Build-UKF {
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host "Building UKF Example" -ForegroundColor $ColorInfo
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host ""

    Write-Host "Compiling UKF example..." -ForegroundColor $ColorWarning
    
    gcc -o ukf_demo.exe Examples/ukf_example.c UKF/ukf.c `
        LinearAlgebra/chol.c LinearAlgebra/inv.c LinearAlgebra/mul.c `
        LinearAlgebra/tran.c LinearAlgebra/lup.c LinearAlgebra/linsolve_lup.c `
        LinearAlgebra/linsolve_lower_triangular.c `
        LinearAlgebra/linsolve_upper_triangular.c LinearAlgebra/dot.c `
        LinearAlgebra/angle_utils.c `
        -I. -IHeaders -ILinearAlgebra -lm
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] UKF compilation failed!" -ForegroundColor $ColorError
        return $false
    }
    
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorSuccess
    Write-Host "[OK] UKF build successful!" -ForegroundColor $ColorSuccess
    Write-Host "============================================" -ForegroundColor $ColorSuccess
    Write-Host ""
    Write-Host "Run with: " -NoNewline
    Write-Host ".\ukf_demo.exe" -ForegroundColor $ColorWarning
    Write-Host ""
    
    return $true
}

# ============================================================================
# BUILD PF
# ============================================================================

function Build-PF {
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host "Building Particle Filter Example" -ForegroundColor $ColorInfo
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host ""

    Write-Host "Compiling PF example..." -ForegroundColor $ColorWarning
    
    gcc -o pf_demo.exe Examples/pf_example.c PF/pf.c `
        LinearAlgebra/angle_utils.c `
        -I. -IHeaders -ILinearAlgebra -lm
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] PF compilation failed!" -ForegroundColor $ColorError
        return $false
    }
    
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorSuccess
    Write-Host "[OK] PF build successful!" -ForegroundColor $ColorSuccess
    Write-Host "============================================" -ForegroundColor $ColorSuccess
    Write-Host ""
    Write-Host "Run with: " -NoNewline
    Write-Host ".\pf_demo.exe" -ForegroundColor $ColorWarning
    Write-Host ""
    
    return $true
}

# ============================================================================
# BUILD EKF
# ============================================================================

function Build-EKF {
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host "Building EKF Example" -ForegroundColor $ColorInfo
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host ""

    Write-Host "Compiling EKF example..." -ForegroundColor $ColorWarning
    
    gcc -o ekf_demo.exe Examples/ekf_example.c EKF/ekf.c `
        LinearAlgebra/mul.c LinearAlgebra/inv.c LinearAlgebra/tran.c `
        LinearAlgebra/lup.c LinearAlgebra/linsolve_lup.c `
        LinearAlgebra/linsolve_lower_triangular.c `
        LinearAlgebra/linsolve_upper_triangular.c LinearAlgebra/dot.c `
        LinearAlgebra/angle_utils.c `
        -I. -IHeaders -ILinearAlgebra -lm
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] EKF compilation failed!" -ForegroundColor $ColorError
        return $false
    }
    
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorSuccess
    Write-Host "[OK] EKF build successful!" -ForegroundColor $ColorSuccess
    Write-Host "============================================" -ForegroundColor $ColorSuccess
    Write-Host ""
    Write-Host "Run with: " -NoNewline
    Write-Host ".\ekf_demo.exe" -ForegroundColor $ColorWarning
    Write-Host ""
    
    return $true
}

# ============================================================================
# BUILD ALL
# ============================================================================
# BUILD MENU (Unified executable with all examples)
# ============================================================================

function Build-Menu {
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host "Building Unified Menu System" -ForegroundColor $ColorInfo
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host ""

    Write-Host "Compiling menu with all examples..." -ForegroundColor $ColorWarning
    Write-Host "This will create a single executable with interactive menu" -ForegroundColor $ColorInfo
    Write-Host ""
    
    # Compile with BUILD_WITH_MENU flag to disable individual main() functions
    gcc -o menu.exe main.c Examples/ukf_example.c Examples/pf_example.c Examples/ekf_example.c `
        UKF/ukf.c PF/pf.c EKF/ekf.c `
        LinearAlgebra/chol.c LinearAlgebra/inv.c LinearAlgebra/mul.c `
        LinearAlgebra/tran.c LinearAlgebra/lup.c LinearAlgebra/linsolve_lup.c `
        LinearAlgebra/linsolve_lower_triangular.c `
        LinearAlgebra/linsolve_upper_triangular.c LinearAlgebra/dot.c `
        LinearAlgebra/angle_utils.c `
        -DBUILD_WITH_MENU `
        -I. -IHeaders -ILinearAlgebra -IUKFs -IPF -IEKF `
        -lm
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] Menu compilation failed!" -ForegroundColor $ColorError
        return $false
    }
    
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorSuccess
    Write-Host "[OK] Menu build successful!" -ForegroundColor $ColorSuccess
    Write-Host "============================================" -ForegroundColor $ColorSuccess
    Write-Host ""
    Write-Host "Run with: " -NoNewline
    Write-Host ".\menu.exe" -ForegroundColor $ColorWarning
    Write-Host ""
    Write-Host "The menu provides interactive access to:" -ForegroundColor $ColorInfo
    Write-Host "  [1] UKF - Unscented Kalman Filter" -ForegroundColor White
    Write-Host "  [2] PF  - Particle Filter" -ForegroundColor White
    Write-Host "  [3] EKF - Extended Kalman Filter" -ForegroundColor White
    Write-Host ""
    
    return $true
}

# ============================================================================
# BUILD ALL
# ============================================================================

function Build-All {
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host "Building All Examples" -ForegroundColor $ColorInfo
    Write-Host "============================================" -ForegroundColor $ColorInfo
    
    $ukfSuccess = Build-UKF
    $pfSuccess = Build-PF
    $ekfSuccess = Build-EKF
    
    Write-Host ""
    Write-Host "============================================" -ForegroundColor $ColorInfo
    Write-Host "Build Summary" -ForegroundColor $ColorInfo
    Write-Host "============================================" -ForegroundColor $ColorInfo
    
    if ($ukfSuccess) {
        Write-Host "[OK] UKF:  ukf_demo.exe" -ForegroundColor $ColorSuccess
    } else {
        Write-Host "[FAIL] UKF" -ForegroundColor $ColorError
    }
    
    if ($pfSuccess) {
        Write-Host "[OK] PF:   pf_demo.exe" -ForegroundColor $ColorSuccess
    } else {
        Write-Host "[FAIL] PF" -ForegroundColor $ColorError
    }
    
    if ($ekfSuccess) {
        Write-Host "[OK] EKF:  ekf_demo.exe" -ForegroundColor $ColorSuccess
    } else {
        Write-Host "[FAIL] EKF" -ForegroundColor $ColorError
    }
    Write-Host ""
    
    return ($ukfSuccess -and $pfSuccess -and $ekfSuccess)
}

# ============================================================================
# MAIN
# ============================================================================

Write-Host ""
Write-Host "============================================" -ForegroundColor $ColorInfo
Write-Host "State Estimation Build Script" -ForegroundColor $ColorInfo
Write-Host "============================================" -ForegroundColor $ColorInfo
Write-Host ""

# Check GCC
if (-not (Test-GCC)) {
    exit 1
}

# Execute based on target
switch ($Target.ToLower()) {
    "menu" {
        $success = Build-Menu
        if ($success) { exit 0 } else { exit 1 }
    }
    "ukf" {
        $success = Build-UKF
        if ($success) { exit 0 } else { exit 1 }
    }
    "pf" {
        $success = Build-PF
        if ($success) { exit 0 } else { exit 1 }
    }
    "ekf" {
        $success = Build-EKF
        if ($success) { exit 0 } else { exit 1 }
    }
    "all" {
        $success = Build-All
        if ($success) { exit 0 } else { exit 1 }
    }
    "help" {
        Show-Help
        exit 0
    }
    default {
        Show-Help
        exit 1
    }
}

# Clean script for UKF Implementation (PowerShell)
# Author: MaverickST
# Description: Removes all build artifacts

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "Cleaning build artifacts..." -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Remove object files
$objectFiles = @(
    "main.o",
    "UKF/ukf.o",
    "LinearAlgebra/chol.o",
    "LinearAlgebra/inv.o",
    "LinearAlgebra/mul.o",
    "LinearAlgebra/tran.o",
    "LinearAlgebra/lup.o",
    "LinearAlgebra/linsolve_lup.o",
    "LinearAlgebra/linsolve_lower_triangular.o",
    "LinearAlgebra/linsolve_upper_triangular.o",
    "LinearAlgebra/dot.o"
)

$removed = 0
foreach ($file in $objectFiles) {
    if (Test-Path $file) {
        Remove-Item $file -Force
        Write-Host "  ✓ Removed $file" -ForegroundColor Gray
        $removed++
    }
}

# Remove executable
if (Test-Path "ukf_demo.exe") {
    Remove-Item "ukf_demo.exe" -Force
    Write-Host "  ✓ Removed ukf_demo.exe" -ForegroundColor Gray
    $removed++
}

Write-Host ""
if ($removed -gt 0) {
    Write-Host "✓ Cleaned $removed file(s)" -ForegroundColor Green
} else {
    Write-Host "✓ Nothing to clean (already clean)" -ForegroundColor Green
}
Write-Host ""

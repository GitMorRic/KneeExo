# tools/idf-shell.ps1
# Activate ESP-IDF v5.5.4 in this PowerShell session.
#
# Usage (from project root):
#   . .\tools\idf-shell.ps1
#
# After this you can use idf.py / esptool.py in the same terminal.
# Change $IdfProfile below if you reinstall IDF to another location.

param()

$IdfProfile = "C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1"

if (-not (Test-Path $IdfProfile)) {
    Write-Error "ESP-IDF profile not found at: $IdfProfile. Edit tools/idf-shell.ps1 to point at your install."
    return
}

. $IdfProfile

Write-Host ""
Write-Host "ESP-IDF env ready. Try:"
Write-Host "  idf.py set-target esp32s3"
Write-Host "  idf.py menuconfig            # Application -> App profile"
Write-Host "  idf.py -p COMx flash monitor"

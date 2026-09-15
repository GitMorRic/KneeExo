# Activate ESP-IDF in this PowerShell session.
# Reuse an active environment; override the installer profile via -IdfProfile
# or KNEEEXO_IDF_PROFILE when IDF is installed elsewhere.
param([string]$IdfProfile = "")

$ErrorActionPreference = "Stop"
if (-not $IdfProfile -and -not $env:KNEEEXO_IDF_PROFILE -and
    $env:IDF_PATH -and (Test-Path -LiteralPath $env:IDF_PATH) -and
    (Get-Command idf.py -ErrorAction SilentlyContinue)) {
    Write-Host "[idf-shell] Reusing active ESP-IDF: $env:IDF_PATH"
    return
}
if (-not $IdfProfile) { $IdfProfile = $env:KNEEEXO_IDF_PROFILE }
if (-not $IdfProfile) {
    $IdfProfile = "C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1"
}
if (-not (Test-Path -LiteralPath $IdfProfile -PathType Leaf)) {
    throw "ESP-IDF profile not found: $IdfProfile. Open an ESP-IDF PowerShell terminal, or set KNEEEXO_IDF_PROFILE to your installer profile."
}
. $IdfProfile
if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
    throw "The selected profile did not provide idf.py: $IdfProfile"
}
Write-Host "[idf-shell] Ready: $env:IDF_PATH (project tested with ESP-IDF v5.5.4)"

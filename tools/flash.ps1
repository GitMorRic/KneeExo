# Select profile, build, optionally flash and monitor.
# No Profile: preserve current sdkconfig. No Port: build only.
# Examples: .\tools\flash.ps1 motor COM3
#           .\tools\flash.ps1 assist COM3 flash
#           .\tools\flash.ps1 wear
param(
    [Parameter(Position=0)] [string]$Profile = "",
    [Parameter(Position=1)] [string]$Port = "",
    [Parameter(Position=2)]
    [ValidateSet("monitor", "flash", "build", "nomon", "nomonitor", "flashonly")]
    [string]$Action = "monitor",
    [string]$IdfProfile = ""
)

$ErrorActionPreference = "Stop"
$profileMap = @{
    normal = "NORMAL"; twai = "TWAI_LOOPBACK"; imu = "IMU_ONLY"
    motor = "MOTOR_TEST"; limit = "MOTOR_TEST"; calib = "MOTOR_TEST"
    wear = "NORMAL"; transparent = "NORMAL"; damp = "NORMAL"; damping = "NORMAL"
    assist = "NORMAL"; follow = "NORMAL"; gait = "NORMAL"; assistgait = "NORMAL"
}
$profileName = $Profile.ToLowerInvariant()
if ($Profile -and -not $profileMap.ContainsKey($profileName)) {
    throw "Unknown profile '$Profile'. Valid: $(($profileMap.Keys | Sort-Object) -join ', ')"
}
if ($Action -in @("nomon", "nomonitor", "flashonly")) { $Action = "flash" }

function Invoke-IdfChecked {
    param([string[]]$IdfArgs)
    & idf.py @IdfArgs
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py $($IdfArgs -join ' ') failed (exit $LASTEXITCODE)."
    }
}

# Match enabled/disabled lines; insert keys omitted by prior Kconfig dependencies.
function Set-ConfigValue {
    param([string]$Key, [string]$Value)
    $pattern = "(?m)^(?:CONFIG_${Key}=[^\r\n]*|# CONFIG_${Key} is not set)\r?$"
    $line = if ($Value -eq "n") { "# CONFIG_${Key} is not set" } else { "CONFIG_${Key}=$Value" }
    if ([regex]::IsMatch($script:ConfigText, $pattern)) {
        $script:ConfigText = [regex]::Replace($script:ConfigText, $pattern, $line)
    } else {
        $script:ConfigText = $script:ConfigText.TrimEnd() + "`n$line`n"
    }
}

function Set-ConfigDefault {
    param([string]$Key, [string]$Value)
    if ($script:ConfigText -notmatch "(?m)^CONFIG_${Key}=") {
        Set-ConfigValue $Key $Value
    }
}

$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectRoot
try {
    . "$PSScriptRoot\idf-shell.ps1" -IdfProfile $IdfProfile
    if (-not (Test-Path -LiteralPath sdkconfig)) {
        Write-Host "[flash.ps1] Creating sdkconfig from sdkconfig.defaults..."
        Invoke-IdfChecked -IdfArgs @("reconfigure")
    }

    if ($Profile) {
        $script:ConfigText = Get-Content -LiteralPath sdkconfig -Raw
        foreach ($name in @("NORMAL", "TWAI_LOOPBACK", "IMU_ONLY", "MOTOR_TEST")) {
            Set-ConfigValue "KNEEEXO_APP_PROFILE_$name" "n"
        }
        $selected = $profileMap[$profileName]
        Set-ConfigValue "KNEEEXO_APP_PROFILE_$selected" "y"
        Set-ConfigValue "KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN" "n"
        Set-ConfigValue "KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB" "n"

        if ($profileName -in @("limit", "calib")) {
            Set-ConfigValue "KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB" "y"
            Set-ConfigDefault "KNEEEXO_LIMIT_CALIB_TORQUE_X100" "120"
            Set-ConfigDefault "KNEEEXO_LIMIT_CALIB_SPEED_MRAD_S" "120"
            Set-ConfigDefault "KNEEEXO_LIMIT_CALIB_MAX_TRAVEL_MRAD" "2500"
            Set-ConfigDefault "KNEEEXO_LIMIT_CALIB_HOLD_MS" "350"
        }

        if ($selected -eq "NORMAL") {
            foreach ($mode in @("TRANSPARENT", "DAMPING", "ASSIST")) {
                Set-ConfigValue "KNEEEXO_WEAR_MODE_$mode" "n"
            }
            $wearMode = if ($profileName -in @("damp", "damping")) { "DAMPING" }
                        elseif ($profileName -in @("assist", "follow", "assistgait")) { "ASSIST" }
                        else { "TRANSPARENT" }
            Set-ConfigValue "KNEEEXO_WEAR_MODE_$wearMode" "y"
            $gaitEnabled = if ($profileName -in @("gait", "assistgait")) { "y" } else { "n" }
            Set-ConfigValue "KNEEEXO_GAIT_IMU_ENABLE" $gaitEnabled
            foreach ($pair in @{
                KNEEEXO_USER_ZERO_CALIB_MS = "3000"
                KNEEEXO_DAMPING_B_X100 = "30"
                KNEEEXO_DAMPING_MAX_TORQUE_X100 = "80"
                KNEEEXO_ASSIST_FOLLOW_B_X100 = "10"
                KNEEEXO_ASSIST_MAX_TORQUE_X100 = "35"
                KNEEEXO_ASSIST_VEL_DEADBAND_MRAD_S = "120"
                KNEEEXO_LANDING_BUFFER_B_X100 = "80"
                KNEEEXO_LANDING_BUFFER_MAX_TORQUE_X100 = "80"
                KNEEEXO_LANDING_BUFFER_NEAR_EXT_MRAD = "450"
                KNEEEXO_LANDING_BUFFER_MIN_FLEX_VEL_MRAD_S = "350"
                KNEEEXO_GAIT_IMPACT_SPIKE_X100 = "18"
                KNEEEXO_GAIT_LANDING_COOLDOWN_MS = "350"
                KNEEEXO_GAIT_SWING_RATE_DPS = "35"
            }.GetEnumerator()) {
                Set-ConfigDefault $pair.Key $pair.Value
            }
        }
        if ($selected -eq "IMU_ONLY") {
            Set-ConfigDefault "KNEEEXO_IMU_CSV_HZ" "100"
        }
        [System.IO.File]::WriteAllText((Join-Path $projectRoot "sdkconfig"),
            $script:ConfigText, [System.Text.UTF8Encoding]::new($false))
        Write-Host "[flash.ps1] Requested profile: $Profile" -ForegroundColor Cyan
    }

    Write-Host "[flash.ps1] Building..." -ForegroundColor Yellow
    Invoke-IdfChecked -IdfArgs @("build")

    # Report after Kconfig has resolved dependencies, not just the requested mode.
    Write-Host "[flash.ps1] Effective build configuration:" -ForegroundColor Green
    Select-String -Path sdkconfig -Pattern "^CONFIG_KNEEEXO_APP_PROFILE_\w+=y",
        "^CONFIG_KNEEEXO_WEAR_MODE_\w+=y",
        "^(CONFIG_KNEEEXO_GAIT_IMU_ENABLE=y|# CONFIG_KNEEEXO_GAIT_IMU_ENABLE is not set)",
        "^CONFIG_KNEEEXO_PROFILE_MOTOR_(TEST_AUTOSPIN|LIMIT_CALIB)=y" |
        ForEach-Object { Write-Host "  $($_.Line)" }
    $description = Get-Content -LiteralPath build/project_description.json -Raw | ConvertFrom-Json
    $binary = Join-Path $projectRoot "build/knee_exo.bin"
    Write-Host "[flash.ps1] Built app version: $($description.project_version)"
    Write-Host "[flash.ps1] Binary: $binary"
    Write-Host "[flash.ps1] Binary SHA256: $((Get-FileHash -LiteralPath $binary -Algorithm SHA256).Hash)"

    if ($Port -and $Action -ne "build") {
        Write-Host "[flash.ps1] Flashing on $Port ..." -ForegroundColor Yellow
        if ($Action -eq "monitor") {
            Invoke-IdfChecked -IdfArgs @("-p", $Port, "flash", "monitor")
        } else {
            Invoke-IdfChecked -IdfArgs @("-p", $Port, "flash")
            Write-Host "[flash.ps1] Flash completed. Serial port is free." -ForegroundColor Green
        }
    } else {
        Write-Host "[flash.ps1] Build only; no device was flashed." -ForegroundColor Gray
    }
} finally {
    Pop-Location
}

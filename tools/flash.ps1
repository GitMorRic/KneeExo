# tools/flash.ps1 - 一键切 profile + build + flash + monitor
#
# 用法：
#   . .\tools\idf-shell.ps1         # 第一次开终端时激活一次 IDF 环境
#   .\tools\flash.ps1 imu COM3      # 烧录 imu_only profile 到 COM3
#
# Profile 缩写（不区分大小写）：
#   normal     ->  KNEEEXO_APP_PROFILE_NORMAL
#   twai       ->  KNEEEXO_APP_PROFILE_TWAI_LOOPBACK
#   imu        ->  KNEEEXO_APP_PROFILE_IMU_ONLY
#   motor      ->  KNEEEXO_APP_PROFILE_MOTOR_TEST
#   limit      ->  motor_test + KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB
#   wear       ->  normal + transparent + user-zero
#   damp       ->  normal + damping + user-zero
#   assist     ->  normal + small velocity-follow assist + landing buffer
#   gait       ->  normal + transparent + IMU gait display
#   assistgait -> normal + assist + IMU gait display
#
# 示例：
#   .\tools\flash.ps1                      # 只 build（normal，上次用的 COM）
#   .\tools\flash.ps1 imu                  # 切到 imu_only 然后 build（不烧录）
#   .\tools\flash.ps1 imu COM3             # 切到 imu_only，build + flash + monitor
#   .\tools\flash.ps1 motor COM3 nomon     # 烧录但不打开 monitor（给 imu_plot.py 让口）
#   .\tools\flash.ps1 twai COM3 flash      # 只烧录，不 monitor（同上）

param(
    [Parameter(Position=0)]
    [string]$Profile = "",

    [Parameter(Position=1)]
    [string]$Port = "",

    [Parameter(Position=2)]
    [string]$Action = "monitor"   # monitor | flash | build
)

Set-StrictMode -Off
$ErrorActionPreference = "Stop"

# ---------- 1. profile 名称 -> CONFIG key ----------
$profileMap = @{
    "normal" = "KNEEEXO_APP_PROFILE_NORMAL"
    "twai"   = "KNEEEXO_APP_PROFILE_TWAI_LOOPBACK"
    "imu"    = "KNEEEXO_APP_PROFILE_IMU_ONLY"
    "motor"  = "KNEEEXO_APP_PROFILE_MOTOR_TEST"
    "limit"  = "KNEEEXO_APP_PROFILE_MOTOR_TEST"
    "calib"  = "KNEEEXO_APP_PROFILE_MOTOR_TEST"
    "wear"   = "KNEEEXO_APP_PROFILE_NORMAL"
    "transparent" = "KNEEEXO_APP_PROFILE_NORMAL"
    "damp"   = "KNEEEXO_APP_PROFILE_NORMAL"
    "damping" = "KNEEEXO_APP_PROFILE_NORMAL"
    "assist" = "KNEEEXO_APP_PROFILE_NORMAL"
    "follow" = "KNEEEXO_APP_PROFILE_NORMAL"
    "gait" = "KNEEEXO_APP_PROFILE_NORMAL"
    "assistgait" = "KNEEEXO_APP_PROFILE_NORMAL"
}

# action aliases
if ($Action -in @("nomon","nomonitor","flashonly")) { $Action = "flash" }

# 每次都激活 IDF 环境（幂等，重复激活无副作用，确保跨 shell 会话可用）
. "$PSScriptRoot\idf-shell.ps1"

# ---------- 2. 切换 sdkconfig ----------
if ($Profile -ne "") {
    $key = $profileMap[$Profile.ToLower()]
    if (-not $key) {
        Write-Error "Unknown profile '$Profile'. Valid: $($profileMap.Keys -join ', ')"
        exit 1
    }

    $sdkconfig = "sdkconfig"
    if (-not (Test-Path $sdkconfig)) {
        Write-Error "sdkconfig not found. Run 'idf.py set-target esp32s3' first."
        exit 1
    }

    $content = Get-Content $sdkconfig -Raw

    # 把所有 profile key 全部 disable
    foreach ($v in $profileMap.Values) {
        $content = $content -replace "(?m)^CONFIG_${v}=y", "# CONFIG_${v} is not set"
    }

    # 只启用目标 key
    $content = $content -replace "# CONFIG_${key} is not set", "CONFIG_${key}=y"

    # motor 子模式默认全部关闭，避免误触发电机动作
    foreach ($sub in @("KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN", "KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB")) {
        if ($content -match "CONFIG_${sub}=y") {
            $content = $content -replace "(?m)^CONFIG_${sub}=y", "# CONFIG_${sub} is not set"
        } elseif ($content -notmatch "CONFIG_${sub}") {
            $content = $content -replace "(CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST=y)", "`$1`n# CONFIG_${sub} is not set"
        }
    }

    if ($Profile.ToLower() -in @("limit", "calib")) {
        $content = $content -replace "# CONFIG_KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB is not set", "CONFIG_KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB=y"
        foreach ($pair in @{
            "KNEEEXO_LIMIT_CALIB_TORQUE_X100" = "120";
            "KNEEEXO_LIMIT_CALIB_SPEED_MRAD_S" = "120";
            "KNEEEXO_LIMIT_CALIB_MAX_TRAVEL_MRAD" = "2500";
            "KNEEEXO_LIMIT_CALIB_HOLD_MS" = "350";
        }.GetEnumerator()) {
            if ($content -notmatch "CONFIG_$($pair.Key)=") {
                $content = $content -replace "(CONFIG_KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB=y)", "`$1`nCONFIG_$($pair.Key)=$($pair.Value)"
            }
        }
    }

    # normal/wear 子模式：默认安全透明，damp/damping 才开启小阻尼
    if ($key -eq "KNEEEXO_APP_PROFILE_NORMAL") {
        foreach ($mode in @("KNEEEXO_WEAR_MODE_TRANSPARENT", "KNEEEXO_WEAR_MODE_DAMPING", "KNEEEXO_WEAR_MODE_ASSIST")) {
            if ($content -match "CONFIG_${mode}=y") {
                $content = $content -replace "(?m)^CONFIG_${mode}=y", "# CONFIG_${mode} is not set"
            } elseif ($content -notmatch "CONFIG_${mode}") {
                $content = $content -replace "(CONFIG_KNEEEXO_APP_PROFILE_NORMAL=y)", "`$1`n# CONFIG_${mode} is not set"
            }
        }

        if ($Profile.ToLower() -in @("damp", "damping")) {
            $content = $content -replace "# CONFIG_KNEEEXO_WEAR_MODE_DAMPING is not set", "CONFIG_KNEEEXO_WEAR_MODE_DAMPING=y"
        } elseif ($Profile.ToLower() -in @("assist", "follow", "assistgait")) {
            $content = $content -replace "# CONFIG_KNEEEXO_WEAR_MODE_ASSIST is not set", "CONFIG_KNEEEXO_WEAR_MODE_ASSIST=y"
        } else {
            $content = $content -replace "# CONFIG_KNEEEXO_WEAR_MODE_TRANSPARENT is not set", "CONFIG_KNEEEXO_WEAR_MODE_TRANSPARENT=y"
        }

        if ($Profile.ToLower() -in @("gait", "assistgait")) {
            if ($content -match "CONFIG_KNEEEXO_GAIT_IMU_ENABLE=y") {
                # already enabled
            } elseif ($content -match "# CONFIG_KNEEEXO_GAIT_IMU_ENABLE is not set") {
                $content = $content -replace "# CONFIG_KNEEEXO_GAIT_IMU_ENABLE is not set", "CONFIG_KNEEEXO_GAIT_IMU_ENABLE=y"
            } else {
                $content = $content -replace "(CONFIG_KNEEEXO_APP_PROFILE_NORMAL=y)", "`$1`nCONFIG_KNEEEXO_GAIT_IMU_ENABLE=y"
            }
        } else {
            if ($content -match "CONFIG_KNEEEXO_GAIT_IMU_ENABLE=y") {
                $content = $content -replace "(?m)^CONFIG_KNEEEXO_GAIT_IMU_ENABLE=y", "# CONFIG_KNEEEXO_GAIT_IMU_ENABLE is not set"
            }
        }

        foreach ($pair in @{
            "KNEEEXO_USER_ZERO_CALIB_MS" = "3000";
            "KNEEEXO_DAMPING_B_X100" = "30";
            "KNEEEXO_DAMPING_MAX_TORQUE_X100" = "80";
            "KNEEEXO_ASSIST_FOLLOW_B_X100" = "10";
            "KNEEEXO_ASSIST_MAX_TORQUE_X100" = "35";
            "KNEEEXO_ASSIST_VEL_DEADBAND_MRAD_S" = "120";
            "KNEEEXO_LANDING_BUFFER_B_X100" = "80";
            "KNEEEXO_LANDING_BUFFER_MAX_TORQUE_X100" = "80";
            "KNEEEXO_LANDING_BUFFER_NEAR_EXT_MRAD" = "450";
            "KNEEEXO_LANDING_BUFFER_MIN_FLEX_VEL_MRAD_S" = "350";
            "KNEEEXO_GAIT_IMPACT_SPIKE_X100" = "18";
            "KNEEEXO_GAIT_LANDING_COOLDOWN_MS" = "350";
            "KNEEEXO_GAIT_SWING_RATE_DPS" = "35";
        }.GetEnumerator()) {
            if ($content -notmatch "CONFIG_$($pair.Key)=") {
                $content = $content -replace "(CONFIG_KNEEEXO_APP_PROFILE_NORMAL=y)", "`$1`nCONFIG_$($pair.Key)=$($pair.Value)"
            }
        }
    }

    # imu_only profile 额外确保 IMU_CSV_HZ 出现
    if ($key -eq "KNEEEXO_APP_PROFILE_IMU_ONLY") {
        if ($content -notmatch "CONFIG_KNEEEXO_IMU_CSV_HZ") {
            $content = $content -replace "(CONFIG_KNEEEXO_APP_PROFILE_IMU_ONLY=y)", "`$1`nCONFIG_KNEEEXO_IMU_CSV_HZ=100"
        }
    }

    Set-Content $sdkconfig $content -NoNewline

    Write-Host "[flash.ps1] Profile switched to: $Profile  ($key)" -ForegroundColor Cyan
}

# ---------- 3. 查看当前 profile ----------
$current = (Select-String -Path sdkconfig -Pattern "^CONFIG_KNEEEXO_APP_PROFILE_\w+=y" | Select-Object -First 1).Line
Write-Host "[flash.ps1] Current sdkconfig profile: $current" -ForegroundColor Green

# ---------- 4. build ----------
Write-Host "[flash.ps1] Building..." -ForegroundColor Yellow
idf.py build
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed!"; exit 1 }

# ---------- 5. flash / monitor ----------
if ($Port -ne "") {
    if ($Action -eq "monitor") {
        Write-Host "[flash.ps1] Flashing + monitoring on $Port ..." -ForegroundColor Yellow
        idf.py -p $Port flash monitor
    } elseif ($Action -eq "flash") {
        Write-Host "[flash.ps1] Flashing on $Port (no monitor)..." -ForegroundColor Yellow
        idf.py -p $Port flash
        Write-Host "[flash.ps1] Done. Serial port free for other tools." -ForegroundColor Green
    }
} else {
    Write-Host "[flash.ps1] No port specified. Build only. To flash: .\tools\flash.ps1 $Profile COMx" -ForegroundColor Gray
}

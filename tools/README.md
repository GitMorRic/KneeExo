# tools/

PC 端配套脚本。

## flash.ps1 — 推荐日常入口

不用 menuconfig，一条命令切 profile + build + flash：

```powershell
# 第一次开终端，激活 IDF 环境（之后当前终端不用再激活）
. .\tools\idf-shell.ps1

# 常用示例
.\tools\flash.ps1 twai  COM3          # TWAI 自检，烧录+monitor
.\tools\flash.ps1 imu   COM3          # IMU 可视化，烧录+monitor
.\tools\flash.ps1 imu   COM3 flash    # 烧录但不开 monitor（给 imu_plot.py 让口）
.\tools\flash.ps1 motor COM3          # 电机测试，烧录+monitor
.\tools\flash.ps1 normal COM3         # 正式形态，烧录+monitor

# 只切 profile + build，不烧录（检查编译）
.\tools\flash.ps1 imu
```

Profile 缩写：`normal` / `twai` / `imu` / `motor`

---

## idf-shell.ps1

一键拉起 ESP-IDF v5.5.4 PowerShell 环境。每次开新终端都要 dot-source 一次：

```powershell
. .\tools\idf-shell.ps1
```

之后 `idf.py` / `esptool.py` / `python` (IDF 自带的 venv) 都可用。

> 实质就是 dot-source `C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1`。换一台机器只需改这个路径。

## imu_plot.py

配合固件 profile = `imu_only` 用。固件每 10ms（默认）通过主串口打印一行 CSV：

```
$IMU,t_us,ax,ay,az,gx,gy,gz,roll,pitch,yaw,T
```

PC 端实时画 4 张曲线图（加速度 / 角速度 / 欧拉角 / 温度）。

### 安装依赖

```powershell
. .\tools\idf-shell.ps1            # 用 IDF 自带的 python venv，省得另装
pip install pyserial matplotlib
```

### 运行

```powershell
# 1) 在另一个 IDF shell 里烧录 imu_only 固件后退出 monitor，把串口让出来
idf.py menuconfig                  # 选 Application -> App profile = imu_only
idf.py -p COM3 flash               # 不再 monitor
# 2) 然后运行实时绘图
python tools\imu_plot.py --port COM3
```

按 Ctrl+C 退出绘图。

### 注意

- `idf.py monitor` 与本脚本会争抢同一个 COM 口，二选一。
- ESP_LOGx 的日志行不以 `$IMU,` 开头，会被脚本忽略。如果想完全无干扰，可以在 menuconfig 里把 `Component config -> Log output -> Default log verbosity` 调到 Warning。

# 编译、识别固件与重新烧录

本项目使用 **ESP-IDF v5.5.4 / ESP32-S3-N16R8（16 MB Flash、8 MB Octal PSRAM）**。下文命令在 Windows PowerShell 中执行，`COM3` 只是示例，必须替换为实际端口。

> 先在无人穿戴、机构可靠固定的台架上恢复调试。`normal`、`wear`、`assist` 等 NORMAL 程序会在启动标定后自动使能电机；`transparent` 也可能叠加状态阻抗和已保存的重力补偿。烧录完成通常会复位运行，打开普通 monitor 也可能复位。不要把模式名称当成硬件断力保证。

## 1. 先区分三种“版本”

| 项目 | 表示什么 | 如何确认 |
| --- | --- | --- |
| 源码版本 | 当前 Git 提交及未提交修改 | `git log -1 --oneline`、`git status --short` |
| 本地构建 | 由当前配置编译出的程序 | `sdkconfig`、`build/config/sdkconfig.h`、`build/project_description.json` |
| 板上固件 | 设备此刻真正运行的程序 | 启动日志、`$INFO`、NORMAL 模式下的 `$CMD,GET_INFO` |

这三者可能不同。GitHub 已更新、电脑上 build 成功，都不代表板子已经更新。`IDF v5.5.4` 是开发框架版本，也不能单独证明固件控制模式正确。

2026-09-15 恢复项目时，**更新和重新构建之前**的本地记录是：`sdkconfig` 与 `build/config/sdkconfig.h` 均选择 `NORMAL + TRANSPARENT`，控制频率 100 Hz，用户零位采样 3000 ms，`GAIT_IMU_ENABLE=n`；旧 `build/project_description.json` 的项目版本为 `0ff87a3-dirty`，旧二进制修改时间为 2026-05-16。这仅是本地历史证据，未读取板上固件。

## 2. 准备 ESP-IDF 环境

在仓库根目录执行：

```powershell
Set-Location D:\Engineering\Machinecal\KneeExo
. .\tools\idf-shell.ps1
idf.py --version
```

预期版本为 `ESP-IDF v5.5.4`。本机安装入口是 `C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1`，IDF 源码位于 `D:\esp-idf\.espressif\v5.5.4\esp-idf`。脚本会复用已经激活的 IDF 环境；换电脑时可以指定自己的安装入口：

```powershell
. .\tools\idf-shell.ps1 -IdfProfile 'D:\Espressif\tools\your-idf-profile.ps1'
```

也可以设置当前终端的 `KNEEEXO_IDF_PROFILE`，或直接使用 Espressif 安装器提供的 ESP-IDF PowerShell 终端。`flash.ps1` 同样接受 `-IdfProfile`。这些示例中的安装路径需要按实际情况替换。新电脑安装和项目路径要求见 [Espressif Windows 入门文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/get-started/windows-start-project.html)。项目与 IDF 路径应避免空格。

不要用尚未激活的 Conda Python 直接替代 IDF 环境。本机普通终端的默认 Python、CMake、Ninja 曾分别来自 Conda/MSYS2；先激活环境，再检查 `idf.py --version`，可避免混用工具链。

## 3. 选择要编译的 profile

profile 是同一份工程的编译选项，不是需要寻找和下载的另一个“固件版本”。

| `flash.ps1` 参数 | App profile / Wear mode | 当前用途与注意点 |
| --- | --- | --- |
| `twai` | TWAI_LOOPBACK | 控制器自发自收 50 帧；不能代替真实 CAN 收发器、布线和电机通信验证 |
| `imu` | IMU_ONLY | 双 IMU 采样、串口遥测，不启动正常电机控制 |
| `motor` | MOTOR_TEST | CAN ID 扫描、位置参数读取；脚本关闭 AUTOSPIN 与自动限位搜索，不发送使能请求 |
| `normal` / `wear` / `transparent` | NORMAL / TRANSPARENT | 不额外叠加 Wear 速度助力或阻尼；仍运行状态机、状态阻抗与重力补偿路径 |
| `damp` / `damping` | NORMAL / DAMPING | 在普通控制上叠加速度反向阻尼；不等于助力 |
| `assist` / `follow` | NORMAL / ASSIST | 叠加很小的速度顺势力矩和近伸膝端缓冲；不是完整步态助力方案 |
| `gait` | NORMAL / TRANSPARENT，GAIT 配置开启 | 历史实验入口；当前状态估计代码没有由此标志统一开关 |
| `assistgait` | NORMAL / ASSIST，GAIT 配置开启 | 助力分支加上述实验配置，不能仅凭名字认为步态控制已验证 |
| `limit` / `calib` | MOTOR_TEST，自动寻找限位 | 会主动运动和触碰机械限位，仅适用于离体标定，不作为恢复调试入口 |

`assist` 的原有缺省顺势增益为 0.10 Nm/(rad/s)，速度死区 0.12 rad/s，顺势分支限幅 0.35 Nm；这是该分支的数值，不是全部输出力矩上限。低速或静止时无明显顺势助力是预期结果。硬件参数与标定值在 [`config.h`](../components/config/include/config.h)，编译模式与参数定义在 [`Kconfig.projbuild`](../components/config/Kconfig.projbuild)。

## 4. 只编译，不连接硬件

先编译电机通信诊断程序：

```powershell
.\tools\flash.ps1 motor
```

脚本在 `sdkconfig` 不存在时先执行 `idf.py reconfigure`，从 `sdkconfig.defaults` 和 Kconfig 缺省值生成配置。随后选择指定模式并 build，不带端口就不会烧录。

构建成功后检查脚本输出的 **App profile、Wear mode（若有）、GAIT 配置、实际 app version、二进制 SHA256**。也可直接查看：

```powershell
Select-String -Path sdkconfig -Pattern '^CONFIG_KNEEEXO_.*=y'
Select-String -Path build\config\sdkconfig.h -Pattern 'KNEEEXO_APP_PROFILE|KNEEEXO_WEAR_MODE'
Get-Content build\project_description.json -Raw | ConvertFrom-Json |
    Select-Object project_version, target, idf_path
Get-FileHash build\knee_exo.bin -Algorithm SHA256
```

`build/knee_exo.bin` 是应用镜像。平时使用 `idf.py flash` 写入工程生成的 bootloader、分区表和应用组合，不需要手动填写烧录地址。

## 5. 烧录和查看日志

1. 断开电机动力电源，将机构离体固定，连接板子的 USB 数据线。
2. 在设备管理器确认端口，也可运行 `python -m serial.tools.list_ports -v` 列出端口。
3. 关闭占用这个串口的仪表盘、绘图工具或其他 monitor。
4. 明确选择诊断模式后烧录：

```powershell
# 编译、烧录电机通信诊断程序，并打开日志
.\tools\flash.ps1 motor COM3

# 编译、烧录，不打开 monitor，方便之后用其他串口工具
.\tools\flash.ps1 motor COM3 flash
```

这两个命令是替代用法，选一个执行。`flash`（别名 `nomon`）表示 build + flash，`monitor` 表示 build + flash + monitor；`build` 即使提供端口也只编译。**不提供 Profile 会保留现有 `sdkconfig`，不是自动回到 normal；不提供 Port 不会记忆或使用上次端口。**

若当前使用正确的 IDF 环境，也可以按原生流程操作：

```powershell
idf.py menuconfig
# KneeExo Application -> App profile
# NORMAL 还要检查 Wear control mode
idf.py build
idf.py -p COM3 flash monitor
```

仅查看已经运行的程序并要求 monitor 不主动复位：

```powershell
idf.py -p COM3 monitor --no-reset
```

退出 monitor 用 `Ctrl+]`，释放端口。普通 monitor 默认会尝试复位设备；启动和快捷键行为参见 [Espressif IDF Monitor 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/api-guides/tools/idf-monitor.html)。

通信诊断通过、控制风险和标定问题处理完成后，才在台架上评估 NORMAL/ASSIST，例如：

```powershell
# 先只生成待评估的助力模式固件
.\tools\flash.ps1 assist
# 台架准备完毕后，由操作者执行烧录
.\tools\flash.ps1 assist COM3
```

## 6. 确认烧入的是哪一个程序

这次整理新增的固件在启动时打印 `$INFO`，包含项目版本、profile、wear mode、构建时间、IDF 版本等信息。NORMAL 程序也接受只读命令：

```text
$CMD,GET_INFO
$CMD,GET_PARAMS
```

发送到**设备串口终端**，每条以 LF 换行结束；不是 PowerShell 命令。前者确认固件身份，后者读取实际运行参数和控制开关。旧固件可能不认识 `GET_INFO`，这时检查原有启动日志中的 `Profile: ...` 与 `control loop ... mode=...`。

验证要点：

- `Profile: NORMAL` 只说明主程序类型；还要看到 `mode=ASSIST` 才能确认 Wear 助力分支已编译进去。
- 记录 `$INFO` 的版本与构建时间，同此次 build 输出对应；`-dirty` 表示构建时有未提交修改，不能只靠提交号复现。
- 构建脚本打印的是**整个 `.bin` 文件**的 SHA256；ESP-IDF 启动日志里的 ELF SHA256 是另一项标识，不能直接互相比较。
- `GET_PARAMS` 查看零位、`G/phi/bias`、`grav_en`、`state_ctrl` 等；随后结合 `$EXO` 和 `tq_cmd / grav / imp / motor_age / FAULT` 日志判断实际控制输出。

### 能看曲线，但按按钮没有反应

恢复时的本地配置以 **UART0、115200 baud 作为主控制台**，USB Serial/JTAG 只是第二日志输出口。`command_task` 使用 `getchar()` 读取主控制台，第二输出口能看日志不代表能收命令。

使用 USB-UART 桥对应的 COM 口进行命令交互；若希望原生 USB Serial/JTAG 也能交互，在 `menuconfig -> Component config -> ESP System Settings -> Channel for console output` 选择 `USB Serial/JTAG Controller`，重新编译烧录，并确认当前端口。不要误选 USB CDC/TinyUSB；这是不同的控制台实现。该限制在 [ESP-IDF 的控制台配置定义](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_system/Kconfig) 中有明确说明。

## 7. 避免旧配置、旧产物和旧 NVS 混在一起

### 配置文件分别管理什么

- `sdkconfig.defaults`：纳入 Git 的工程缺省配置，不覆盖已有 `sdkconfig` 的值。
- `sdkconfig`：本机实际构建配置，Git 忽略；换 profile、menuconfig 都会改它。
- `build/config/sdkconfig.h`：构建系统根据配置生成的 C/C++ 宏；检查成功构建最终采用了什么值，不要手工修改。
- `config.h`：源码内的接线、方向、限位、控制常量，修改后必须重新 build + flash。
- 板上 NVS：运行时保存的零位和辨识参数，普通重新编译或烧录不会清除它。

`sdkconfig.defaults` 的优先级及项目配置机制见 [ESP-IDF 构建系统文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/api-guides/build-system.html)。

### 重复构建与恢复配置

切换 profile 后正常 `idf.py build` 会重新生成所需配置，通常无需 `fullclean`。遇到迁移工程路径、切换 IDF、旧 CMake 缓存指向别处时，可以清理构建缓存再构建：

```powershell
idf.py fullclean
idf.py reconfigure
.\tools\flash.ps1 motor
```

`fullclean` 不等于恢复 `sdkconfig` 缺省。确实要从仓库缺省配置重新开始时，先把当前 `sdkconfig` 复制到自行命名的备份文件，再从配置入口重建；`idf.py set-target esp32s3` 会清理已有构建和重新初始化配置，不应作为每次开终端的固定操作。保存参数需要时可先使用 `idf.py save-defconfig` 导出，但不要不加检查地提交由本机实验配置产生的 defaults。

脚本切模式会保留仍然存在的自定义数值，只在相关键缺失时补入原有缺省值；Kconfig 可能移除不满足依赖的参数。切走又切回模式后，重新检查实际参数，不要假设上次实验值仍然存在。

### 为什么重烧后参数还像以前一样

NORMAL 会加载 NVS `exo` 命名空间中的电机零位、IMU 零位、重力参数和步态阈值。当前代码先做启动用户零位采样，再加载 NVS；如果之前保存过 `motor_zero`，它会覆盖刚采样得到的零位。这是现有实现的待修复点。

先用 `GET_PARAMS` 和 `runtime params:` 日志记录实际值，再按标定流程处理。不要用全片 `erase-flash` 当成默认排障步骤：它会连同保存参数一起清除。重新烧录正常应用通常保留 NVS；工程目前没有独立的“恢复参数默认值”串口命令。

## 8. 常见问题

| 现象 | 首先检查 |
| --- | --- |
| `idf.py` 找不到、报 Python 环境不存在 | 激活 v5.5.4 环境，检查 `IDF_PATH`、`IDF_PYTHON_ENV_PATH`；不要直接用系统 Python 猜安装路径 |
| 烧录连接超时 | 端口、数据线、USB 驱动和端口占用；自动下载不成功时按板卡说明使用 BOOT/RESET 进入下载模式 |
| 烧录高速不稳定 | 激活 IDF 后尝试 `idf.py -p COM3 -b 115200 flash` |
| 烧录成功后看不到日志 | 实际 console 通道和 COM 是否一致；复位/USB 重新枚举后端口可能改变 |
| PSRAM 初始化失败 | 是否确为 N16R8，Octal PSRAM 和硬件型号是否一致；不要通过盲目忽略 PSRAM 错误继续穿戴 |
| 只看到 `$IMU`，没有正常控制状态 | 检查是否烧了 IMU_ONLY；它不提供 NORMAL 主控制 |
| `motor` 能读位置但电机不动 | 此 profile 默认不使能，是正常诊断行为 |
| 显示 `NORMAL`，但感觉无助力 | 继续检查 Wear mode、实时扭矩、NVS 参数、传感器与标定；详见项目 [README](../README.md) 的诊断入口 |
| 仪表盘按钮没反应，但曲线正常 | 主控制台可能仍是 UART0，而连接的是仅输出日志的原生 USB 第二控制台 |

本页记录的是可复现的操作方法；此次仓库整理没有连接设备、烧录固件或执行电机动作。

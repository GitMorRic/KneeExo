# PC 工具与实验数据

[返回 README](../README.md) · [烧录完整步骤](../docs/flashing.md) · [串口协议](../docs/protocol.md)

## 1. 环境

固件使用ESP-IDF终端；PC界面建议用独立`.venv`，避免把Qt依赖装进IDF工具链：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools\requirements.txt
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
```

依赖范围用于安装兼容的大版本，尚不是完全锁定的发布环境。复现实验时记录实际`pip freeze`。以下用COM3举例，从项目根目录运行。

## 2. 工具选择

| 工具 | 用途 |
|---|---|
| `flash.ps1` | 切编译profile、build、可选flash/monitor；输出实际模式、构建版本、bin SHA256 |
| `idf-shell.ps1` | 复用/载入IDF环境，支持自定义安装profile |
| **`exo_dashboard_qt.py`** | 推荐主界面：normal遥测、参数、版本查询、实验导出 |
| `exo_dashboard.py` | 较早的Matplotlib综合仪表盘，供兼容/对照 |
| `exo_console.py` | Matplotlib四组EXO曲线；兼容旧17/18/20和当前21字段；CSV无type列 |
| `imu_plot.py` | IMU单项可视化，配合imu_only，默认显示IMU2 |
| `gravity_id_fit.py` | 读取带type列的连续CSV，筛选静态样本拟合重力模型 |
| `transparency_report.py` | 同格式CSV的反馈力矩RMS/峰值/速度相关性比较 |

## 3. 编译与烧录入口

```powershell
.\tools\flash.ps1 normal            # 只编译NORMAL+TRANSPARENT
.\tools\flash.ps1 assist            # 只编译NORMAL+ASSIST
.\tools\flash.ps1 motor COM3 flash   # 模块反馈测试，烧录后释放串口
```

不传profile保留当前sdkconfig；不传端口只编译；第三参数build即使带端口也只编译。normal/wear/transparent只关闭附加速度项，新版仍可能计算状态阻抗与重力。全部别名、初次配置、主控制台、NVS和故障说明见[烧录文档](../docs/flashing.md)。

## 4. Qt 仪表盘

退出IDF monitor（Ctrl+]）再打开：

```powershell
.\.venv\Scripts\python.exe tools\exo_dashboard_qt.py --port COM3 --csv logs\recovery.csv
```

- **Firmware info**：只读GET_INFO，新固件返回版本/profile/wear/构建时间；旧固件可能返回UNKNOWN。
- **Read params**：只读GET_PARAMS；检查零位、G参数、开关与阈值。响应显示在命令状态区。
- **Zero IMU / Zero Motor**：修改当前坐标零位；后者不是修改电机硬件零位。需要在明确标定姿态、离体验证条件下操作。
- **G sample / G fit / G send / G apply / G on/off**：采样、拟合、应用、开关补偿。应用参数会改变输出；G on可应用输入框值并开启，不能当成单纯状态查询。
- **State on/off / Auto FSM / Lock…**：控制状态阻抗和状态选择。State off只关状态项，其他项不一定为0。
- **SAFE on**：当前只强制SAFE状态并退出手动模式，**不等于停机/断电/零力矩**。
- **手动力矩/位置**：离体实验控制；Manual off会恢复自动计算，不能当停机使用。
- **Save params**：保存零位、重力和阈值到NVS，普通重烧后保留。先查[零位覆盖问题](../docs/troubleshooting.md)。

默认会丢弃超过8192字节的串口积压以恢复实时显示；控制状态区会提示。记录完整性需要自行检查。可用`--drop-backlog-bytes 0`关闭主动丢积压，但PC处理不足时显示会滞后。`--smooth`只平滑界面，不修改固件控制。

IMU2原始pitch曲线与固件校正后的shank_deg应区分；命令力矩tau_cmd与反馈tau_fb也要分别观察。

## 5. 实验导出和离线分析

Qt中选择实验类型，按住录制按钮（或R/Space），松开后导出CSV、PNG、摘要到logs/experiments/。界面中的实验动作提示只是数据采集模板，执行前必须满足项目现状文档中的台架条件。

有三种不同CSV：

| 来源 | 结构 | 可直接用于下面两个分析脚本 |
|---|---|---|
| Qt `--csv logs/run.csv` | 连续IMU/EXO混合流，含type列 | 是 |
| Qt按住录制导出 | 实验片段，有t_rel，无type=EXO过滤列 | 否，需显式转换 |
| `exo_console.py --csv` | EXO列，无type | 否，需显式转换 |

```powershell
.\.venv\Scripts\python.exe tools\gravity_id_fit.py --csv logs\recovery.csv
.\.venv\Scripts\python.exe tools\transparency_report.py --csv logs\recovery.csv
.\.venv\Scripts\python.exe tools\transparency_report.py --before logs\gravity_off.csv --after logs\gravity_on.csv
```

重力拟合需有足够的静态、多角度样本，且要明确当时的机械约束和力矩来源。默认固定phi=0，`--free-phase`放开相位。拟合出的模型为关节坐标的**有符号补偿力矩**`G*sin(shank_rad+phi)+bias`，须离体核验方向，不能因为拟合成功就直接开启穿戴补偿。

透明度报告描述的是驱动器估计力矩及与速度的关系，不能单独证明人体交互力、肌肉负担或助力收益。实验数据默认被Git忽略；提交结论时附可复核的配置、参数和统计，不把本机日志自动上传。

## 6. IMU 单项绘图

```powershell
.\tools\flash.ps1 imu COM3 flash
.\.venv\Scripts\python.exe tools\imu_plot.py --port COM3 --channel 2
```

默认选择IMU2，`--channel 1`用于IMU1；兼容旧单通道$IMU帧。两通道不会混入同一组曲线。同一时刻只能有一个程序打开COM口。新烧录的imu模式不执行电机助力；完成诊断后需按需要重新构建normal/assist。

# 测试与台架验证

[返回 README](../README.md)

## 主机离线回归

从项目根目录运行；仅用Python标准库，不需串口、Qt或电机：

```powershell
python -B -m unittest discover -s tests -p "test_*.py" -v
```

`test_exo_console.py`覆盖当前21字段EXO帧、历史17/18/20字段、NaN反馈、CSV字段保留和损坏输入。它从真实工具源码提取解析函数进行测试，不运行GUI或串口线程。

`test_imu_plot.py`覆盖IMU1/IMU2和旧单通道帧、默认小腿通道选择、交错流隔离与畸形输入。两组共10项离线回归测试。

## 固件构建检查

```powershell
.\tools\flash.ps1 twai
.\tools\flash.ps1 imu
.\tools\flash.ps1 motor
.\tools\flash.ps1 assist
.\tools\flash.ps1 normal
```

不带端口不会烧录。以上覆盖4种应用profile及normal的ASSIST编译分支；最后回到NORMAL+TRANSPARENT。构建验证不代表硬件或穿戴控制通过。

### 2026-09-15 本次验证记录

- ESP-IDF v5.5.4实际编译通过：NORMAL/TRANSPARENT、NORMAL/ASSIST、TWAI_LOOPBACK、IMU_ONLY、MOTOR_TEST；最后恢复本地NORMAL/TRANSPARENT配置并构建。
- 10项Python离线回归测试通过；全部Python工具与测试的语法检查通过。
- PowerShell 5/7语法检查及9组隔离mock-idf场景通过，覆盖配置初始化、模式切换、参数传递、失败传播和目录恢复。
- Qt使用无窗口平台和模拟串口检查：两个只读查询按钮发送正确命令，完整参数响应保留；未打开硬件串口。
- 新文档文件链接与Git diff格式检查通过。本次没有烧录、电机动作或穿戴测试，也没有验证P0控制整改。

## 硬件验证

优先使用现有profile，不要为单模块测试覆盖main/app_main.cpp：

| 目的 | 入口 | 验证范围 |
|---|---|---|
| TWAI控制器自环 | `flash.ps1 twai COMx` | 控制器自收发；不验证收发器/外部线束/电机 |
| IMU原始帧 | `flash.ps1 imu COMx` | 两路UART，重点IMU2；观察方向/更新情况 |
| RS02参数反馈 | `flash.ps1 motor COMx` | 默认关闭自动运动；确认ID、通信、机械位置 |
| 自动寻找机械限位 | `flash.ps1 limit COMx` | **会主动运动并接触机械限位，仅适用于已准备好的无人离体台架** |

历史片段esp_core/test_twai_loopback.cpp、motor_rs02/test_motor_enable.cpp、imu_witmotion/test_imu_rx.cpp不是独立可构建工程；部分会使能电机，只作参考。目前没有覆盖FSM、安全输出不变量、驱动协议、NVS迁移的完整自动化套件。

恢复有源实验前，按[项目现状P0清单](../docs/project-status.md)先补齐并验证超时、故障、停止和限位策略。测试中保留构建版本、实际profile、NVS参数、装配标定、输入场景和最终输出曲线。

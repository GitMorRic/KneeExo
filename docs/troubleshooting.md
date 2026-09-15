# 穿上没有助力：如何定位

[返回 README](../README.md) · [烧录](flashing.md) · [架构](architecture.md)

## 1. 目前可以确定什么

2026-09-15 恢复项目时，源码最新已有提交为 `d6b3a68`（2026-05-16）；远程 main 为 `0ff87a3`（2026-05-12）。本地旧 `build/project_description.json` 记录 `0ff87a3-dirty`，`sdkconfig` 及旧构建均为 NORMAL + TRANSPARENT。`dirty` 表示构建时有源码改动，不能把它当成纯净的 `0ff87a3` 发布版。

**目前没有读取板上固件或实时串口，不能断言烧错了版本。** 同一份源码能构建出不同 profile，而相同二进制还可能使用不同 NVS 参数。排查须同时记录这三层。

现有 Qt 上位机没有统一的电机 Enable 按钮；`normal` 在启动零位采样后自动调用 `rs02_enable()`。不要按其他项目的 STANDBY/Enable 流程解释这里的状态。

## 2. 先区分“没生成力矩”和“生成了但没感觉”

在离体、可切断电机电源的台架上检查。当前 SAFE 不是急停，断 IMU/断 CAN 测试不能在人身上做。

| 观察项 | 现象 | 对应检查 |
|---|---|---|
| 启动 profile | TWAI_LOOPBACK / IMU_ONLY / MOTOR_TEST | 这些是分模块测试，不是膝关节助力主程序 |
| `wear_mode` | TRANSPARENT | 不叠加速度跟随；新版仍有状态阻抗和重力项 |
| `$IMU2` / `shank_deg` | 无数据或摆动不变 | IMU2 接线、9600 波特率、安装方向、是否只接了 IMU1 |
| `motor_ok` / `motor_age_ms` | 0 / 大于等于 120 ms / NaN | CAN 供电、共地、收发器、CANH/L、终端、电机 ID 0x02 |
| `state` | 长期 SAFE | 静止未满足进入条件；IMU/电机无效；强制 SAFE；角度/速度超阈值 |
| `tau_g` | 总为 0 | 补偿关闭，或 G/bias 为零，或当前角度恰好使模型接近零 |
| `tau_imp` | 接近 0 | `state_ctrl=0`、SAFE 小阻尼、角度接近平衡点；先看状态 |
| `tau_cmd` | 接近 0 | 上述各项、软限位、力矩抵消、斜率限制；不是烧录失败的直接证据 |
| `tau_cmd` 非零但 `tau_fb` 很小 | 命令到反馈链路不符 | 核对电机实际运行模式/故障，CAN 发包结果和方向；当前遥测尚不全 |
| `tau_cmd`、`tau_fb` 都是几十分之一 Nm | 有输出但体感小 | 当前参数本来很小，机构摩擦/自重/佩戴传力可能盖过输出 |

`motor_ok=1` 只表示近期收到反馈，不保证驱动器已进入正确运行模式；`tau_fb` 是驱动器估计力矩，不是人体与外骨骼之间的力矩传感器测量值。

## 3. 读取版本和真实参数

1. 保留启动日志里的 `Project version`、`Profile:` 和 `control loop ... mode=`。
2. 本次之后构建的固件在启动时打印 `$INFO`；normal 主程序还支持 `$CMD,GET_INFO`。Qt 中点击 **Firmware info** 可查询。
3. 点击 **Read params**，或通过主控制台发送下列单行命令（以换行结束）：

```text
$CMD,GET_INFO
$CMD,GET_PARAMS
```

`GET_PARAMS` 中重点核对：`motor_zero`、`imu_zero`、`G/phi/bias`、`grav_en`、`state_ctrl`、`state_lock`、`impact`、`swing`。本次新增 `force_safe` 和 `manual_mode`，不改变控制。

命令无回应时，先看[主/次控制台说明](flashing.md)：本机原配置主控制台是 UART0，原生 USB Serial/JTAG 只是次输出。能看日志不等于能发送命令。旧固件返回 `$ERR,UNKNOWN,GET_INFO` 也是有效线索，不应通过反复换电机参数解决。

## 4. 当前哪些参数真的影响助力

| 想改变的行为 | 实际修改入口 | 注意 |
|---|---|---|
| 选择主程序或测试程序 | `tools/flash.ps1` / Kconfig App profile | `normal` 别名明确选择 transparent；不传 profile 才保留已有配置 |
| 添加实验速度跟随 | `assist` profile 别名；`KNEEEXO_ASSIST_*` | `B=0.10`，分量上限0.35 Nm，速度死区0.12 rad/s；不是静止支撑 |
| 状态机的平衡角、刚度、阻尼、分量限幅 | `main/control_task.cpp` 的 `params_for_state()` | 这些是当前主要状态阻抗参数；不是 `KNEEEXO_DEFAULT_KP/KD` |
| 步态进入条件/时序 | `update_features()`、`update_state_machine()`；运行 `SET_THRESH` | `GAIT_IMU_ENABLE` 当前不能关闭这个 FSM；用数据验证阈值 |
| 抵消机构重力 | `config.h` 的 `SHANK_GRAVITY_*` 或运行 `SET_GRAV` / `GRAV_ENABLE` | NVS 可覆盖编译默认值；拟合的是有符号补偿力矩 |
| 电机方向/机械零位/机械限位 | `config.h` 的 `JOINT_DIR_SIGN`、`JOINT_ZERO_OFFSET_RAD`、`KNEE_*_LIMIT_RAD` | 只针对已标定机构，不能复制到另一条腿；与用户零位不同 |
| 小腿姿态轴及符号 | `IMU_SHANK_PITCH_AXIS/SIGN` | 以实际安装测试：小腿前摆为正；更换安装须重新标定 |

`assist` 默认在 1 rad/s 膝角速度下仅增加约 0.10 Nm。接近伸膝端且快速屈膝时，还会叠加反向缓冲。各分量不是总输出上限；当前缺少统一的低力矩总限幅，因此不建议靠增大这些数字来尝试恢复助力。

## 5. 零位和 NVS 为什么会导致“重烧没变化”

- 开机用户零位：机械关节坐标的平均值，用于把自然伸膝作为当次 0°。
- 机械零位：`config.h` 固定的电机位置偏置，用于机械限位判断。
- IMU 零位：小腿姿态偏置，影响步态阈值和重力模型。
- `SAVE_PARAMS` 将用户/IMU零位、重力参数、重力开关和步态阈值保存到 NVS `exo` 命名空间。
- **已发现的缺陷**：开机先采样用户零位，随后控制任务读取旧 NVS `motor_zero`，可能把刚采样的零位覆盖。`GET_PARAMS` 反映最终生效值，应与 `USER ZERO done` 对比。
- 普通 flash 不等于整片擦除。不要为“试试”执行 `erase-flash`，它会丢失标定；应先记录参数，并优先修复加载顺序。

## 6. 从恢复到验证的顺序

1. 保存现有启动信息、当前参数以及短时间遥测，确认实际主控制台。
2. 对照[规格](specs.md)核对 IMU2、CAN 引脚、ID、机械方向与供电。
3. 按[烧录文档](flashing.md)重新构建所需 profile；先只编译，再做离体模块检查。
4. 先完成[改进清单](project-status.md)中的输出禁止、超时、限位和零位优先级整改及台架验收。
5. 用同步曲线核对 `state / shank_deg / knee_rad / tau_g / tau_imp / tau_cmd / tau_fb`，再决定是状态识别、重力辨识还是机构阻力的问题。
6. 在控制输出范围、方向和故障处理均验证后，再设计穿戴效果试验。当前仓库没有提供已经验证的站立承重、上下楼或康复助力配置。

历史的 `fsm_swing` 实验摘要记录过约0.316 Nm命令 RMS、0.346 Nm反馈 RMS，说明此前有低幅输出；该数据不能代替目前设备的检查或人体助力效果验证。

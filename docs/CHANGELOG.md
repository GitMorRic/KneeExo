# Changelog

## [Unreleased] - 2026-09-15

### Added

- 新增根README、架构、项目现状与P0/P1/P2改进清单、无助力排查、构建烧录、串口协议说明；重写工具、规格与测试入口，标明PRD是目标。
- 固件启动输出`$INFO`；normal支持只读`$CMD,GET_INFO`，报告应用版本、profile、wear模式、构建时间和IDF版本。Qt新增Firmware info / Read params按钮；GET_PARAMS补充force_safe/manual_mode。
- 增加PC工具依赖清单与遥测解析离线回归测试。

### Fixed

- 旧EXO控制台接受当前21字段帧并保存impact_g，兼容17/18/20字段；损坏数值帧不再使读取线程退出。
- IMU绘图支持当前`$IMU1/$IMU2`帧并选择通道，默认小腿IMU2，兼容旧`$IMU`帧。
- 烧录脚本从工程根运行，缺sdkconfig时先reconfigure，缺失配置键正确插入，校验动作并传播build/flash失败；成功编译后打印实际模式、版本和镜像SHA256。
- IDF环境脚本支持已有环境和自定义安装profile；纠正transparent/GAIT开关含义、ADC分压与重力符号旧说明。

### Scope

- 本次未改变力矩计算、助力增益或自动使能策略。SAFE不等于停机、IMU缓存过期检测缺失、NVS零位覆盖等问题已列出，但尚待独立控制整改和离体验证。

## 2026-05 开发记录补录

以下按Git历史补记，不补造已发布版本号或硬件验收结论：

- `f336c49`（05-11）：更新外骨骼状态机与显示上位机。
- `06250f1` / `0ff87a3`（05-12）：完善上位机并使用PyQtGraph。
- `d6b3a68`（05-16）：电池与电机反馈新鲜度遥测、手动力矩/位置实验命令、实验CSV/PNG/摘要采集及图示，忽略本地实验导出。

下面保留旧版本记录，其中“transparent=0力矩”“gait不参与控制”等描述只代表当时设计；当前行为见README和架构文档。

## [0.2.1] - 2026-04-28

### Added

- **悬空机械限位标定模式**：新增 `KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB`，可通过
  `.\tools\flash.ps1 limit COMx` 启用。该模式仅限离体悬空机构使用，电机会低速向负/正两个方向
  搜索机械限位，用力矩升高、速度停滞、位置变化很小、持续时间共同判定碰限位，并打印
  `JOINT_DIR_SIGN`、`JOINT_ZERO_OFFSET_RAD`、`KNEE_EXT_LIMIT_RAD`、`KNEE_FLEX_LIMIT_RAD`
  的两组候选写法。
- `Kconfig.projbuild` 新增限位标定参数：`KNEEEXO_LIMIT_CALIB_TORQUE_X100`、
  `KNEEEXO_LIMIT_CALIB_SPEED_MRAD_S`、`KNEEEXO_LIMIT_CALIB_MAX_TRAVEL_MRAD`、
  `KNEEEXO_LIMIT_CALIB_HOLD_MS`，方便根据机构重量、摩擦和限位强度调整。
- 限位标定检测逻辑调整：碰限位后不再只依赖“速度/位置完全停滞”，新增命令跟随误差判断，
  并在未严格判定成功时保留最大力矩处的候选限位，最后统一打印正/负限位候选值和
  `config.h` 填写提示。
- **穿戴调试模式**：`normal` profile 新增运行时用户伸膝零位平均、透明模式和小阻尼模式。
  `.\tools\flash.ps1 wear COMx` 进入 0 力矩透明模式，`.\tools\flash.ps1 damp COMx`
  进入速度阻尼模式；两者均在电机使能前采样 `USER_ZERO`，并在控制循环中打印
  `joint_user` / `joint_mech` / 命令力矩 / 反馈力矩。
- **实验助力跟随模式**：新增 `.\tools\flash.ps1 assist COMx`，基于电机编码器速度给小幅顺势力矩，
  并在接近伸膝端且检测到快速屈膝时叠加落地缓冲阻尼。该模式暂不依赖 IMU，所有力矩均受
  软限位和力矩夹紧保护。
- **IMU 步态识别占位**：新增 `KNEEEXO_GAIT_IMU_ENABLE` 实验开关和 IMU thigh pitch/rate
  相位估计函数，默认关闭且不参与力矩控制，为后续 stance/swing 状态机预留。
- **IMU 步态显示与落地尖刺检测**：新增 `.\tools\flash.ps1 gait COMx` 和
  `.\tools\flash.ps1 assistgait COMx`，在日志中显示 `gait=stance/swing/landing`、
  `acc_norm` 与 `impact`。`landing` 由加速度模长高通尖刺 + 冷却时间检测，先只显示，
  不参与助力控制。
- 优化 `landing` 可见性：检测到落地尖刺时即时打印 `GAIT landing spike`，并将
  `landing` 状态保持约 250ms，避免 100Hz 瞬时事件被 0.5s 周期日志错过；默认尖刺阈值
  从 0.35g 降到 0.18g，便于先观察事件。
- 调低实验助力默认值：`assist` 速度跟随增益从 0.20 降到 0.10 Nm/(rad/s)，
  助力力矩上限从 0.60 降到 0.35 Nm，落地缓冲上限从 1.20 降到 0.80 Nm，
  减少滞后导致的过冲和“幅度太大”体感。

### Fixed

- **IMU 引脚冲突 (GPIO41/42 → GPIO17/18)**：GPIO41 (JTAG MTDI) / GPIO42 (JTAG MTMS) 是
  ESP32-S3 经典 JTAG 接口的专用引脚。ESP-IDF 固件启动时即使只用板载 USB-JTAG（GPIO19/20），
  仍会把 GPIO39–42 预配置为 JTAG，导致 `uart: GPIO 42 is not usable, maybe used by others` 警告，
  UART1 TX 无法正常发送，`raw_uart_probe` 返回 0 bytes。改用 GPIO17（TX）/ GPIO18（RX）后正常。
- **IMU 波特率不匹配 (115200 → 9600)**：WitMotion WT-IMU63 出厂默认波特率为 9600 bps，非 115200。
  `raw_uart_probe` 在 9600 baud 下检测到 `has 0x55`，在 115200 下字节乱码（first=0x00）；
  将 `IMU_UART_BAUD` 改为 9600 后 CSV 数据流正常。
- **IMU 5V 供电导致 UART 数据乱码**：WT-IMU63 用 5V 供电时，UART TX 输出为 5V 逻辑电平；
  ESP32-S3 GPIO 最大输入电压 3.6V，高于此值时逻辑判断出错（字节被接收但 0x55 找不到）。
  已改为 3.3V 供电。若需 5V 供电见下方 Known Issues。

### Changed

- `config.h` IMU 引脚更新：`PIN_IMU1_MCU_TX = GPIO17`，`PIN_IMU1_MCU_RX = GPIO18`；
  `PIN_IMU2_MCU_TX = GPIO21`，`PIN_IMU2_MCU_RX = GPIO38`（同步规避 GPIO48 RGB LED 冲突）
- `config.h` `IMU_UART_BAUD` 从 115200 改为 9600，注释说明出厂默认值及可上调路径

### Known Issues

- **IMU 5V 供电 + ESP32-S3 的电平兼容问题**：若需 5V 供电（指示灯更亮），需在
  IMU TX → GPIO18（ESP32 RX）之间加分压电路：10kΩ 串联 + 20kΩ 对地，将 5V 降至 3.3V。
  ESP32 TX (3.3V) → IMU RX 方向无需处理（5V TTL 门限 VIH=2.0V，3.3V 满足）。

---

## [0.2.0] - 2026-04-26

### Added

- **App profile 切换机制**：`Kconfig.projbuild` 加 `KNEEEXO_APP_PROFILE` 4 选 1
  (`normal` / `twai_loopback` / `imu_only` / `motor_test`)，`main/app_main.cpp`
  改为 dispatcher，4 个 profile 拆到 `main/profiles/profile_*.cpp`，每个文件
  用 `#if CONFIG_KNEEEXO_APP_PROFILE_xxx` 包整文件，未选中时空 TU。
- `main/profiles/profile_normal.cpp`：原 main 的形态（IMU + RS02 + 100Hz 控制循环），
  全局 `g_motor` 句柄定义在此。
- `main/profiles/profile_twai_loopback.cpp`：TWAI 自发自收 50 帧，验证 CAN 控制器。
- `main/profiles/profile_imu_only.cpp`：仅 IMU，按 `KNEEEXO_IMU_CSV_HZ` 输出 CSV
  (`$IMU,t_us,ax,ay,az,gx,gy,gz,roll,pitch,yaw,T`)，配合 PC 端实时绘图。
- `main/profiles/profile_motor_test.cpp`：阶段 A 只读反馈 / 阶段 B（需手动开启
  `KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN`）±0.1rad 摆动，用于方向标定。
- `tools/idf-shell.ps1`：dot-source 一键拉起 ESP-IDF v5.5.4 环境。
- `tools/imu_plot.py`：PC 端 pyserial + matplotlib 实时绘图，配合 imu_only profile。
- `tools/README.md`：上述脚本使用说明。
- `config.h` 新增**关节 / IMU 安装方向标定常量** + `motor_to_joint_*`/`joint_to_motor_*`
  辅助函数，统一上层只看膝关节坐标 (屈曲为正)。
  - `JOINT_DIR_SIGN`、`JOINT_ZERO_OFFSET_RAD`、`KNEE_FLEX_LIMIT_RAD`、`KNEE_EXT_LIMIT_RAD`
  - `IMU_THIGH_PITCH_AXIS`、`IMU_THIGH_PITCH_SIGN`
  - 当前为占位默认值，待 motor_test profile 实测后改实。

### Changed

- `control_task.cpp` 改为通过 `joint_to_motor_*` 在关节 / 电机坐标之间换算，
  日志同时打印 `thigh_pitch` 与 `joint pos (rad / deg)`。
- `main/CMakeLists.txt` 加入 4 个 profile 源文件。
- 验证 ESP-IDF v5.5.4 环境（安装路径 `D:\esp-idf\.espressif\v5.5.4\esp-idf`）：
  4 个 profile 全部 `idf.py build` 通过。

## [0.1.1] - 2026-04-20

### Fixed

- `config.h` ADC 通道注释错误：GPIO1 = ADC1_CH0，非 CH3（ADC1_CH3 = GPIO4）
- `config.h` 注释补全：GPIO41/42 为 JTAG MTDI/MTMS 引脚警告；GPIO48 为 DevKitC-1 板载 RGB LED 警告；GPIO15/16 为 32kHz 晶振引脚（无外接晶振时可用）说明
- `config.h` IMU section 注释更新至实际 GPIO（42/41/48/47），删除过时的 GPIO17/18 注释
- `config.h` `IMU_BAUD` 重命名为 `IMU_UART_BAUD` 更明确；`RS02_CAN_ID/RS02_HOST_ID` 类型由 `uint16_t` 改为 `uint8_t`（与驱动 API 一致）

### Changed

- `rs02_motor.c` → `rs02_motor.cpp`：`config.h` 引入 C++ `namespace ExoConfig`，`.c` 文件无法编译，改为 `.cpp` 并添加 `using namespace ExoConfig;`
- `rs02_motor/CMakeLists.txt` 对应更新 SRCS 文件名
- `app_main.cpp` 全局 `using namespace ExoConfig;`，所有引脚/波特率常量加命名空间前缀；IMU 初始化由旧的 `IMU_UART_NUM / PIN_IMU_UART_TX/RX / IMU_BAUD` 更新为 `IMU1_UART_NUM / PIN_IMU1_MCU_TX/RX / IMU_UART_BAUD`
- `docs/skills/skill_esp32s3_core/SKILL.md` GPIO 分配表扩充，增加方向列、完整引脚列表、JTAG 和 RGB LED 警告说明

## [0.1.0] - 2026-04-20

### Added

- 搭建 ESP-IDF v5.5.4 工程骨架（根 `CMakeLists.txt`、`sdkconfig.defaults`、`partitions.csv`、`main/` 入口）
- 新增 `components/drivers/can_bus/`：薄封装 TWAI (CAN 2.0B)，1Mbps，共享给上层驱动
- 新增 `components/drivers/rs02_motor/`：Robstride RS02 私有协议驱动
  - 使能 / 停止 / 清故障 / 设置零位 / 模式切换 (0x7005)
  - 运控模式控制帧 (通信类型 1)，支持 torque_ff / pos / vel / kp / kd
  - 参数读 / 写 (通信类型 17 / 18)
  - 后台 `rx_task` 自动解析反馈帧 (类型 2) 与故障帧 (类型 21)
- 新增 `components/drivers/witmotion_imu/`：JY901 兼容的 0x55 帧解析
  - 支持 0x51 加速度 / 0x52 角速度 / 0x53 欧拉角 / 0x59 四元数
  - 带校验和、失同步移位恢复
  - mutex 保护的最新数据结构
- `main/control_task.cpp`：100Hz 控制循环 demo，每 0.5s 打印 IMU + 电机反馈
- 完善文档：`specs.md`、`system_prd.md`、三个 SKILL.md 全部补内容
- 增加 `Kconfig.projbuild` 暴露控制周期和默认 Kp/Kd

### Changed

- `components/config/include/config.h`
  - 修正 ESP32-S3 不存在的 GPIO22：TWAI 改为 `TX=GPIO5, RX=GPIO6`
  - 细化电机物理量限幅 (`MOTOR_MIN_POS/KP/KD` 等)
  - 增加 `CAN_BITRATE_HZ = 1_000_000`
- 将 `components/config/Kconfig` 替换为 `Kconfig.projbuild` 以在根 menuconfig 显示

### Known Issues

- `idf.py build` 未本地验证（需要在装有 ESP-IDF 的环境执行），首次编译如有小改动请按日志修正
- 急停按键硬件未集成
- 电池电压曲线校准未做
- 步态相位识别算法未实现

## [0.0.0] - 2026-04-20

### Added

- 新建了整个项目
- 新增 esp32s3 开发板、RS02 电机、维特智能 witmotion_imu 的资料并建立对应的 skills

# 参考资料

1. [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)

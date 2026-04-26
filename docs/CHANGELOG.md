# Changelog

## [0.2.1] - 2026-04-26

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

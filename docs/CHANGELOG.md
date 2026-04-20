# Changelog

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

---
name: skill-witmotion-imu-6axis
description: |
  WitMotion WT-IMU63 六轴 IMU 串口驱动速查（兼容 JY901 标准协议）。
version: "1.0.0"
metadata:
  author: Ricky
  last_updated: "2026-04-20"
  project: knee-exoskeleton
  dependencies: ["config"]
---

# WitMotion WT-IMU63 Skill

## 1. 模块概况

- 六轴 IMU，带板载姿态解算，输出加速度 / 角速度 / 欧拉角 / 四元数
- 默认串口 115200，8N1；默认 10ms 上报一次
- 支持 UART 或 I²C，本项目用 **UART**
- 规格书：<https://wit-motion.yuque.com/wumwnr/bf4d0f/meggs4ct2gthmtda>
- 示例代码（与 JY901 同协议）：<https://github.com/WITMOTION/WitStandardProtocol_JY901>

## 2. 本项目接法 (交叉)

```
MCU (ESP32-S3)     WT-IMU63
  GPIO17 TX   ----->  RX
  GPIO18 RX   <-----  TX
  3V3         -----> VCC (3.3~5V)
  GND         -----> GND
```

对应宏：
- `PIN_IMU1_MCU_TX = GPIO_NUM_42`（IMU1，UART1）
- `PIN_IMU1_MCU_RX = GPIO_NUM_41`
- `PIN_IMU2_MCU_TX = GPIO_NUM_48`（IMU2，UART2；DevKitC-1 冲突 RGB_LED，建议改 GPIO17）
- `PIN_IMU2_MCU_RX = GPIO_NUM_47`
- `IMU_UART_NUM = UART_NUM_1`
- `IMU_BAUD = 115200`

接线参考图：`datasheets/串口接线图.png`

## 3. 帧格式（标准协议）

每帧**固定 11 字节**：

```
[0]   0x55           帧头
[1]   kind           帧类型（0x51/0x52/0x53/0x59 等）
[2-9] 8 bytes data   数据区
[10]  checksum       前 10 字节相加的低 8 位
```

## 4. 数据解析（小端 int16）

| kind | 含义 | Byte[2:3] | Byte[4:5] | Byte[6:7] | Byte[8:9] | 单位换算 |
|---|---|---|---|---|---|---|
| **0x51** | 加速度 | Ax | Ay | Az | T | acc = (int16)/32768 × 16 (g)；T = (int16)/100 (°C) |
| **0x52** | 角速度 | Gx | Gy | Gz | T | gyro = (int16)/32768 × 2000 (°/s) |
| **0x53** | 角度 | Roll | Pitch | Yaw | Ver | angle = (int16)/32768 × 180 (°) |
| 0x54 | 磁场 | Hx | Hy | Hz | T | 原始 LSB |
| **0x59** | 四元数 | q0 | q1 | q2 | q3 | q = (int16)/32768 |

公式示例 (C)：

```c
int16_t raw = (int16_t)(buf[L] | (buf[H] << 8));
float acc_g = raw / 32768.0f * 16.0f;
```

## 5. 解析状态机要点

1. 字节流里找 `0x55`
2. 读够 11 字节，算 checksum
3. 校验失败 → **丢一个字节重新找 `0x55`**（不能整帧丢，否则多数据包边界错位后恢复慢）
4. 校验通过 → 按 kind 分发解码
5. 用 mutex 保护共享 `witmotion_data_t`

## 6. 驱动入口

- 头：[`components/drivers/witmotion_imu/include/witmotion_imu.h`](../../../components/drivers/witmotion_imu/include/witmotion_imu.h)
- 实现：[`components/drivers/witmotion_imu/witmotion_imu.c`](../../../components/drivers/witmotion_imu/witmotion_imu.c)

```c
witmotion_init(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 115200);
witmotion_data_t d;
witmotion_get_latest(&d);
// d.euler_deg[1] 即 pitch（°）
```

## 7. 上电配置建议

- 默认输出 `0x51 0x52 0x53` 三帧，这是本项目最需要的
- 若要 `0x59` 四元数：通过上位机 (SUT) 或发送配置命令把输出带宽打开
- 输出速率：10Hz~200Hz 可调，外骨骼控制建议 100Hz 或 200Hz

## 8. 典型坐标系

WT-IMU63 默认右手系，X 前、Y 右、Z 上（以 PCB 印字方向为准）。安装到大腿外侧时，**pitch 的正方向一般代表膝关节屈曲方向**，按实际安装调整。

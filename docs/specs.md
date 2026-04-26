# System Specifications

## 1. 硬件清单 (BOM v0.1)

| 组件 | 型号 | 数量 | 说明 |
|---|---|---|---|
| 主控 | ESP32-S3-N16R8 开发板 | 1 | 16MB Flash / 8MB Octal PSRAM |
| 关节电机 | Robstride RS02 | 1 (首版单侧) | 17Nm 峰值，CAN 1Mbps |
| CAN 收发器 | SN65HVD230 / TJA1050 类 | 1 | TTL <-> CAN 差分，3.3V 版 |
| IMU | WitMotion WT-IMU63 | 1 | 6 轴，UART 115200，板载姿态解算 |
| 电池 | 48V 锂电池包 | 1 | RS02 额定 48VDC / 24~60V 可用 |
| DC-DC | 48V -> 5V / 3.3V | 1 | 主控供电；建议独立地与主电源共地 |
| 连接器 | XT30PB(2+2) (电机侧板端) | 1 | AMASS 品牌，2 根电源 + 2 根 CAN |

后续扩展双侧膝关节时：RS02 / IMU / 线束各翻一倍，主控不变（同一 CAN 总线挂两个电机即可，ID 不同）。

## 2. 软件与开发环境

| 项目 | 版本 |
|---|---|
| 固件框架 | ESP-IDF v5.5.4 |
| 目标 | esp32s3 |
| 编译器 | 随 IDF 自带 toolchain |
| 主语言 | C (驱动) + C++ (main) |
| 构建 | `idf.py set-target esp32s3` → `idf.py build flash monitor` |

## 3. 电源与功耗

### 3.1 电源拓扑

```
24V Battery ──┬─► RS02 电机 (Vin 24~60V，峰值相电流 23Apk)
              │
              └─► 24V→5V DC-DC ──► 24V→3.3V DC-DC ──► ESP32-S3 + IMU + CAN 收发器
```

### 3.2 电池电压采样

- 采样点：电池正端经 **10kΩ + 1kΩ** 分压到 ADC1_CH3 (GPIO4)
- 分压比 `BATTERY_DIVIDER_RATIO = 11.0f`（对应 48V 时 ADC 见 ~4.36V）
- 注意：分压电阻下端到地务必短且抗扰；ADC 需配合校准曲线读出精确电压

### 3.3 估算功耗

| 状态 | 典型电流 @24V |
|---|---|
| 待机 (驱动器 standby) | ≤18 mA |
| 空载旋转 | ~0.5 Arms |
| 额定 6 Nm | 7 Apk |
| 峰值 17 Nm | 23 Apk 短时 |
| MCU + IMU + 收发器 | ~150 mA @5V，约 20 mA @48V (含 DC-DC 效率) |

续航目标：4S~13S 锂电包，10Ah 容量下正常步行辅助 1~2 小时。

## 4. 物理性能

| 指标 | 值 |
|---|---|
| 电机重量 | 405 g ± 3 g |
| 电机减速比 | 7.75 : 1 |
| 关节峰值力矩 | 17 Nm（瞬时） |
| 连续工作力矩 | 6 Nm（散热板 260×280mm 下） |
| 关节最大速度 | 44 rad/s |
| 编码器分辨率 | 14 bit 单圈绝对 |

## 5. 控制环规划

- **控制周期**：10ms (100Hz) —— `CONFIG_KNEEEXO_CONTROL_HZ`
- **控制模式**：运控模式（t_ref = Kd·Δv + Kp·Δp + τ_ff）
- **上层带宽**：IMU 默认 100Hz，与控制周期同频
- **安全机制**：看门狗 5s、`rs02_stop(clear_fault=true)` 异常退出，急停按键（待加）

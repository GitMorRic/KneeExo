---
name: skill-rs02-motor
description: |
  Robstride RS02 (灵足时代) 17Nm 一体化关节电机 —— 私有协议 (默认) 驱动速查。
version: "1.0.0"
metadata:
  author: Ricky
  last_updated: "2026-04-20"
  project: knee-exoskeleton
  dependencies: ["config", "can_bus"]
---

# RS02 Motor Skill

## 1. 电机概况

- 产品：灵足时代 Robstride RS02 准直驱一体化电机
- 额定 6Nm / 峰值 17Nm；减速比 7.75:1；空载最大 ~44 rad/s
- 驱动器默认参数：**CAN 2.0B，1 Mbps，29 位扩展帧**
- 出厂电机 ID = 0x7F，主机示例 ID = 0xFD
- 官方：<https://robstride.com/products/robStride02>  GitHub：<https://github.com/RobStride/Product_Information>
- 原始手册：`datasheets/RS02使用说明书260410.pdf`

## 2. 本项目接法

| 信号 | 电机侧 | 收发器 | MCU |
|---|---|---|---|
| CAN_H | 接口引脚4 | CAN_H | — |
| CAN_L | 接口引脚3 | CAN_L | — |
| TXD  | — | TXD | GPIO16 (`PIN_TWAI_TX`) |
| RXD  | — | RXD | GPIO15 (`PIN_TWAI_RX`) |
| VCC  | 24V~60V | 3V3 | 3V3 |
| GND  | GND | GND | GND |

> 所谓"CAN 转 TTL 模块"就是标准 CAN 收发器，只做电平转换，协议仍由 ESP32-S3 内部的 TWAI 控制器负责。

## 3. 协议三选一

| 协议 | 帧格式 | 优点 | 缺点 | 是否用 |
|---|---|---|---|---|
| **私有协议** | 29 位扩展帧 | 5 种控制模式、参数读写、故障详报、主动上报，示例代码完备 | 帧 ID 位域有点绕 | **✔ 本项目采用** |
| MIT 协议 | 11 位标准帧 | 帧简洁 | 功能受限，参数读写需 0.2.3.32+ 固件 | ✘ |
| CANopen | 29 位 SDO/PDO | 接入工控系统友好 | 状态机重 | ✘ |

## 4. 扩展帧 ID 布局（私有协议）

```
bit[28:24] | bit[23:8]      | bit[7:0]
   type    | data (16 bit)  | target_motor_id
```

data 域对不同类型含义不同：对多数类型是"主机 CAN_ID"，对类型 1 (运控) 是 `float_to_u16(torque_ff,-17,17)`。

## 5. 通信类型速查

| type | 名称 | 发起方 | 说明 |
|---|---|---|---|
| 0 | 获取设备 ID | 主机 | 广播后设备回 64-bit UUID |
| **1** | **运控模式控制** | 主机 | pos/vel/kp/kd + ff torque in ID |
| **2** | **反馈** | 电机 | pos/vel/torque/temp + 故障位 |
| **3** | **使能** | 主机 | data 区全 0 |
| **4** | **停止** | 主机 | data[0]=1 时清故障 |
| 6 | 设置零位 | 主机 | data[0]=1 |
| 7 | 修改 CAN_ID | 主机 | |
| **17** | **单参数读取** | 主机 | data[0:1]=index |
| **18** | **单参数写入** | 主机 | data[0:1]=index, data[4:7]=value |
| 21 | 故障反馈 | 电机 | 4 字节 fault + 4 字节 warning |
| 22 | 数据保存 | 主机 | |
| 23 | 波特率修改 | 主机 | |
| 24 | 主动上报开关 | 主机 | data 末字节 0=关 1=开 |

## 6. 运控模式帧格式 (type=1)

```
ID:   bit[28:24]=1, bit[23:8]=f2u16(torque_ff,-17,17), bit[7:0]=motor_id
Data: [0:1]=f2u16(pos,-12.57,12.57)  (大端)
      [2:3]=f2u16(vel,-44,44)
      [4:5]=f2u16(kp, 0, 500)
      [6:7]=f2u16(kd, 0, 5)
```

控制律（电机内部）：
$$\tau_{ref} = K_d(v_{set}-v_{actual}) + K_p(p_{set}-p_{actual}) + \tau_{ff}$$

**外骨骼典型参数建议**：`kp = 5~30`（柔顺），`kd = 0.5~2`，`torque_ff` 由上层前馈算法给。切忌一次给大 kp 带大期望偏差，会产生冲击。

## 7. 反馈帧格式 (type=2)

```
ID:   bit[23:16]=故障位, bit[15:8]=电机CAN_ID, bit[7:0]=主机CAN_ID
      (data_field bit[13:8] = 故障 bit[21:16]；bit[15:14] = 模式状态)
Data: [0:1]=pos (大端)
      [2:3]=vel (大端)
      [4:5]=torque (大端)
      [6:7]=temperature*10 (大端)
```

模式状态：`0=Reset` `1=Cali` `2=Motor`。

## 8. 常用参数 index (0x7000 段)

| index | 名称 | 类型 | 量纲 | 用途 |
|---|---|---|---|---|
| 0x7005 | run_mode | uint8 | 0 运控 / 1 PP / 2 速度 / 3 电流 / 5 CSP | 切模式必写 |
| 0x7006 | iq_ref | float | -16~16 A | 电流模式指令 |
| 0x700A | spd_ref | float | -33~33 rad/s | 速度模式指令 |
| 0x700B | limit_torque | float | 0~14 Nm | 力矩上限 |
| 0x7016 | loc_ref | float | rad | 位置指令 (CSP/PP) |
| 0x7017 | limit_spd | float | 0~33 rad/s | CSP 速度上限 |
| 0x7018 | limit_cur | float | 0~16 A | 速度模式电流上限 |
| 0x7019 | mechPos | float (R) | rad | 机械角度 |
| 0x701B | mechVel | float (R) | rad/s | 机械角速度 |
| 0x701C | VBUS | float (R) | V | 母线电压 |
| 0x701E | loc_kp | float | 默认 40 | 位置环 Kp |
| 0x701F | spd_kp | float | 默认 6 | 速度环 Kp |
| 0x7020 | spd_ki | float | 默认 0.02 | 速度环 Ki |
| 0x7024 | vel_max | float | rad/s | PP 模式速度 |
| 0x7025 | acc_set | float | rad/s² | PP 模式加速度 |

## 9. 故障位速查 (type=21 Byte[0:3])

| bit | 含义 |
|---|---|
| 0 | 过温 (默认 135°C) |
| 1 | 驱动芯片故障 |
| 2 | 欠压 |
| 3 | 过压 |
| 4 | B 相过流 |
| 5 | C 相过流 |
| 7 | 编码器未标定 |
| 8 | 硬件识别故障 |
| 9 | 位置初始化故障 |
| 14 | 堵转过载保护 |
| 16 | A 相过流 |

## 10. 典型启动序列 (运控模式)

```c
rs02_init(&m, 0x7F, 0xFD);
rs02_set_mode(&m, RS02_MODE_MOTION);   // 写 0x7005 = 0
vTaskDelay(20 / portTICK_PERIOD_MS);
rs02_enable(&m);                        // 类型 3
// 控制循环:
rs02_motion_control(&m, 0.0f, 0.0f, 0.0f, 10.0f, 1.0f);
// 断电前:
rs02_stop(&m, false);                   // 类型 4
```

## 11. 代码入口

- 头：[`components/drivers/rs02_motor/include/rs02_motor.h`](../../../components/drivers/rs02_motor/include/rs02_motor.h)
- 实现：[`components/drivers/rs02_motor/rs02_motor.c`](../../../components/drivers/rs02_motor/rs02_motor.c)

## 12. 参考

1. RS02 使用说明书 260410：`datasheets/RS02使用说明书260410.pdf`（章节 4.1 / 4.4）
2. 官方 GitHub：<https://github.com/RobStride/Product_Information>

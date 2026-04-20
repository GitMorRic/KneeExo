---
name: skill-esp32s3-core
description: |
  ESP32-S3-N16R8 主控核心：GPIO/外设分配、ESP-IDF v5.5.4 构建指引。
version: "1.0.0"
metadata:
  author: Ricky
  last_updated: "2026-04-20"
  project: knee-exoskeleton
  dependencies: ["config"]
---

# ESP32-S3 Core Skill

## 1. 芯片概况

- 主芯片：**ESP32-S3-N16R8** (16MB Flash + 8MB Octal PSRAM)
- 双核 Xtensa LX7 @ 240MHz
- 45 个可编程 GPIO，其中 GPIO26~32 被 Flash/PSRAM 占用，**GPIO22 不存在**
- 自带 TWAI (CAN 2.0B) 控制器，无需外挂 CAN 控制器芯片

## 2. 本项目 GPIO / 外设分配表

| GPIO | 外设 | 方向 | 用途 | 备注 |
|---|---|---|---|---|
| GPIO43 | UART0 TX | OUT | 日志 / 下载 (`idf.py monitor`) | 固定，勿改 |
| GPIO44 | UART0 RX | IN  | 日志 / 下载 | 固定，勿改 |
| GPIO42 | UART1 TX | OUT | IMU1 MCU→IMU RX | JTAG MTMS，见下方说明 |
| GPIO41 | UART1 RX | IN  | IMU1 IMU→MCU TX | JTAG MTDI，见下方说明 |
| GPIO48 | UART2 TX | OUT | IMU2 MCU→IMU RX | DevKitC-1 板载 RGB LED，见下方说明 |
| GPIO47 | UART2 RX | IN  | IMU2 IMU→MCU TX | — |
| GPIO16 | TWAI TX  | OUT | CAN 收发器 TX | XTAL_32K_P（无外接晶振时可用） |
| GPIO15 | TWAI RX  | IN  | CAN 收发器 RX | XTAL_32K_N（无外接晶振时可用） |
| GPIO1  | ADC1_CH0 | IN  | 电池电压采样 | 外部 11:1 分压，注意：CH0 ≠ CH3 |
| GPIO9  | GPIO IN  | IN  | 按键 SUB | 内部上拉 |
| GPIO10 | GPIO IN  | IN  | 按键 ADD | 内部上拉 |
| GPIO11 | GPIO IN  | IN  | 按键 MODE | 内部上拉 |
| GPIO12 | GPIO OUT | OUT | WS2812 LED 数据线 | — |
| GPIO26~32 | — | — | **占用（Flash/PSRAM）** | 不可用 |
| GPIO19/20 | USB JTAG | — | 板载 USB-CDC / JTAG | idf.py flash/monitor 默认走这里 |

> ⚠️ **GPIO41/42 (JTAG MTDI/MTMS)**：若使用 OpenOCD + 外部 JTAG 探针调试，这两根线会被占用，导致 IMU1 通信故障。若只使用板载 USB-JTAG（GPIO19/20），可放心使用 GPIO41/42。
>
> ⚠️ **GPIO48 (RGB_LED)**：ESP32-S3-DevKitC-1 开发板在 GPIO48 上接了一颗 WS2812 RGB LED。正式 PCB 上无此冲突。

引脚定义集中在 [`components/config/include/config.h`](../../../components/config/include/config.h)，改板时只改这一处。

## 3. 构建与烧录

前提：已安装 ESP-IDF v5.5.4 并 `export.ps1` 过。

```powershell
# 一次性设置目标
idf.py set-target esp32s3

# 打开菜单改配置 (可选)
idf.py menuconfig

# 编译 + 烧录 + 打开串口
idf.py -p COM3 flash monitor
```

首次构建会生成 `build/`；切板子重新 set-target 会重置 sdkconfig。

## 4. sdkconfig 重点

| 配置 | 值 | 原因 |
|---|---|---|
| `CONFIG_FREERTOS_HZ` | 1000 | 10ms 周期控制循环需要 ≥1kHz tick |
| `CONFIG_SPIRAM` | y (Octal 80M) | 给未来上位机协议、大缓冲留空间 |
| `CONFIG_COMPILER_CXX_EXCEPTIONS` | n | 实时场景不用 C++ 异常 |
| `CONFIG_ESP_MAIN_TASK_STACK_SIZE` | 8192 | 初始化多组件够用 |

## 5. 分区表

自定义 `partitions.csv`：3MB factory app + 1MB SPIFFS，后续加 OTA 再拆 app0/app1。

## 6. 常见坑

1. **GPIO 编号**：ESP32-S3 的可用引脚范围与 ESP32 不同。26~32 是 PSRAM/Flash 专用，不能用于普通 IO。
2. **TWAI 必须配合外部收发器**：内部只是 CAN 控制器，不能直连 CAN_H/CAN_L。
3. **C 代码头文件顺序**：`driver/twai.h` 需要在 `freertos/FreeRTOS.h` 之后；否则 `TickType_t` 未定义。
4. **xTaskCreatePinnedToCore 核心选择**：控制循环钉在 Core 1，日志/IMU RX 任务放 Core 0，避免日志 jitter 打乱控制周期。

## 7. 参考资料

1. [ESP-IDF v5.5.4 文档（乐鑫官网）](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.5.4/esp32s3/get-started/index.html)
2. ESP32-S3 数据表：`datasheets/esp32-s3-pinout.png`
3. TWAI 驱动：<https://docs.espressif.com/projects/esp-idf/zh_CN/v5.5.4/esp32s3/api-reference/peripherals/twai.html>

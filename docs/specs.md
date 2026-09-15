# 当前硬件与软件配置

[返回 README](../README.md) · [架构](architecture.md) · [烧录](flashing.md)

本页按当前 [config.h](../components/config/include/config.h)、[sdkconfig.defaults](../sdkconfig.defaults) 和实现整理。常量描述软件期望，不证明现场装配、电池或传感器设置与其一致。旧文档中的 CAN GPIO5/6、ADC GPIO4、IMU 115200 等不能继续作为当前接线依据。

## 1. 工程与硬件

| 项目 | 当前软件基线/期望 |
|---|---|
| 主控 | ESP32-S3-N16R8，16MB Flash / 8MB Octal PSRAM |
| 框架 | ESP-IDF v5.5.4，C/C++，CMake / idf.py |
| 电机 | 单侧 RobStride RS02，CAN 私有协议，运控模式 |
| CAN | 经典 CAN/TWAI，1 Mbps，扩展帧；外接与 MCU 3.3V 逻辑兼容的收发器 |
| IMU | WitMotion WT-IMU63 / 驱动兼容0x55串口帧；UART 9600、8N1 |
| 控制与上位机 | 默认100 Hz控制；PC主控制台默认115200；Qt/PyQtGraph |
| 存储 | NVS保存部分标定和参数；PC保存实验CSV/PNG |

RS02电气与安装要求须核对仓库保存的[电机手册](skills/skill_rs02_motor/datasheets/RS02使用说明书260410.pdf)以及实际设备铭牌。协议编码范围±17 Nm不等于当前允许给人体施加的力矩。当前还没有双腿控制、板端SD日志或康复轨迹控制。

## 2. 当前接线表

方向以MCU为参照；UART TX接对端RX，RX接对端TX，电源地共地。

| 信号 | GPIO / 接口 | 说明 |
|---|---|---|
| CAN TX | GPIO16 | 接收发器TXD；不要直接接CANH |
| CAN RX | GPIO15 | 接收发器RXD；CANH/L从收发器到电机 |
| IMU1 TX / RX | GPIO17 / GPIO18，UART1 | 大腿预留；normal目前初始化它，但不用它计算控制 |
| **IMU2 TX / RX** | **GPIO21 / GPIO38，UART2** | **当前小腿控制输入；优先核对** |
| 电池采样 | GPIO1 / ADC1_CH0 | 分压与保护须重新核对，见下节 |
| 用户减/加按键 | GPIO9 / GPIO10 | 仅定义常量，尚未接入控制交互 |

CAN电机ID=`0x02`，主机ID=`0xFD`；这是现有装配记录的配置，不保证另一台RS02使用相同ID。终端电阻、线束、供电、收发器型号必须按真实总线核查。TWAI自环只验证控制器自收发路径。

IMU供电/电平按实际模块规格确认；ESP32-S3接口不应接入5V逻辑。旧记录采用3.3V IMU供电。参见 [ESP32-S3数据手册的DC Characteristics](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)。

## 3. 电源与ADC：旧说明必须纠正

旧资料中24V、48V供电描述混用，当前仓库没有足够的实测装配记录来确定实际电池电压或完整电源设计。

**不要照旧文档把48V经11:1分压接入GPIO1。** `48/11≈4.36V`，超过此3.3V域ADC的适用输入范围。当前 `BAT_DIVIDER_RATIO=11.0f` 只是软件换算系数，不是经过验证的高压采样电路。旧注释“9.1kΩ+1kΩ=11:1”也不对，实际是10.1:1。

应按实际电池**最高满充电压**、电阻容差和保护电路核算：

```text
分压比 = (R上 + R下) / R下
ADC最大输入 = 电池最高电压 / 分压比
```

选择输入范围时同时核对芯片电气限制和当前ADC衰减下的可测范围，留出容差后再实测、同步修改软件系数。[ESP-IDF ADC文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/api-reference/peripherals/adc_oneshot.html)说明量程与衰减相关，换算还需要校准。

当前软件尝试ADC校准，无法校准时使用近似换算；`bat_v`仅用于显示，没有完成欠压停机策略。历史日志的电压读数不能反推出真实电池规格。

## 4. 机械和姿态标定

| 常量 | 当前值 | 含义 |
|---|---|---|
| `JOINT_DIR_SIGN` | −1 | 电机位置增大对应伸膝，膝屈曲为正 |
| `JOINT_ZERO_OFFSET_RAD` | +1.74678 rad | 电机坐标中的历史机械伸膝端 |
| `KNEE_EXT_LIMIT_RAD` | 0 rad | 机械关节坐标伸膝边界 |
| `KNEE_FLEX_LIMIT_RAD` | 2.53286 rad | 历史机械屈膝边界常量 |
| `IMU_SHANK_PITCH_AXIS / SIGN` | 1 / +1 | 取Euler pitch，向前摆为正 |
| `SHANK_GRAVITY_G_NM / PHI_RAD / BIAS_NM` | 0 / 0 / 0 | 默认无有效补偿；设备NVS可能覆盖 |

机械常量只适用于原装配。控制另有约−5°…100°的**用户角度**FSM回退阈值，和机械边界不是同一概念。用户零位平移不会修改机械限位。先读[坐标和启动说明](architecture.md)，重装后做离体标定，不能把自动碰限位程序用于穿戴状态。

## 5. 性能与保护的验证边界

100 Hz是控制任务设置值；UART9600限制了可获取的新IMU帧速率。`<5ms`延迟、`±200μs`抖动、续航和人体助力收益仍是待测指标，未在本次验证。

当前有分量限幅、目标软限位和FSM回退，但缺少最终输出许可、传感器全面超时和可靠急停闭环；详细列表见[项目现状](project-status.md)。正常编译或收到反馈不能替代硬件/控制验收。

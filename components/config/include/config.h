// config.h
// 全局硬件常量：引脚、总线速率、电机参数限幅
// 所有可复用组件都只依赖这个头文件；改板时只改这里。
//
// 注意：本文件使用 C++ namespace + constexpr，只能被 .cpp 文件 include。
// 如需在 .c 文件中使用常量，请将该 .c 文件改名为 .cpp。
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/twai.h"

//初级写法
//#define PIN_IMU_UART_TX     GPIO_NUM_17
//中级写法
//constexpr uint8_t PIN_IMU_UART_TX = 17;
//高级写法
//constexpr gpio_num_t PIN_IMU_UART_TX = GPIO_NUM_17;
//最佳实践
//namespace ExoConfig{
//  constexpr gpio_num_t PIN_IMU_UART_TX = GPIO_NUM_17;
//}
namespace ExoConfig{
    // =============================================================================
    // 1. 电池采样
    //    GPIO1 = ADC1_CH0（通道 0）
    //      ADC1_CH0=GPIO1, CH1=GPIO2, CH3=GPIO4, CH4=GPIO5 ...（见 datasheets/esp32-s3-pinout.png）
    //    分压比：11:1（9.1kΩ + 1kΩ），48V 电池时 ADC 输入约 4.36V
    // =============================================================================
    constexpr gpio_num_t  PIN_BAT_ADC         = GPIO_NUM_1;   // ADC1_CH0
    constexpr float       BAT_DIVIDER_RATIO   = 11.0f;

    // =============================================================================
    // 2. IMU — WitMotion WT-IMU63，UART，9600 8N1（出厂默认波特率）
    //    接线（交叉）：MCU TX -> IMU RX，MCU RX <- IMU TX，⚠ 必须 3.3V 供电
    //    ⚠ ESP32-S3 GPIO 最大输入 3.6V，IMU 用 5V 供电时 TX 可能输出 5V 电平，损坏 GPIO。
    //
    //    ⚠️ GPIO41 (MTDI) / GPIO42 (MTMS) 是 JTAG 经典调试引脚。
    //       ESP-IDF 启动时默认把 GPIO39-42 配为 JTAG，即使只用板载 USB-JTAG (GPIO19/20)，
    //       GPIO42 在固件里也会被标为 "not usable"，导致 UART 发送失败。
    //       已改为普通 GPIO 引脚，彻底避免冲突。
    //
    //    IMU1: UART1, TX=GPIO17, RX=GPIO18  ← 原 GPIO42/41，已换为无冲突引脚
    //    IMU2: UART2, TX=GPIO21, RX=GPIO38
    //
    //    ⚠️ GPIO48 在 ESP32-S3-DevKitC-1 开发板上接了板载 RGB LED (WS2812)。
    //       已将 IMU2 TX 改到 GPIO21，避免冲突。
    // =============================================================================
    constexpr gpio_num_t  PIN_IMU1_MCU_TX     = GPIO_NUM_17;  // 普通 GPIO，无冲突
    constexpr gpio_num_t  PIN_IMU1_MCU_RX     = GPIO_NUM_18;  // 普通 GPIO，无冲突
    constexpr gpio_num_t  PIN_IMU2_MCU_TX     = GPIO_NUM_21;  // 普通 GPIO（原 GPIO48 冲突 RGB LED）
    constexpr gpio_num_t  PIN_IMU2_MCU_RX     = GPIO_NUM_38;

    constexpr uart_port_t IMU1_UART_NUM       = UART_NUM_1;
    constexpr uart_port_t IMU2_UART_NUM       = UART_NUM_2;
    constexpr int         IMU_UART_BAUD       = 9600;   // WitMotion 出厂默认；可通过上位机软件改至 115200

    // =============================================================================
    // 3. CAN 总线 (TWAI) — 1 Mbps，连接至 CAN 电平 <-> TTL 收发器模块
    //    GPIO15 / GPIO16 为 32kHz 晶振引脚（XTAL_32K_N / XTAL_32K_P），
    //    DevKitC-1 无外接 32kHz 晶振，故这两个引脚可用作普通 GPIO。
    // =============================================================================
    constexpr gpio_num_t  PIN_TWAI_TX         = GPIO_NUM_16;
    constexpr gpio_num_t  PIN_TWAI_RX         = GPIO_NUM_15;
    constexpr uint32_t    CAN_BITRATE_HZ      = 1000000U;

    // =============================================================================
    // 4. RS02 电机（灵足时代 Robstride RS02，私有协议默认）
    //    出厂 CAN_ID = 0x7F；本 MCU 主机 ID 可任意选 0x00~0xFE（不与电机 ID 重复）
    //    额定 6 Nm / 峰值 17 Nm；出轴最大 ~44 rad/s；减速比 7.75:1
    // =============================================================================
    constexpr uint8_t     RS02_CAN_ID         = 0x02;  // 实测扫描到的电机 ID（出厂非默认 0x7F）
    constexpr uint8_t     RS02_HOST_ID        = 0xFD;

    // 运控模式编解码范围（对应 float <-> uint16 线性映射）
    constexpr float       MOTOR_MAX_TORQUE    = 17.0f;    // Nm
    constexpr float       MOTOR_MIN_TORQUE    = -17.0f;
    constexpr float       MOTOR_MAX_SPEED     = 44.0f;    // rad/s
    constexpr float       MOTOR_MIN_SPEED     = -44.0f;
    constexpr float       MOTOR_MAX_POS       = 12.57f;   // rad (~4π)
    constexpr float       MOTOR_MIN_POS       = -12.57f;
    constexpr float       MOTOR_MAX_KP        = 500.0f;
    constexpr float       MOTOR_MIN_KP        = 0.0f;
    constexpr float       MOTOR_MAX_KD        = 5.0f;
    constexpr float       MOTOR_MIN_KD        = 0.0f;

    // =============================================================================
    // 5. 用户交互 — 按键 + WS2812 LED 灯条
    //    按键：低电平触发（外部上拉或内部上拉），TOUCH 功能也可用于电容感应
    // =============================================================================
    constexpr gpio_num_t  PIN_BTN_SUB         = GPIO_NUM_9;
    constexpr gpio_num_t  PIN_BTN_ADD         = GPIO_NUM_10;
    constexpr gpio_num_t  PIN_BTN_MODE        = GPIO_NUM_11;
    constexpr gpio_num_t  PIN_LED_DATA        = GPIO_NUM_12;  // WS2812 数据线

    // =============================================================================
    // 6. 关节安装方向（标定参数）
    //
    //    定义统一约定：
    //      joint_angle_rad > 0 表示膝关节屈曲方向（小腿向大腿后方靠拢）。
    //      joint_angle_rad = 0 表示直腿。
    //
    //    安装情况 v0.1（右腿）：
    //      - 电机定子固定在大腿外侧支架，输出轴朝身体内侧、连接小腿；
    //      - IMU1 装大腿外侧 PCB（用 IMU1：UART1 / GPIO17-18）。
    //
    //    映射公式：
    //      joint_angle_rad     = JOINT_DIR_SIGN * (motor_pos_rad - JOINT_ZERO_OFFSET_RAD)
    //      joint_velocity_rad  = JOINT_DIR_SIGN * motor_velocity_rad
    //      joint_torque_nm     = JOINT_DIR_SIGN * motor_torque_nm
    //
    //    JOINT_DIR_SIGN  - +1：电机正向旋转 (motor_pos +) ⇒ 屈膝；
    //                      -1：电机正向旋转 (motor_pos +) ⇒ 伸膝。
    //    JOINT_ZERO_OFFSET_RAD - 直腿姿态下电机的机械位置 (rad)。
    //
    //    ⚠ 这两个值必须用 motor_test profile 实测后改实！下方为占位默认值。
    //    标定步骤见 docs/skills/skill_rs02_motor/SKILL.md §10 与 README。
    //
    //    安全限位（软件保护）：
    //      KNEE_FLEX_LIMIT_RAD：屈膝最大角度 (默认 120°)
    //      KNEE_EXT_LIMIT_RAD ：伸膝最大角度，负值表示防超伸 (默认 -5°)
    // =============================================================================
    enum class LegSide : uint8_t { Right = 0, Left = 1 };
    constexpr LegSide LEG_SIDE                = LegSide::Right;

    constexpr float   JOINT_DIR_SIGN          = +1.0f;     // ⚠ 待标定
    constexpr float   JOINT_ZERO_OFFSET_RAD   = 0.0f;      // ⚠ 待标定
    constexpr float   KNEE_FLEX_LIMIT_RAD     = 2.094f;    // 120°
    constexpr float   KNEE_EXT_LIMIT_RAD      = -0.087f;   // -5°（防超伸）

    // 关节坐标 <-> 电机坐标 转换
    inline float motor_to_joint_rad   (float motor_pos)    { return JOINT_DIR_SIGN * (motor_pos - JOINT_ZERO_OFFSET_RAD); }
    inline float joint_to_motor_rad   (float joint_rad)    { return JOINT_DIR_SIGN * joint_rad + JOINT_ZERO_OFFSET_RAD; }
    inline float motor_to_joint_vel   (float motor_vel)    { return JOINT_DIR_SIGN * motor_vel; }
    inline float joint_to_motor_vel   (float joint_vel)    { return JOINT_DIR_SIGN * joint_vel; }
    inline float motor_to_joint_torque(float motor_tq)     { return JOINT_DIR_SIGN * motor_tq; }
    inline float joint_to_motor_torque(float joint_tq)     { return JOINT_DIR_SIGN * joint_tq; }

    // =============================================================================
    // 7. IMU 安装方向（标定参数）
    //
    //    IMU1 装大腿外侧。我们关心：哪一个欧拉角是「大腿俯仰角」(thigh pitch)，
    //    并约定 thigh_pitch_rad > 0 代表大腿向前抬起 (髋屈曲)。
    //
    //    IMU_THIGH_PITCH_AXIS：euler_deg[i] 中哪一维 (0=Roll/X, 1=Pitch/Y, 2=Yaw/Z)。
    //    IMU_THIGH_PITCH_SIGN：取出后是否翻转符号 (取决于 IMU 的物理朝向)。
    //
    //    ⚠ 同样需要标定：人静坐 -> 大腿前抬 30° -> 看哪个角度变化最大、方向是否对。
    // =============================================================================
    constexpr int     IMU_THIGH_PITCH_AXIS    = 1;          // ⚠ 待标定（默认 Pitch）
    constexpr float   IMU_THIGH_PITCH_SIGN    = +1.0f;      // ⚠ 待标定

} // namespace ExoConfig

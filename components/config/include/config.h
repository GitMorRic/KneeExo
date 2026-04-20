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
    // 2. IMU — WitMotion WT-IMU63，UART，115200 8N1
    //    接线（交叉）：MCU TX -> IMU RX，MCU RX <- IMU TX，3.3V/5V，GND
    //
    //    ⚠️ GPIO41 (MTDI) / GPIO42 (MTMS) 是 JTAG 经典调试引脚。
    //       若需要用 OpenOCD + JTAG 探针调试，这两个引脚会被占用，需换其他 GPIO。
    //       若只用板载 USB-JTAG（GPIO19/20），GPIO41/42 空闲，可以用。
    //
    //    IMU1: UART1, TX=GPIO42, RX=GPIO41
    //    IMU2: UART2, TX=GPIO48, RX=GPIO47
    //
    //    ⚠️ GPIO48 在 ESP32-S3-DevKitC-1 开发板上接了板载 RGB LED (WS2812)。
    //       若使用 DevKitC-1 开发板调试，建议 IMU2 TX 改到其他引脚（如 GPIO17）。
    //       若是自制 PCB 则无此冲突。
    // =============================================================================
    constexpr gpio_num_t  PIN_IMU1_MCU_TX     = GPIO_NUM_42;  // JTAG MTMS — 见上方说明
    constexpr gpio_num_t  PIN_IMU1_MCU_RX     = GPIO_NUM_41;  // JTAG MTDI — 见上方说明
    constexpr gpio_num_t  PIN_IMU2_MCU_TX     = GPIO_NUM_48;  // DevKitC-1 板载 RGB LED — 见上方说明
    constexpr gpio_num_t  PIN_IMU2_MCU_RX     = GPIO_NUM_47;

    constexpr uart_port_t IMU1_UART_NUM       = UART_NUM_1;
    constexpr uart_port_t IMU2_UART_NUM       = UART_NUM_2;
    constexpr int         IMU_UART_BAUD       = 115200;

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
    constexpr uint8_t     RS02_CAN_ID         = 0x7F;
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

} // namespace ExoConfig

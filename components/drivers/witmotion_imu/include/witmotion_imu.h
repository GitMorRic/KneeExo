// witmotion_imu.h
// 维特智能 (WitMotion) WT-IMU63 六轴 IMU 串口驱动
// 兼容 JY901 标准协议帧：
//   [0x55] [kind] [d0..d7] [checksum]    共 11 字节
//   checksum = (0x55 + kind + d0..d7) & 0xFF
//
// 本驱动处理的帧类型：
//   0x51 加速度 (acc_x,y,z + temp)     ：acc = (int16)/32768 * 16g
//   0x52 角速度 (gyro_x,y,z + temp)    ：gyro = (int16)/32768 * 2000 °/s
//   0x53 角度   (roll,pitch,yaw + ver) ：angle = (int16)/32768 * 180 °
//   0x59 四元数 (q0,q1,q2,q3)          ：q = (int16)/32768
//
// 接线：MCU TX(17) -> IMU RX；MCU RX(18) <- IMU TX；3.3V、GND。
// 默认波特率 115200，默认按 10ms 上报。
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float   accel_g[3];     // Ax, Ay, Az (单位 g)
    float   gyro_dps[3];    // Gx, Gy, Gz (单位 deg/s)
    float   euler_deg[3];   // Roll, Pitch, Yaw (单位 deg)
    float   quat[4];        // q0, q1, q2, q3 (归一化 -1~1)
    float   temp_c;         // 芯片温度 (来自 0x51 或 0x52 帧)
    int64_t ts_us;          // 最新一帧 0x53 到达时间
    uint32_t frame_count;   // 已解析有效帧总数
} witmotion_data_t;

/**
 * @brief 初始化串口 + 启动后台 RX 任务
 */
esp_err_t witmotion_init(uart_port_t port,
                         gpio_num_t  tx_pin,
                         gpio_num_t  rx_pin,
                         int         baud);

esp_err_t witmotion_init_channel(uart_port_t port,
                                 gpio_num_t  tx_pin,
                                 gpio_num_t  rx_pin,
                                 int         baud);

/**
 * @brief 拷贝一份最新数据（线程安全，mutex 保护）
 */
esp_err_t witmotion_get_latest(witmotion_data_t *out);

esp_err_t witmotion_get_latest_channel(uart_port_t port, witmotion_data_t *out);

/**
 * @brief 是否至少收到过一帧有效数据
 */
bool witmotion_is_alive(void);

bool witmotion_is_alive_channel(uart_port_t port);

#ifdef __cplusplus
}
#endif

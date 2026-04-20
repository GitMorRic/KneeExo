// rs02_motor.h
// 灵足时代 Robstride RS02 电机驱动（私有协议，默认出厂）
// 硬件通道：ESP32-S3 TWAI -> CAN 收发器 -> RS02 (1Mbps, 29-bit 扩展帧)
//
// 协议要点（依据《RS02 使用说明书 260410》4.1 节）：
//   - 所有帧 DLC = 8，扩展帧 ID 按位划分：
//       bit[28:24] = 通信类型 (0~25)
//       bit[23:8]  = data 域 (通常高 8 位 = 主机 CAN_ID 或额外数据)
//       bit[7:0]   = 目标电机 CAN_ID
//   - 运控模式 (通信类型 1)：
//       bit[23:8]  = float_to_u16(torque_ff, -17, 17)   [高字节在前]
//       Byte[0:1]  = pos  (rad, -12.57 ~ 12.57)
//       Byte[2:3]  = vel  (rad/s, -44 ~ 44)
//       Byte[4:5]  = kp   (0 ~ 500)
//       Byte[6:7]  = kd   (0 ~ 5)
//       以上 u16 均为大端序 (高字节在前)
//   - 反馈 (通信类型 2)：
//       bit[23:16] = 故障位；bit[15:8] = 当前电机 CAN_ID；bit[7:0] = 主机 CAN_ID
//       Byte[0:1]  = pos   (rad, 大端)
//       Byte[2:3]  = vel   (rad/s, 大端)
//       Byte[4:5]  = torque (Nm, 大端)
//       Byte[6:7]  = temperature*10 (℃, 大端)
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------- 控制模式 (对应参数 0x7005 run_mode) ------------------------------
typedef enum {
    RS02_MODE_MOTION  = 0,  // 运控模式（默认/推荐，外骨骼阻抗控制）
    RS02_MODE_POS_PP  = 1,  // 位置模式 PP
    RS02_MODE_SPEED   = 2,
    RS02_MODE_CURRENT = 3,
    RS02_MODE_POS_CSP = 5,
} rs02_mode_t;

// ---------- 反馈结构 ---------------------------------------------------------
typedef struct {
    float    position_rad;     // -4π ~ 4π
    float    velocity_rad_s;   // -44 ~ 44 rad/s
    float    torque_nm;        // -17 ~ 17 Nm
    float    temperature_c;    // 摄氏度
    uint8_t  mode_state;       // 0=Reset 1=Cali 2=Motor
    bool     has_fault;        // 任意故障位为 1 时置 true
    uint8_t  fault_bits;       // 见说明书 4.1 "通信类型 2 / 21"
    int64_t  ts_us;            // 接收时刻 esp_timer_get_time()
} rs02_feedback_t;

// ---------- 常用参数 index (0x7000 段) ---------------------------------------
#define RS02_IDX_RUN_MODE       0x7005  // uint8
#define RS02_IDX_IQ_REF         0x7006  // float, -16~16 A
#define RS02_IDX_SPD_REF        0x700A  // float, -33~33 rad/s
#define RS02_IDX_LIMIT_TORQUE   0x700B  // float, 0~14 Nm
#define RS02_IDX_LOC_REF        0x7016  // float, rad
#define RS02_IDX_LIMIT_SPD      0x7017  // float, 0~33 rad/s (CSP)
#define RS02_IDX_LIMIT_CUR      0x7018  // float, 0~16 A (SPEED)
#define RS02_IDX_MECH_POS       0x7019  // float, R only
#define RS02_IDX_MECH_VEL       0x701B  // float, R only
#define RS02_IDX_VBUS           0x701C  // float, R only
#define RS02_IDX_LOC_KP         0x701E  // float
#define RS02_IDX_SPD_KP         0x701F  // float
#define RS02_IDX_SPD_KI         0x7020  // float
#define RS02_IDX_ACC_RAD        0x7022  // float, rad/s^2
#define RS02_IDX_VEL_MAX        0x7024  // float, PP 模式速度
#define RS02_IDX_ACC_SET        0x7025  // float, PP 模式加速度

// ---------- 驱动句柄 ---------------------------------------------------------
// 每台电机一个句柄。内部 rx_task 全局只起一次，由 rs02_init 第一次调用时懒启动。
typedef struct {
    uint8_t          motor_can_id;
    uint8_t          host_can_id;
    QueueHandle_t    rx_queue;       // 长度 4：最新 feedback (溢出丢最旧)
    rs02_feedback_t  last_feedback;  // 供非阻塞 peek
    bool             has_feedback;   // 至少收到过一次有效 type=2
    bool             enabled;
    rs02_mode_t      mode;

    // 参数读取同步（通信类型 17 的应答）
    SemaphoreHandle_t param_resp_sem;
    uint16_t         pending_param_idx;
    uint8_t          param_resp_raw[4];
} rs02_handle_t;

// =============================================================================
// API
// =============================================================================

esp_err_t rs02_init(rs02_handle_t *h, uint8_t motor_id, uint8_t host_id);

// 通信类型 3：电机使能
esp_err_t rs02_enable(rs02_handle_t *h);

// 通信类型 4：电机停止运行。clear_fault=true 时 Byte[0]=1，尝试清故障。
esp_err_t rs02_stop(rs02_handle_t *h, bool clear_fault);

// 通信类型 6：设置当前位置为机械零位
esp_err_t rs02_set_zero(rs02_handle_t *h);

// 通信类型 18 写 0x7005 切模式
esp_err_t rs02_set_mode(rs02_handle_t *h, rs02_mode_t mode);

/**
 * 通信类型 1：运控模式下的一条控制帧
 * @param torque_ff  前馈力矩 Nm, -17 ~ 17
 * @param pos        目标位置 rad, -12.57 ~ 12.57
 * @param vel        目标速度 rad/s, -44 ~ 44
 * @param kp         0 ~ 500
 * @param kd         0 ~ 5
 */
esp_err_t rs02_motion_control(rs02_handle_t *h,
                              float torque_ff,
                              float pos,
                              float vel,
                              float kp,
                              float kd);

// 通信类型 18：写入单个 float 参数（掉电丢失）
esp_err_t rs02_write_param_f32(rs02_handle_t *h, uint16_t index, float value);

// 通信类型 18：写入单个 uint8 参数
esp_err_t rs02_write_param_u8(rs02_handle_t *h, uint16_t index, uint8_t value);

// 通信类型 17：读取单个 float 参数
esp_err_t rs02_read_param_f32(rs02_handle_t *h, uint16_t index, float *out, TickType_t timeout);

/**
 * 取最新反馈。
 *  - timeout = 0 时：不阻塞，取最近一次缓存的 feedback，若从未收到返回 ESP_ERR_NOT_FOUND
 *  - timeout > 0 时：阻塞等待下一帧新反馈。
 */
esp_err_t rs02_get_feedback(rs02_handle_t *h, rs02_feedback_t *out, TickType_t timeout);

#ifdef __cplusplus
}
#endif

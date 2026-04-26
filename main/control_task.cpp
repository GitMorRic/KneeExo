// control_task.cpp
// 100Hz 控制循环 demo（profile = normal 时启动）：
//
// 仅在 normal profile 下编译；其他 profile 不引用 g_motor。
//   - 读取 IMU 最新数据
//   - 读取电机反馈
//   - 下发温和的运控指令 (保持关节 0 位，小 Kp/Kd，0 力矩前馈)
//   - 每 0.5s 打印一行日志（关节坐标 + 大腿俯仰）
//
// 后续替换思路：
//   1. 从 IMU 估算大腿姿态 / 步态相位
//   2. 由相位机决定 (target_joint_pos, torque_ff)
//   3. 经 joint_to_motor_rad/_torque 转回电机坐标，调 rs02_motion_control

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_NORMAL

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

#include "config.h"
#include "rs02_motor.h"
#include "witmotion_imu.h"
#include "app_main.hpp"

using namespace ExoConfig;

static const char *TAG = "control";

static constexpr float DEFAULT_KP = (float)CONFIG_KNEEEXO_DEFAULT_KP / 100.0f;
static constexpr float DEFAULT_KD = (float)CONFIG_KNEEEXO_DEFAULT_KD / 100.0f;
static constexpr int   LOOP_HZ    = CONFIG_KNEEEXO_CONTROL_HZ;

extern "C" void control_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t loop_count = 0;
    const uint32_t log_every = LOOP_HZ / 2;  // 每 0.5s 打印一次
    witmotion_data_t  imu{};
    rs02_feedback_t   fb{};

    ESP_LOGI(TAG, "control loop @ %dHz, Kp=%.2f Kd=%.2f",
             LOOP_HZ, DEFAULT_KP, DEFAULT_KD);

    while (true) {
        witmotion_get_latest(&imu);
        rs02_get_feedback(&g_motor, &fb, 0);

        // 控制律 demo：关节保持在 0 位（直腿），给一点点刚度 + 阻尼
        const float target_joint_pos = 0.0f;
        const float target_joint_vel = 0.0f;
        const float joint_torque_ff  = 0.0f;

        rs02_motion_control(&g_motor,
                            joint_to_motor_torque(joint_torque_ff),
                            joint_to_motor_rad(target_joint_pos),
                            joint_to_motor_vel(target_joint_vel),
                            DEFAULT_KP,
                            DEFAULT_KD);

        if ((loop_count++ % log_every) == 0) {
            const float joint_pos = motor_to_joint_rad(fb.position_rad);
            const float thigh_pitch =
                IMU_THIGH_PITCH_SIGN * imu.euler_deg[IMU_THIGH_PITCH_AXIS];
            ESP_LOGI(TAG,
                     "thigh_pitch=%+6.2f deg | joint pos=%+6.3f rad (%+6.1f deg) "
                     "vel=%+6.3f tq=%+5.2f T=%4.1f%s",
                     thigh_pitch,
                     joint_pos, joint_pos * 180.0f / (float)M_PI,
                     motor_to_joint_vel(fb.velocity_rad_s),
                     motor_to_joint_torque(fb.torque_nm),
                     fb.temperature_c,
                     fb.has_fault ? " FAULT!" : "");
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_NORMAL

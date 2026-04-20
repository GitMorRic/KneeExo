// control_task.cpp
// 100Hz 控制循环 demo：
//   - 读取 IMU 最新数据 (俯仰角)
//   - 读取电机反馈 (位置/速度/力矩/温度)
//   - 下发温和的运控指令 (保持 0 位，小 Kp/Kd，0 力矩前馈)
//   - 每 0.5s 打印一行日志
//
// 在实际膝外骨骼控制中，这里会替换为：
//   - 从 IMU 估算大腿姿态 / 步态相位
//   - 计算期望关节角度或前馈力矩
//   - 调 rs02_motion_control 下发

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"
#include "rs02_motor.h"
#include "witmotion_imu.h"
#include "app_main.hpp"

static const char *TAG = "control";

// 默认刚度，可后续在 menuconfig 里调
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

        // 控制律 demo：保持在 0 位，给一点点刚度 + 阻尼
        const float target_pos = 0.0f;
        const float target_vel = 0.0f;
        const float torque_ff  = 0.0f;
        rs02_motion_control(&g_motor,
                            torque_ff,
                            target_pos,
                            target_vel,
                            DEFAULT_KP,
                            DEFAULT_KD);

        if ((loop_count++ % log_every) == 0) {
            ESP_LOGI(TAG,
                     "imu pitch=%+6.2f roll=%+6.2f yaw=%+6.2f | "
                     "motor pos=%+6.3f vel=%+6.3f tq=%+5.2f T=%4.1f%s",
                     imu.euler_deg[1], imu.euler_deg[0], imu.euler_deg[2],
                     fb.position_rad, fb.velocity_rad_s,
                     fb.torque_nm, fb.temperature_c,
                     fb.has_fault ? " FAULT!" : "");
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

// profile_motor_test.cpp
// 电机调通 + 方向标定 profile：
//   阶段 A（默认 KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN=n）：
//     - 初始化 CAN + RS02
//     - 不使能电机，仅每 100ms 打印一次反馈 (mech_pos / vel / torque / temp)
//     - 用于检查 CAN 接线、电机供电、ID 正确
//
//   阶段 B（KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN=y）：
//     - 上电后等 3s（人离手）
//     - set_mode(MOTION) -> enable
//     - 记录当前位置作 zero_ref
//     - 软软地以 ±0.1 rad 来回摆动 8 个周期 (kp=2, kd=0.5)
//     - 每帧打印 motor_pos 与 joint_pos（通过 motor_to_joint_rad 换算）
//     - 结束后 stop(false) 退出
//   ⚠ AUTOSPIN 时务必把电机悬空、未连腿，方便观察转向；
//     转向不对 → 改 config.h 的 JOINT_DIR_SIGN。

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <math.h>

#include "config.h"
#include "can_bus.h"
#include "rs02_motor.h"
#include "app_main.hpp"

using namespace ExoConfig;

static const char *TAG = "profile.motor";

static void log_feedback(const char *tag_extra, const rs02_feedback_t *fb)
{
    const float joint_rad = motor_to_joint_rad(fb->position_rad);
    ESP_LOGI(TAG,
             "%smotor pos=%+6.3f vel=%+6.3f tq=%+5.2f T=%4.1f mode=%d%s | "
             "joint pos=%+6.3f rad (%+6.1f deg)",
             tag_extra ? tag_extra : "",
             fb->position_rad, fb->velocity_rad_s,
             fb->torque_nm, fb->temperature_c,
             fb->mode_state, fb->has_fault ? " FAULT!" : "",
             joint_rad, joint_rad * 180.0f / (float)M_PI);
}

extern "C" void profile_motor_test_main(void)
{
    ESP_ERROR_CHECK(can_bus_init(PIN_TWAI_TX, PIN_TWAI_RX, CAN_BITRATE_HZ));

    static rs02_handle_t m;
    ESP_ERROR_CHECK(rs02_init(&m, RS02_CAN_ID, RS02_HOST_ID));
    vTaskDelay(pdMS_TO_TICKS(200));

    // ---------------- 阶段 A：只读反馈 ----------------
    ESP_LOGI(TAG, "Phase A: feedback-only (motor NOT enabled). 5s window.");
    for (int i = 0; i < 50; i++) {
        rs02_feedback_t fb;
        if (rs02_get_feedback(&m, &fb, pdMS_TO_TICKS(150)) == ESP_OK) {
            if (i % 5 == 0) log_feedback("A: ", &fb);
        } else {
            // 没反馈：rs02_motion_control 不发，电机不会主动上报；这是正常的。
            // 我们手动主动读一次 mech_pos 看看 CAN 通不通。
            float mech = 0.0f;
            esp_err_t er = rs02_read_param_f32(&m, RS02_IDX_MECH_POS, &mech, pdMS_TO_TICKS(200));
            if (er == ESP_OK) {
                ESP_LOGI(TAG, "A: read_param mech_pos=%+6.3f rad (CAN OK, motor idle)", mech);
            } else if (i == 0) {
                ESP_LOGW(TAG, "A: no feedback & read_param timeout (%s) — check CAN wiring / power",
                         esp_err_to_name(er));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

#if CONFIG_KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN
    // ---------------- 阶段 B：自动 ±0.1 rad 摆动 ----------------
    ESP_LOGW(TAG, "Phase B: AUTOSPIN enabled. 3s pause, KEEP HANDS OFF MOTOR!");
    for (int i = 3; i > 0; i--) {
        ESP_LOGW(TAG, "  enabling motor in %d ...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(rs02_set_mode(&m, RS02_MODE_MOTION));
    vTaskDelay(pdMS_TO_TICKS(20));

    // 读当前位置作零参考
    rs02_feedback_t fb0 = {};
    rs02_get_feedback(&m, &fb0, pdMS_TO_TICKS(200));
    const float pos0 = fb0.position_rad;
    ESP_LOGI(TAG, "zero ref motor_pos = %+6.3f rad", pos0);

    ESP_ERROR_CHECK(rs02_enable(&m));
    vTaskDelay(pdMS_TO_TICKS(20));

    const float amp     = 0.10f;   // ±0.1 rad
    const float Kp      = 2.0f;    // 软
    const float Kd      = 0.5f;
    const int   periods = 8;
    const int   loop_hz = 100;
    const int   total_loops = periods * loop_hz;  // 8s @ 100Hz
    const TickType_t dt = pdMS_TO_TICKS(1000 / loop_hz);
    TickType_t last = xTaskGetTickCount();

    for (int i = 0; i < total_loops; i++) {
        // 1Hz 正弦摆动
        const float phase = 2.0f * (float)M_PI * (float)i / (float)loop_hz;
        const float pos_cmd = pos0 + amp * sinf(phase);
        rs02_motion_control(&m, 0.0f, pos_cmd, 0.0f, Kp, Kd);

        rs02_feedback_t fb;
        if (rs02_get_feedback(&m, &fb, 0) == ESP_OK && (i % 25 == 0)) {
            ESP_LOGI(TAG, "B: cmd=%+6.3f  motor=%+6.3f  joint=%+6.3f rad",
                     pos_cmd, fb.position_rad,
                     motor_to_joint_rad(fb.position_rad));
        }
        vTaskDelayUntil(&last, dt);
    }

    rs02_stop(&m, false);
    ESP_LOGI(TAG, "Phase B done, motor stopped.");
    ESP_LOGW(TAG, "标定结论方法：观察电机在 cmd 增大时是屈膝(+) 还是伸膝(-)。");
    ESP_LOGW(TAG, "  屈膝 => JOINT_DIR_SIGN = +1.0f (当前默认)");
    ESP_LOGW(TAG, "  伸膝 => 改 config.h JOINT_DIR_SIGN = -1.0f, 重新 build flash。");
#else
    ESP_LOGI(TAG, "AUTOSPIN disabled. Set KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN=y to spin.");
#endif

    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST

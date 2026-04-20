// test_motor_enable.cpp
// 把本文件当作 main/app_main.cpp 即可验证 RS02 驱动：
//   1. 初始化 CAN
//   2. 使能电机
//   3. 每 100ms 读一次反馈，打印 pos/vel/torque/temp
//   4. 10 秒后 rs02_stop()
// 预期：手动转动电机，pos 实时变化。

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "can_bus.h"
#include "rs02_motor.h"

using namespace ExoConfig;

static const char *TAG = "motor_test";

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(can_bus_init(PIN_TWAI_TX, PIN_TWAI_RX, CAN_BITRATE_HZ));

    rs02_handle_t m;
    ESP_ERROR_CHECK(rs02_init(&m, RS02_CAN_ID, RS02_HOST_ID));
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(rs02_set_mode(&m, RS02_MODE_MOTION));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(rs02_enable(&m));

    for (int i = 0; i < 100; i++) {
        // 软软的阻抗，观测跟踪
        rs02_motion_control(&m, 0.0f, 0.0f, 0.0f, 2.0f, 0.5f);

        rs02_feedback_t fb;
        if (rs02_get_feedback(&m, &fb, pdMS_TO_TICKS(50)) == ESP_OK) {
            ESP_LOGI(TAG, "pos=%+6.3f vel=%+6.3f tq=%+5.2f T=%4.1f mode=%d%s",
                     fb.position_rad, fb.velocity_rad_s,
                     fb.torque_nm, fb.temperature_c,
                     fb.mode_state, fb.has_fault ? " FAULT!" : "");
        } else {
            ESP_LOGW(TAG, "no feedback #%d", i);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    rs02_stop(&m, false);
    ESP_LOGI(TAG, "motor stopped");
}

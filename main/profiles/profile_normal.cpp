// profile_normal.cpp
// 正式形态：CAN -> IMU -> RS02 init/enable -> 启动 100Hz 控制 task
//
// 注意：穿戴调试模式会先做用户伸膝零位平均，再使能电机。

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_NORMAL

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>

#include "config.h"
#include "can_bus.h"
#include "rs02_motor.h"
#include "witmotion_imu.h"
#include "app_main.hpp"

using namespace ExoConfig;

static const char *TAG = "profile.normal";

// 全局电机句柄定义（其他 profile 不引用）
rs02_handle_t g_motor;
float g_user_zero_offset_rad = 0.0f;

static void calibrate_user_zero(void)
{
    const int calib_ms = CONFIG_KNEEEXO_USER_ZERO_CALIB_MS;
    if (calib_ms <= 0) {
        ESP_LOGW(TAG, "User zero calibration skipped (CONFIG_KNEEEXO_USER_ZERO_CALIB_MS=0).");
        return;
    }

    ESP_LOGW(TAG, "USER ZERO: wear the exoskeleton, keep knee naturally extended.");
    for (int i = 3; i > 0; i--) {
        ESP_LOGW(TAG, "  sampling starts in %d ...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    const int sample_period_ms = 50;
    const int sample_count = calib_ms / sample_period_ms;
    float sum_joint = 0.0f;
    int ok_count = 0;

    for (int i = 0; i < sample_count; i++) {
        float motor_pos = 0.0f;
        if (rs02_read_param_f32(&g_motor, RS02_IDX_MECH_POS, &motor_pos, pdMS_TO_TICKS(100)) == ESP_OK) {
            sum_joint += motor_to_joint_rad(motor_pos);
            ok_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(sample_period_ms));
    }

    if (ok_count > 0) {
        g_user_zero_offset_rad = sum_joint / (float)ok_count;
        ESP_LOGW(TAG,
                 "USER ZERO done: offset=%+.4f rad (%+.1f deg), samples=%d. "
                 "Runtime joint = mechanical_joint - user_zero.",
                 g_user_zero_offset_rad,
                 g_user_zero_offset_rad * 180.0f / (float)M_PI,
                 ok_count);
    } else {
        ESP_LOGE(TAG, "USER ZERO failed: no motor position samples. Continue with offset=0.");
        g_user_zero_offset_rad = 0.0f;
    }
}

extern "C" void profile_normal_main(void)
{
    ESP_ERROR_CHECK(can_bus_init(PIN_TWAI_TX, PIN_TWAI_RX, CAN_BITRATE_HZ));

    ESP_ERROR_CHECK(witmotion_init(IMU1_UART_NUM,
                                   PIN_IMU1_MCU_TX,
                                   PIN_IMU1_MCU_RX,
                                   IMU_UART_BAUD));

    ESP_ERROR_CHECK(rs02_init(&g_motor, RS02_CAN_ID, RS02_HOST_ID));
    vTaskDelay(pdMS_TO_TICKS(200));

    calibrate_user_zero();

    ESP_ERROR_CHECK(rs02_set_mode(&g_motor, RS02_MODE_MOTION));
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_ERROR_CHECK(rs02_enable(&g_motor));
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "Motor enabled, starting control task...");
    xTaskCreatePinnedToCore(control_task,
                            "control_task",
                            8192,
                            NULL,
                            configMAX_PRIORITIES - 4,
                            NULL,
                            1);
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_NORMAL

// profile_normal.cpp
// 正式形态：CAN -> IMU -> RS02 init/enable -> 启动 100Hz 控制 task
//
// 注意：上电立即使能电机！请确认硬件、限位、急停就绪再烧此 profile。

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_NORMAL

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "config.h"
#include "can_bus.h"
#include "rs02_motor.h"
#include "witmotion_imu.h"
#include "app_main.hpp"

using namespace ExoConfig;

static const char *TAG = "profile.normal";

// 全局电机句柄定义（其他 profile 不引用）
rs02_handle_t g_motor;

extern "C" void profile_normal_main(void)
{
    ESP_ERROR_CHECK(can_bus_init(PIN_TWAI_TX, PIN_TWAI_RX, CAN_BITRATE_HZ));

    ESP_ERROR_CHECK(witmotion_init(IMU1_UART_NUM,
                                   PIN_IMU1_MCU_TX,
                                   PIN_IMU1_MCU_RX,
                                   IMU_UART_BAUD));

    ESP_ERROR_CHECK(rs02_init(&g_motor, RS02_CAN_ID, RS02_HOST_ID));
    vTaskDelay(pdMS_TO_TICKS(200));

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

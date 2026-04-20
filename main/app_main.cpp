// app_main.cpp - KneeExo 入口
// 初始化 CAN / RS02 / WitMotion IMU，启动 100Hz 控制任务。

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

static const char *TAG = "app_main";

// 电机句柄，app_main 与 control_task 共享
rs02_handle_t g_motor;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "KneeExo boot, IDF " IDF_VER);

    // 1. CAN 总线初始化 (TWAI, 1Mbps)
    ESP_ERROR_CHECK(can_bus_init(PIN_TWAI_TX, PIN_TWAI_RX, CAN_BITRATE_HZ));

    // 2. IMU1 (UART1, GPIO42/41, 115200)
    ESP_ERROR_CHECK(witmotion_init(IMU1_UART_NUM,
                                   PIN_IMU1_MCU_TX,
                                   PIN_IMU1_MCU_RX,
                                   IMU_UART_BAUD));

    // 3. RS02 电机驱动初始化
    ESP_ERROR_CHECK(rs02_init(&g_motor, RS02_CAN_ID, RS02_HOST_ID));

    vTaskDelay(pdMS_TO_TICKS(200));

    // 4. 切到运控模式
    ESP_ERROR_CHECK(rs02_set_mode(&g_motor, RS02_MODE_MOTION));
    vTaskDelay(pdMS_TO_TICKS(20));

    // 5. 使能电机
    ESP_ERROR_CHECK(rs02_enable(&g_motor));
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "Motor enabled, starting control task...");

    // 6. 启动 100Hz 控制任务，钉在核心 1
    xTaskCreatePinnedToCore(control_task,
                            "control_task",
                            8192,
                            NULL,
                            configMAX_PRIORITIES - 4,
                            NULL,
                            1);
}

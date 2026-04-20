// test_imu_rx.cpp
// 把本文件当作 main/app_main.cpp 即可验证 IMU 驱动：
//   1. 初始化 UART
//   2. 每 200ms 打印一次姿态
// 预期：摆动模块，pitch/roll/yaw 数字同步变化；频率 ~5Hz。

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "witmotion_imu.h"

using namespace ExoConfig;

static const char *TAG = "imu_test";

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(witmotion_init(IMU1_UART_NUM,
                                   PIN_IMU1_MCU_TX,
                                   PIN_IMU1_MCU_RX,
                                   IMU_UART_BAUD));

    witmotion_data_t d;
    for (int i = 0; i < 50; i++) {
        if (witmotion_get_latest(&d) == ESP_OK) {
            ESP_LOGI(TAG,
                     "acc=[%+5.2f %+5.2f %+5.2f]g  gyro=[%+7.1f %+7.1f %+7.1f]dps  "
                     "euler=[%+6.2f %+6.2f %+6.2f]  T=%4.1fC  frames=%lu",
                     d.accel_g[0], d.accel_g[1], d.accel_g[2],
                     d.gyro_dps[0], d.gyro_dps[1], d.gyro_dps[2],
                     d.euler_deg[0], d.euler_deg[1], d.euler_deg[2],
                     d.temp_c, (unsigned long)d.frame_count);
        } else {
            ESP_LOGW(TAG, "no IMU data yet #%d", i);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// test_twai_loopback.cpp
// 把本文件当作 main/app_main.cpp 即可验证 TWAI 驱动：
//   - 以 TWAI_MODE_NO_ACK + loopback 的方式发送一帧并立刻收回
//   - 打印成功次数
// 使用注意：此测试不挂真正的电机，不接收发器也能跑（自回环）。

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

#include "config.h"

using namespace ExoConfig;

static const char *TAG = "twai_test";

extern "C" void app_main(void)
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_TWAI_TX, PIN_TWAI_RX, TWAI_MODE_NO_ACK);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());

    uint32_t ok = 0, fail = 0;
    for (int i = 0; i < 50; i++) {
        twai_message_t tx = {};
        tx.extd = 1;
        tx.identifier = 0x12345678;
        tx.data_length_code = 8;
        for (int k = 0; k < 8; k++) tx.data[k] = i + k;
        tx.self = 1;  // 自发自收
        if (twai_transmit(&tx, pdMS_TO_TICKS(50)) != ESP_OK) { fail++; continue; }

        twai_message_t rx;
        if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK) { fail++; continue; }

        if (rx.identifier == tx.identifier && rx.data[0] == tx.data[0]) ok++;
        else fail++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(TAG, "TWAI loopback result: ok=%lu fail=%lu",
             (unsigned long)ok, (unsigned long)fail);
}

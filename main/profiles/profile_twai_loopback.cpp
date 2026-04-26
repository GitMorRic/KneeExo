// profile_twai_loopback.cpp
// TWAI 自发自收自检：
//   - TWAI_MODE_NO_ACK + msg.self = 1
//   - 不需要外部 CAN 收发器或电机也能跑
//   - 50 帧 / 1Mbps，输出 ok / fail 计数
//
// 通过即说明：CAN 控制器活、引脚配置正确、波特率正确。

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_TWAI_LOOPBACK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

#include "config.h"
#include "app_main.hpp"

using namespace ExoConfig;

static const char *TAG = "profile.twai_lb";

extern "C" void profile_twai_loopback_main(void)
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_TWAI_TX, PIN_TWAI_RX, TWAI_MODE_NO_ACK);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started @1Mbps, sending self-loopback frames...");

    uint32_t ok = 0, fail = 0;
    for (int i = 0; i < 50; i++) {
        twai_message_t tx = {};
        tx.extd = 1;
        tx.identifier = 0x12345678;
        tx.data_length_code = 8;
        for (int k = 0; k < 8; k++) tx.data[k] = (uint8_t)(i + k);
        tx.self = 1;

        if (twai_transmit(&tx, pdMS_TO_TICKS(50)) != ESP_OK) { fail++; continue; }

        twai_message_t rx;
        if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK) { fail++; continue; }

        if (rx.identifier == tx.identifier && rx.data[0] == tx.data[0]) {
            ok++;
        } else {
            fail++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGI(TAG, "TWAI loopback finished: ok=%lu fail=%lu",
             (unsigned long)ok, (unsigned long)fail);

    twai_stop();
    twai_driver_uninstall();
    ESP_LOGI(TAG, "Done. (idle, you can power off)");

    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_TWAI_LOOPBACK

// can_bus.c - ESP32-S3 TWAI 薄封装
#include "can_bus.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "can_bus";

static bool s_inited = false;

static twai_timing_config_t bitrate_to_timing(uint32_t hz)
{
    switch (hz) {
    case 1000000: return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
    case 500000:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
    case 250000:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
    case 125000:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
    default:      return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
    }
}

esp_err_t can_bus_init(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate_hz)
{
    if (s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (bitrate_hz != 125000 && bitrate_hz != 250000 &&
        bitrate_hz != 500000 && bitrate_hz != 1000000) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 16;
    g_config.rx_queue_len = 32;
    // 默认中断分配即可

    twai_timing_config_t t_config = bitrate_to_timing(bitrate_hz);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return err;
    }

    s_inited = true;
    ESP_LOGI(TAG, "TWAI started: TX=%d RX=%d bitrate=%lu",
             (int)tx, (int)rx, (unsigned long)bitrate_hz);
    return ESP_OK;
}

esp_err_t can_bus_tx(const twai_message_t *msg, TickType_t timeout)
{
    if (!s_inited || msg == NULL) return ESP_ERR_INVALID_STATE;
    return twai_transmit(msg, timeout);
}

esp_err_t can_bus_rx(twai_message_t *msg, TickType_t timeout)
{
    if (!s_inited || msg == NULL) return ESP_ERR_INVALID_STATE;
    return twai_receive(msg, timeout);
}

void can_bus_log_status(void)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
        ESP_LOGI(TAG, "state=%d tx_err=%d rx_err=%d tx_failed=%lu rx_missed=%lu",
                 (int)st.state, (int)st.tx_error_counter,
                 (int)st.rx_error_counter,
                 (unsigned long)st.tx_failed_count,
                 (unsigned long)st.rx_missed_count);
    }
}

esp_err_t can_bus_deinit(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    twai_stop();
    twai_driver_uninstall();
    s_inited = false;
    return ESP_OK;
}

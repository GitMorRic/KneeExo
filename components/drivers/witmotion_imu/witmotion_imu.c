// witmotion_imu.c
// 参考：WitMotion JY901 标准协议 / WT-IMU63 规格书
#include "witmotion_imu.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "witmotion";

#define IMU_FRAME_LEN   11
#define IMU_FRAME_HEAD  0x55

#define KIND_ACC        0x51
#define KIND_GYRO       0x52
#define KIND_ANGLE      0x53
#define KIND_QUAT       0x59

static witmotion_data_t  s_data;
static SemaphoreHandle_t s_mtx  = NULL;
static TaskHandle_t      s_task = NULL;
static uart_port_t       s_port = UART_NUM_MAX;
static bool              s_alive = false;

static inline int16_t i16le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void parse_frame(const uint8_t *f)
{
    const uint8_t kind = f[1];
    const uint8_t *d   = &f[2];   // 8 bytes 数据区
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    switch (kind) {
    case KIND_ACC: {
        s_data.accel_g[0] = (float)i16le(d + 0) / 32768.0f * 16.0f;
        s_data.accel_g[1] = (float)i16le(d + 2) / 32768.0f * 16.0f;
        s_data.accel_g[2] = (float)i16le(d + 4) / 32768.0f * 16.0f;
        s_data.temp_c     = (float)i16le(d + 6) / 100.0f;
        break;
    }
    case KIND_GYRO: {
        s_data.gyro_dps[0] = (float)i16le(d + 0) / 32768.0f * 2000.0f;
        s_data.gyro_dps[1] = (float)i16le(d + 2) / 32768.0f * 2000.0f;
        s_data.gyro_dps[2] = (float)i16le(d + 4) / 32768.0f * 2000.0f;
        s_data.temp_c      = (float)i16le(d + 6) / 100.0f;
        break;
    }
    case KIND_ANGLE: {
        s_data.euler_deg[0] = (float)i16le(d + 0) / 32768.0f * 180.0f;
        s_data.euler_deg[1] = (float)i16le(d + 2) / 32768.0f * 180.0f;
        s_data.euler_deg[2] = (float)i16le(d + 4) / 32768.0f * 180.0f;
        s_data.ts_us = esp_timer_get_time();
        break;
    }
    case KIND_QUAT: {
        s_data.quat[0] = (float)i16le(d + 0) / 32768.0f;
        s_data.quat[1] = (float)i16le(d + 2) / 32768.0f;
        s_data.quat[2] = (float)i16le(d + 4) / 32768.0f;
        s_data.quat[3] = (float)i16le(d + 6) / 32768.0f;
        break;
    }
    default:
        break;
    }
    s_data.frame_count++;
    s_alive = true;
    xSemaphoreGive(s_mtx);
}

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[IMU_FRAME_LEN];
    size_t  filled = 0;

    while (1) {
        // 确保 buf[0] 是 0x55
        if (filled == 0) {
            int n = uart_read_bytes(s_port, &buf[0], 1, portMAX_DELAY);
            if (n != 1) continue;
            if (buf[0] != IMU_FRAME_HEAD) continue;
            filled = 1;
        }

        // 再读 10 字节凑齐整帧
        while (filled < IMU_FRAME_LEN) {
            int n = uart_read_bytes(s_port,
                                    &buf[filled],
                                    IMU_FRAME_LEN - filled,
                                    pdMS_TO_TICKS(100));
            if (n <= 0) {
                // 超时，丢掉重来
                filled = 0;
                break;
            }
            filled += n;
        }
        if (filled < IMU_FRAME_LEN) continue;

        // checksum
        uint8_t sum = 0;
        for (int i = 0; i < IMU_FRAME_LEN - 1; i++) sum += buf[i];
        if (sum != buf[IMU_FRAME_LEN - 1]) {
            ESP_LOGV(TAG, "bad checksum kind=%02X sum=%02X expect=%02X",
                     buf[1], sum, buf[IMU_FRAME_LEN - 1]);
            // 丢掉一个字节，尝试重新同步 (移位查找下一个 0x55)
            for (int i = 1; i < IMU_FRAME_LEN; i++) {
                if (buf[i] == IMU_FRAME_HEAD) {
                    int remain = IMU_FRAME_LEN - i;
                    memmove(buf, &buf[i], remain);
                    filled = remain;
                    goto next;
                }
            }
            filled = 0;
            continue;
        }

        parse_frame(buf);
        filled = 0;
next: ;
    }
}

esp_err_t witmotion_init(uart_port_t port,
                         gpio_num_t  tx_pin,
                         gpio_num_t  rx_pin,
                         int         baud)
{
    if (s_task) return ESP_ERR_INVALID_STATE;

    s_port = port;
    s_mtx  = xSemaphoreCreateMutex();
    if (!s_mtx) return ESP_ERR_NO_MEM;

    const uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    const int rx_buf = 512;
    esp_err_t err = uart_driver_install(port, rx_buf, 0, 0, NULL, 0);
    if (err != ESP_OK) goto fail;
    err = uart_param_config(port, &cfg);
    if (err != ESP_OK) goto fail;
    err = uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) goto fail;

    BaseType_t ok = xTaskCreatePinnedToCore(rx_task, "imu_rx",
                                            4096, NULL,
                                            configMAX_PRIORITIES - 5,
                                            &s_task, 0);
    if (ok != pdPASS) { err = ESP_FAIL; goto fail; }

    ESP_LOGI(TAG, "WitMotion IMU UART%d ready, TX=%d RX=%d baud=%d",
             (int)port, (int)tx_pin, (int)rx_pin, baud);
    return ESP_OK;

fail:
    vSemaphoreDelete(s_mtx);
    s_mtx = NULL;
    return err;
}

esp_err_t witmotion_get_latest(witmotion_data_t *out)
{
    if (!out || !s_mtx) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    *out = s_data;
    xSemaphoreGive(s_mtx);
    return s_alive ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool witmotion_is_alive(void)
{
    return s_alive;
}

// witmotion_imu.c
// WitMotion WT-IMU63/JY901-compatible UART parser.
// This implementation supports multiple UART channels at the same time.

#include "witmotion_imu.h"

#include <stdio.h>
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

typedef struct {
    witmotion_data_t  data;
    SemaphoreHandle_t mtx;
    TaskHandle_t      task;
    uart_port_t       port;
    bool              alive;
} witmotion_ctx_t;

static witmotion_ctx_t s_ctx[UART_NUM_MAX];
static uart_port_t s_default_port = UART_NUM_MAX;

static inline int16_t i16le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static witmotion_ctx_t *ctx_from_port(uart_port_t port)
{
    if (port < 0 || port >= UART_NUM_MAX) {
        return NULL;
    }
    return &s_ctx[port];
}

static void parse_frame(witmotion_ctx_t *ctx, const uint8_t *f)
{
    const uint8_t kind = f[1];
    const uint8_t *d = &f[2];

    xSemaphoreTake(ctx->mtx, portMAX_DELAY);
    switch (kind) {
    case KIND_ACC:
        ctx->data.accel_g[0] = (float)i16le(d + 0) / 32768.0f * 16.0f;
        ctx->data.accel_g[1] = (float)i16le(d + 2) / 32768.0f * 16.0f;
        ctx->data.accel_g[2] = (float)i16le(d + 4) / 32768.0f * 16.0f;
        ctx->data.temp_c     = (float)i16le(d + 6) / 100.0f;
        break;
    case KIND_GYRO:
        ctx->data.gyro_dps[0] = (float)i16le(d + 0) / 32768.0f * 2000.0f;
        ctx->data.gyro_dps[1] = (float)i16le(d + 2) / 32768.0f * 2000.0f;
        ctx->data.gyro_dps[2] = (float)i16le(d + 4) / 32768.0f * 2000.0f;
        ctx->data.temp_c      = (float)i16le(d + 6) / 100.0f;
        break;
    case KIND_ANGLE:
        ctx->data.euler_deg[0] = (float)i16le(d + 0) / 32768.0f * 180.0f;
        ctx->data.euler_deg[1] = (float)i16le(d + 2) / 32768.0f * 180.0f;
        ctx->data.euler_deg[2] = (float)i16le(d + 4) / 32768.0f * 180.0f;
        ctx->data.ts_us = esp_timer_get_time();
        break;
    case KIND_QUAT:
        ctx->data.quat[0] = (float)i16le(d + 0) / 32768.0f;
        ctx->data.quat[1] = (float)i16le(d + 2) / 32768.0f;
        ctx->data.quat[2] = (float)i16le(d + 4) / 32768.0f;
        ctx->data.quat[3] = (float)i16le(d + 6) / 32768.0f;
        break;
    default:
        break;
    }
    ctx->data.frame_count++;
    ctx->alive = true;
    xSemaphoreGive(ctx->mtx);
}

static void rx_task(void *arg)
{
    witmotion_ctx_t *ctx = (witmotion_ctx_t *)arg;
    uint8_t buf[IMU_FRAME_LEN];
    size_t filled = 0;

    while (true) {
        if (filled == 0) {
            int n = uart_read_bytes(ctx->port, &buf[0], 1, portMAX_DELAY);
            if (n != 1) continue;
            if (buf[0] != IMU_FRAME_HEAD) continue;
            filled = 1;
        }

        while (filled < IMU_FRAME_LEN) {
            int n = uart_read_bytes(ctx->port,
                                    &buf[filled],
                                    IMU_FRAME_LEN - filled,
                                    pdMS_TO_TICKS(100));
            if (n <= 0) {
                filled = 0;
                break;
            }
            filled += n;
        }
        if (filled < IMU_FRAME_LEN) continue;

        uint8_t sum = 0;
        for (int i = 0; i < IMU_FRAME_LEN - 1; i++) {
            sum += buf[i];
        }
        if (sum != buf[IMU_FRAME_LEN - 1]) {
            ESP_LOGV(TAG, "UART%d bad checksum kind=%02X sum=%02X expect=%02X",
                     (int)ctx->port, buf[1], sum, buf[IMU_FRAME_LEN - 1]);
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

        parse_frame(ctx, buf);
        filled = 0;
next:
        ;
    }
}

esp_err_t witmotion_init(uart_port_t port,
                         gpio_num_t tx_pin,
                         gpio_num_t rx_pin,
                         int baud)
{
    esp_err_t err = witmotion_init_channel(port, tx_pin, rx_pin, baud);
    if (err == ESP_OK) {
        s_default_port = port;
    }
    return err;
}

esp_err_t witmotion_init_channel(uart_port_t port,
                                 gpio_num_t tx_pin,
                                 gpio_num_t rx_pin,
                                 int baud)
{
    witmotion_ctx_t *ctx = ctx_from_port(port);
    if (!ctx) return ESP_ERR_INVALID_ARG;
    if (ctx->task) return ESP_ERR_INVALID_STATE;

    memset(ctx, 0, sizeof(*ctx));
    ctx->port = port;
    ctx->mtx = xSemaphoreCreateMutex();
    if (!ctx->mtx) return ESP_ERR_NO_MEM;

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

    char task_name[16];
    snprintf(task_name, sizeof(task_name), "imu_rx_%d", (int)port);
    BaseType_t ok = xTaskCreatePinnedToCore(rx_task, task_name,
                                            4096, ctx,
                                            configMAX_PRIORITIES - 5,
                                            &ctx->task, 0);
    if (ok != pdPASS) {
        err = ESP_FAIL;
        goto fail;
    }

    ESP_LOGI(TAG, "WitMotion IMU UART%d ready, TX=%d RX=%d baud=%d",
             (int)port, (int)tx_pin, (int)rx_pin, baud);
    return ESP_OK;

fail:
    uart_driver_delete(port);
    if (ctx->mtx) {
        vSemaphoreDelete(ctx->mtx);
    }
    memset(ctx, 0, sizeof(*ctx));
    return err;
}

esp_err_t witmotion_get_latest(witmotion_data_t *out)
{
    if (s_default_port == UART_NUM_MAX) return ESP_ERR_INVALID_STATE;
    return witmotion_get_latest_channel(s_default_port, out);
}

esp_err_t witmotion_get_latest_channel(uart_port_t port, witmotion_data_t *out)
{
    witmotion_ctx_t *ctx = ctx_from_port(port);
    if (!ctx || !out || !ctx->mtx) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(ctx->mtx, portMAX_DELAY);
    *out = ctx->data;
    xSemaphoreGive(ctx->mtx);
    return ctx->alive ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool witmotion_is_alive(void)
{
    if (s_default_port == UART_NUM_MAX) return false;
    return witmotion_is_alive_channel(s_default_port);
}

bool witmotion_is_alive_channel(uart_port_t port)
{
    witmotion_ctx_t *ctx = ctx_from_port(port);
    return ctx && ctx->alive;
}

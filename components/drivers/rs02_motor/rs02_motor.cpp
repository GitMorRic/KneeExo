// rs02_motor.cpp
// 参考：RS02 使用说明书 260410 第 4 章 "私有通信协议"
//
// 注意：config.h 使用 C++ namespace ExoConfig，故本文件为 .cpp，
//       并在文件作用域 using namespace ExoConfig 以简化调用。
#include "rs02_motor.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "can_bus.h"
#include "config.h"

using namespace ExoConfig;

static const char *TAG = "rs02";

// =============================================================================
// 内部：多电机注册表 + 后台 rx_task
// =============================================================================
#ifndef RS02_MAX_MOTORS
#define RS02_MAX_MOTORS 4
#endif

static rs02_handle_t *s_motors[RS02_MAX_MOTORS];
static SemaphoreHandle_t s_reg_mtx = NULL;
static TaskHandle_t s_rx_task = NULL;

// =============================================================================
// 编码工具：float <-> uint16 (大端序写入)
// 照抄说明书 4.4 float_to_uint
// =============================================================================
static inline uint16_t f_to_u16(float x, float xmin, float xmax)
{
    if (x > xmax) x = xmax;
    else if (x < xmin) x = xmin;
    float span = xmax - xmin;
    return (uint16_t)((x - xmin) * 65535.0f / span);
}

static inline float u16_to_f(uint16_t v, float xmin, float xmax)
{
    float span = xmax - xmin;
    return ((float)v) * span / 65535.0f + xmin;
}

// =============================================================================
// 扩展帧 ID 打包：
//   bit[28:24] = type
//   bit[23:8]  = data
//   bit[7:0]   = target_motor_id
// =============================================================================
static inline uint32_t make_ext_id(uint8_t type, uint16_t data, uint8_t motor_id)
{
    return ((uint32_t)(type & 0x1F) << 24)
         | ((uint32_t)data          << 8)
         | ((uint32_t)motor_id);
}

static inline void unpack_ext_id(uint32_t ext_id,
                                 uint8_t *type,
                                 uint16_t *data,
                                 uint8_t *target_id)
{
    if (type)      *type      = (ext_id >> 24) & 0x1F;
    if (data)      *data      = (ext_id >> 8) & 0xFFFF;
    if (target_id) *target_id = ext_id & 0xFF;
}

static esp_err_t send_ext_frame(uint8_t type, uint16_t data_field,
                                uint8_t motor_id,
                                const uint8_t payload[8])
{
    twai_message_t msg = {};
    msg.extd = 1;           // 29-bit
    msg.rtr  = 0;
    msg.ss   = 0;
    msg.self = 0;
    msg.dlc_non_comp = 0;
    msg.data_length_code = 8;
    msg.identifier = make_ext_id(type, data_field, motor_id);
    if (payload) {
        memcpy(msg.data, payload, 8);
    } else {
        memset(msg.data, 0, 8);
    }
    return can_bus_tx(&msg, pdMS_TO_TICKS(20));
}

// =============================================================================
// 接收解析：通信类型 2 (反馈) / 21 (故障)
// =============================================================================
static void parse_type2_feedback(rs02_handle_t *h,
                                 uint16_t data_field,
                                 const uint8_t *d)
{
    rs02_feedback_t fb = {};
    uint16_t pos_raw = ((uint16_t)d[0] << 8) | d[1];
    uint16_t vel_raw = ((uint16_t)d[2] << 8) | d[3];
    uint16_t tor_raw = ((uint16_t)d[4] << 8) | d[5];
    uint16_t tmp_raw = ((uint16_t)d[6] << 8) | d[7];

    fb.position_rad   = u16_to_f(pos_raw, MOTOR_MIN_POS,    MOTOR_MAX_POS);
    fb.velocity_rad_s = u16_to_f(vel_raw, MOTOR_MIN_SPEED,  MOTOR_MAX_SPEED);
    fb.torque_nm      = u16_to_f(tor_raw, MOTOR_MIN_TORQUE, MOTOR_MAX_TORQUE);
    fb.temperature_c  = (float)tmp_raw * 0.1f;

    fb.mode_state  = (data_field >> 14) & 0x03;
    fb.fault_bits  = (data_field >> 8) & 0x3F;
    fb.has_fault   = fb.fault_bits != 0;
    fb.ts_us       = esp_timer_get_time();

    h->last_feedback = fb;
    h->has_feedback  = true;

    if (h->rx_queue) {
        if (xQueueSend(h->rx_queue, &fb, 0) != pdTRUE) {
            rs02_feedback_t dump;
            xQueueReceive(h->rx_queue, &dump, 0);
            xQueueSend(h->rx_queue, &fb, 0);
        }
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    twai_message_t msg;
    while (1) {
        esp_err_t err = can_bus_rx(&msg, portMAX_DELAY);
        if (err != ESP_OK) {
            continue;
        }
        if (!msg.extd) {
            continue;
        }
        uint8_t type; uint16_t data; uint8_t target;
        unpack_ext_id(msg.identifier, &type, &data, &target);

        uint8_t motor_id = (data >> 8) & 0xFF;

        xSemaphoreTake(s_reg_mtx, portMAX_DELAY);
        rs02_handle_t *h = NULL;
        for (int i = 0; i < RS02_MAX_MOTORS; i++) {
            if (s_motors[i] && s_motors[i]->motor_can_id == motor_id) {
                h = s_motors[i];
                break;
            }
        }
        xSemaphoreGive(s_reg_mtx);
        if (!h) continue;

        switch (type) {
        case 2:
            parse_type2_feedback(h, data, msg.data);
            break;
        case 21:
            ESP_LOGW(TAG, "motor 0x%02X fault: %02X%02X%02X%02X warn: %02X%02X%02X%02X",
                     motor_id, msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                     msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
            break;
        case 17: {
            uint16_t resp_idx = ((uint16_t)msg.data[1] << 8) | msg.data[0];
            ESP_LOGD(TAG, "read_param resp idx=0x%04X raw=%02X%02X%02X%02X",
                     resp_idx, msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
            if (resp_idx == h->pending_param_idx && h->param_resp_sem) {
                memcpy(h->param_resp_raw, &msg.data[4], 4);
                xSemaphoreGive(h->param_resp_sem);
            }
            break;
        }
        default:
            break;
        }
    }
}

static esp_err_t register_motor(rs02_handle_t *h)
{
    if (!s_reg_mtx) {
        s_reg_mtx = xSemaphoreCreateMutex();
        if (!s_reg_mtx) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_reg_mtx, portMAX_DELAY);
    esp_err_t err = ESP_ERR_NO_MEM;
    for (int i = 0; i < RS02_MAX_MOTORS; i++) {
        if (!s_motors[i]) {
            s_motors[i] = h;
            err = ESP_OK;
            break;
        }
    }
    xSemaphoreGive(s_reg_mtx);

    if (err == ESP_OK && !s_rx_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(rx_task,
                                                "rs02_rx",
                                                4096, NULL,
                                                configMAX_PRIORITIES - 3,
                                                &s_rx_task, 0);
        if (ok != pdPASS) err = ESP_FAIL;
    }
    return err;
}

// =============================================================================
// Public API
// =============================================================================
esp_err_t rs02_init(rs02_handle_t *h, uint8_t motor_id, uint8_t host_id)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    memset(h, 0, sizeof(*h));
    h->motor_can_id    = motor_id;
    h->host_can_id     = host_id;
    h->rx_queue        = xQueueCreate(4, sizeof(rs02_feedback_t));
    h->param_resp_sem  = xSemaphoreCreateBinary();
    if (!h->rx_queue || !h->param_resp_sem) {
        if (h->rx_queue)       vQueueDelete(h->rx_queue);
        if (h->param_resp_sem) vSemaphoreDelete(h->param_resp_sem);
        h->rx_queue = NULL; h->param_resp_sem = NULL;
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = register_motor(h);
    if (err != ESP_OK) {
        vQueueDelete(h->rx_queue);
        vSemaphoreDelete(h->param_resp_sem);
        h->rx_queue = NULL; h->param_resp_sem = NULL;
        return err;
    }
    ESP_LOGI(TAG, "rs02 init: motor_id=0x%02X host_id=0x%02X", motor_id, host_id);
    return ESP_OK;
}

esp_err_t rs02_enable(rs02_handle_t *h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    uint8_t zeros[8] = {0};
    esp_err_t err = send_ext_frame(3, h->host_can_id, h->motor_can_id, zeros);
    if (err == ESP_OK) h->enabled = true;
    return err;
}

esp_err_t rs02_stop(rs02_handle_t *h, bool clear_fault)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    uint8_t payload[8] = {0};
    if (clear_fault) payload[0] = 1;
    esp_err_t err = send_ext_frame(4, h->host_can_id, h->motor_can_id, payload);
    if (err == ESP_OK) h->enabled = false;
    return err;
}

esp_err_t rs02_set_zero(rs02_handle_t *h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    uint8_t payload[8] = {0};
    payload[0] = 1;
    return send_ext_frame(6, h->host_can_id, h->motor_can_id, payload);
}

esp_err_t rs02_set_mode(rs02_handle_t *h, rs02_mode_t mode)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    esp_err_t err = rs02_write_param_u8(h, RS02_IDX_RUN_MODE, (uint8_t)mode);
    if (err == ESP_OK) h->mode = mode;
    return err;
}

esp_err_t rs02_motion_control(rs02_handle_t *h,
                              float torque_ff,
                              float pos,
                              float vel,
                              float kp,
                              float kd)
{
    if (!h) return ESP_ERR_INVALID_ARG;

    uint16_t tor = f_to_u16(torque_ff, MOTOR_MIN_TORQUE, MOTOR_MAX_TORQUE);
    uint16_t p   = f_to_u16(pos,       MOTOR_MIN_POS,    MOTOR_MAX_POS);
    uint16_t v   = f_to_u16(vel,       MOTOR_MIN_SPEED,  MOTOR_MAX_SPEED);
    uint16_t kp_ = f_to_u16(kp,        MOTOR_MIN_KP,     MOTOR_MAX_KP);
    uint16_t kd_ = f_to_u16(kd,        MOTOR_MIN_KD,     MOTOR_MAX_KD);

    uint8_t payload[8];
    payload[0] = (p   >> 8) & 0xFF;
    payload[1] =  p        & 0xFF;
    payload[2] = (v   >> 8) & 0xFF;
    payload[3] =  v        & 0xFF;
    payload[4] = (kp_ >> 8) & 0xFF;
    payload[5] =  kp_      & 0xFF;
    payload[6] = (kd_ >> 8) & 0xFF;
    payload[7] =  kd_      & 0xFF;

    return send_ext_frame(1, tor, h->motor_can_id, payload);
}

esp_err_t rs02_write_param_f32(rs02_handle_t *h, uint16_t index, float value)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    uint8_t payload[8] = {0};
    payload[0] = index & 0xFF;
    payload[1] = (index >> 8) & 0xFF;
    memcpy(&payload[4], &value, 4);
    return send_ext_frame(18, h->host_can_id, h->motor_can_id, payload);
}

esp_err_t rs02_write_param_u8(rs02_handle_t *h, uint16_t index, uint8_t value)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    uint8_t payload[8] = {0};
    payload[0] = index & 0xFF;
    payload[1] = (index >> 8) & 0xFF;
    payload[4] = value;
    return send_ext_frame(18, h->host_can_id, h->motor_can_id, payload);
}

esp_err_t rs02_read_param_f32(rs02_handle_t *h, uint16_t index, float *out, TickType_t timeout)
{
    if (!h || !out) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(h->param_resp_sem, 0);
    h->pending_param_idx = index;

    uint8_t payload[8] = {0};
    payload[0] = index & 0xFF;
    payload[1] = (index >> 8) & 0xFF;
    esp_err_t err = send_ext_frame(17, h->host_can_id, h->motor_can_id, payload);
    if (err != ESP_OK) return err;

    if (xSemaphoreTake(h->param_resp_sem, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    memcpy(out, h->param_resp_raw, 4);
    return ESP_OK;
}

esp_err_t rs02_get_feedback(rs02_handle_t *h, rs02_feedback_t *out, TickType_t timeout)
{
    if (!h || !out) return ESP_ERR_INVALID_ARG;
    if (timeout == 0) {
        if (!h->has_feedback) return ESP_ERR_NOT_FOUND;
        *out = h->last_feedback;
        return ESP_OK;
    }
    if (xQueueReceive(h->rx_queue, out, timeout) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

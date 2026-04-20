// can_bus.h
// 薄封装 ESP32-S3 TWAI(CAN 2.0B) 控制器：
//   - 初始化统一入口，封装 general/timing/filter config + driver install + start
//   - 发/收两个阻塞接口
// 多个上层驱动（如 RS02 电机驱动）共享同一条总线，按 CAN ID 自行分发。
#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化并启动 TWAI (CAN) 控制器
 *
 * @param tx        TX 引脚（连 CAN 收发器的 TXD）
 * @param rx        RX 引脚（连 CAN 收发器的 RXD）
 * @param bitrate_hz 支持 125k/250k/500k/1M。其它值会返回 ESP_ERR_INVALID_ARG。
 *
 * @note 若已初始化过会返回 ESP_ERR_INVALID_STATE。
 */
esp_err_t can_bus_init(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate_hz);

/**
 * @brief 发送一帧，阻塞等待入队 (最终是否上线由 TWAI 驱动异步完成)
 */
esp_err_t can_bus_tx(const twai_message_t *msg, TickType_t timeout);

/**
 * @brief 接收一帧
 */
esp_err_t can_bus_rx(twai_message_t *msg, TickType_t timeout);

/**
 * @brief 打印总线状态 (for debug)
 */
void can_bus_log_status(void);

/**
 * @brief 停止并卸载驱动（一般不用，主要给单元测试做 teardown）
 */
esp_err_t can_bus_deinit(void);

#ifdef __cplusplus
}
#endif

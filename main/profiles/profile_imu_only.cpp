// profile_imu_only.cpp
// 只跑小腿 IMU 通道（当前为 IMU2/UART2），按 KNEEEXO_IMU_CSV_HZ 频率把数据以 CSV 流到主串口 (UART0/USB-CDC)，
// 配合 PC 端 tools/imu_plot.py 做实时绘图。
//
// CSV 行格式（始终以 $IMU, 开头，方便 PC 端用正则筛选）：
//   $IMU,t_us,ax,ay,az,gx,gy,gz,roll,pitch,yaw,T
//
// 备注：默认日志级别会保留 INFO/WARN，但所有 ESP_LOGx 行都不以 $IMU, 开头，
// PC 端只解析 $IMU, 开头的行即可。

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_IMU_ONLY

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "witmotion_imu.h"
#include "app_main.hpp"

using namespace ExoConfig;

static const char *TAG = "profile.imu";

// =============================================================================
// 原始字节诊断：在正式 witmotion 驱动初始化之前，用最裸的 UART 读取
// 尝试几种常见波特率，看有没有字节进来，第一个字节是什么。
// 这段代码帮助判断：
//   - 0 字节 → IMU 没有供电，或 RX 引脚没接到 IMU TX
//   - 有字节但首字节不是 0x55 → 波特率不对
//   - 有字节且首字节 0x55 → 协议帧到了，witmotion 驱动应该能解析
// =============================================================================
static void raw_uart_probe(void)
{
    // 候选波特率（WitMotion 常见：9600 / 115200 / 460800）
    const int bauds[] = {115200, 9600, 460800};
    const int baud_count = 3;

    ESP_LOGW(TAG, "=== RAW UART PROBE START (GPIO TX=%d RX=%d) ===",
             (int)PIN_SHANK_IMU_MCU_TX, (int)PIN_SHANK_IMU_MCU_RX);

    for (int bi = 0; bi < baud_count; bi++) {
        int baud = bauds[bi];

        // 安装裸 UART（测完后 uninstall）
        uart_config_t cfg = {};
        cfg.baud_rate  = baud;
        cfg.data_bits  = UART_DATA_8_BITS;
        cfg.parity     = UART_PARITY_DISABLE;
        cfg.stop_bits  = UART_STOP_BITS_1;
        cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
        cfg.source_clk = UART_SCLK_DEFAULT;

        uart_driver_install(SHANK_IMU_UART_NUM, 512, 0, 0, NULL, 0);
        uart_param_config(SHANK_IMU_UART_NUM, &cfg);
        uart_set_pin(SHANK_IMU_UART_NUM, PIN_SHANK_IMU_MCU_TX, PIN_SHANK_IMU_MCU_RX,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_flush(SHANK_IMU_UART_NUM);

        // 等待 1.5 秒收字节
        const int WAIT_MS = 1500;
        int total = 0;
        uint8_t first_byte = 0xFF;
        bool has_0x55 = false;
        uint8_t buf[64];
        int64_t t_end = esp_timer_get_time() + WAIT_MS * 1000LL;

        while (esp_timer_get_time() < t_end) {
            int n = uart_read_bytes(SHANK_IMU_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(50));
            if (n > 0) {
                if (total == 0) first_byte = buf[0];
                total += n;
                for (int i = 0; i < n; i++) {
                    if (buf[i] == 0x55) { has_0x55 = true; }
                }
            }
        }

        if (total == 0) {
            ESP_LOGW(TAG, "  baud=%-7d → 0 bytes received  (RX pin not seeing IMU TX?)", baud);
        } else if (has_0x55) {
            ESP_LOGI(TAG, "  baud=%-7d → %d bytes, has 0x55!  ← CORRECT BAUD RATE", baud, total);
        } else {
            ESP_LOGW(TAG, "  baud=%-7d → %d bytes, first=0x%02X, no 0x55  (wrong baud?)", baud, total, first_byte);
        }

        uart_driver_delete(SHANK_IMU_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGW(TAG, "=== RAW UART PROBE DONE ===");
}

extern "C" void profile_imu_only_main(void)
{
    // 先做原始诊断，帮助定位问题
    raw_uart_probe();

    // ------ 正式初始化 ------
    ESP_ERROR_CHECK(witmotion_init_channel(IMU1_UART_NUM,
                                           PIN_IMU1_MCU_TX,
                                           PIN_IMU1_MCU_RX,
                                           IMU_UART_BAUD));
    ESP_ERROR_CHECK(witmotion_init_channel(IMU2_UART_NUM,
                                           PIN_IMU2_MCU_TX,
                                           PIN_IMU2_MCU_RX,
                                           IMU_UART_BAUD));

    ESP_LOGI(TAG, "waiting for first IMU frames: IMU1 UART%d TX=%d RX=%d, IMU2 UART%d TX=%d RX=%d, %d bps...",
             (int)IMU1_UART_NUM, (int)PIN_IMU1_MCU_TX, (int)PIN_IMU1_MCU_RX,
             (int)IMU2_UART_NUM, (int)PIN_IMU2_MCU_TX, (int)PIN_IMU2_MCU_RX,
             IMU_UART_BAUD);

    // 最多等 10 秒，超时报错继续（避免永久卡死）
    for (int i = 0; i < 100
         && !witmotion_is_alive_channel(IMU1_UART_NUM)
         && !witmotion_is_alive_channel(IMU2_UART_NUM); i++) {
        if (i % 10 == 9) ESP_LOGW(TAG, "  still waiting... (%ds)", (i+1)/10);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!witmotion_is_alive_channel(IMU1_UART_NUM)
        && !witmotion_is_alive_channel(IMU2_UART_NUM)) {
        ESP_LOGE(TAG, "NO IMU ALIVE after 10s! Check wiring. Probe results above tell you why.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    const int stream_hz = CONFIG_KNEEEXO_IMU_CSV_HZ > 25 ? 25 : CONFIG_KNEEEXO_IMU_CSV_HZ;
    ESP_LOGI(TAG, "IMU alive: IMU1=%d IMU2=%d. Streaming dual-IMU CSV at %d Hz.",
             witmotion_is_alive_channel(IMU1_UART_NUM) ? 1 : 0,
             witmotion_is_alive_channel(IMU2_UART_NUM) ? 1 : 0,
             stream_hz);

    // 一次性输出表头（带 # 号，PC 端可忽略）
    printf("# header: $IMU1/$IMU2,t_us,ax,ay,az,gx,gy,gz,roll,pitch,yaw,T\n");
    fflush(stdout);

    const TickType_t period = pdMS_TO_TICKS(1000 / stream_hz);
    TickType_t last = xTaskGetTickCount();
    witmotion_data_t d1;
    witmotion_data_t d2;

    while (true) {
        if (witmotion_get_latest_channel(IMU1_UART_NUM, &d1) == ESP_OK) {
            printf("$IMU1,%lld,"
                   "%.4f,%.4f,%.4f,"
                   "%.2f,%.2f,%.2f,"
                   "%.2f,%.2f,%.2f,"
                   "%.1f\n",
                   (long long)d1.ts_us,
                   d1.accel_g[0], d1.accel_g[1], d1.accel_g[2],
                   d1.gyro_dps[0], d1.gyro_dps[1], d1.gyro_dps[2],
                   d1.euler_deg[0], d1.euler_deg[1], d1.euler_deg[2],
                   d1.temp_c);
        }
        if (witmotion_get_latest_channel(IMU2_UART_NUM, &d2) == ESP_OK) {
            printf("$IMU2,%lld,"
                   "%.4f,%.4f,%.4f,"
                   "%.2f,%.2f,%.2f,"
                   "%.2f,%.2f,%.2f,"
                   "%.1f\n",
                   (long long)d2.ts_us,
                   d2.accel_g[0], d2.accel_g[1], d2.accel_g[2],
                   d2.gyro_dps[0], d2.gyro_dps[1], d2.gyro_dps[2],
                   d2.euler_deg[0], d2.euler_deg[1], d2.euler_deg[2],
                   d2.temp_c);
        }
        vTaskDelayUntil(&last, period);
    }
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_IMU_ONLY

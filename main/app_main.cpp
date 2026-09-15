// app_main.cpp - KneeExo profile dispatcher
//
// 通过 menuconfig (Application -> App profile) 选择当次构建烧入哪个 profile：
//   - normal         : IMU + RS02 + 100Hz 实验控制循环
//   - twai_loopback  : TWAI 自环自检（不需要电机）
//   - imu_only       : 仅 IMU 数据，CSV 输出到 USB-CDC，配套 tools/imu_plot.py
//   - motor_test     : 电机使能 + 反馈，用于方向 / 零位标定
//
// 这里只负责日志 + 分发；真正的工作在 main/profiles/profile_*.cpp 里。

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "esp_app_desc.h"
#include <stdio.h>
#include "sdkconfig.h"

#include "app_main.hpp"

static const char *TAG = "app_main";

extern "C" void print_firmware_info(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const char *profile = "UNKNOWN";
    const char *wear_mode = "NA";
#if CONFIG_KNEEEXO_APP_PROFILE_NORMAL
    profile = "NORMAL";
#if CONFIG_KNEEEXO_WEAR_MODE_ASSIST
    wear_mode = "ASSIST";
#elif CONFIG_KNEEEXO_WEAR_MODE_DAMPING
    wear_mode = "DAMPING";
#else
    wear_mode = "TRANSPARENT";
#endif
#elif CONFIG_KNEEEXO_APP_PROFILE_TWAI_LOOPBACK
    profile = "TWAI_LOOPBACK";
#elif CONFIG_KNEEEXO_APP_PROFILE_IMU_ONLY
    profile = "IMU_ONLY";
#elif CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST
    profile = "MOTOR_TEST";
#endif
    // Keep this as one line so terminals and dashboards can show it verbatim.
    printf("$INFO,app_version=%s,profile=%s,wear_mode=%s,build_date=%s,build_time=%s,idf=%s\n",
           app->version, profile, wear_mode, app->date, app->time, app->idf_ver);
    fflush(stdout);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "KneeExo boot, IDF v%d.%d.%d",
             ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
    print_firmware_info();

#if   defined(CONFIG_KNEEEXO_APP_PROFILE_NORMAL)
    ESP_LOGI(TAG, "Profile: NORMAL  (IMU + RS02 + 100Hz control loop)");
    profile_normal_main();

#elif defined(CONFIG_KNEEEXO_APP_PROFILE_TWAI_LOOPBACK)
    ESP_LOGI(TAG, "Profile: TWAI_LOOPBACK  (CAN controller self-test)");
    profile_twai_loopback_main();

#elif defined(CONFIG_KNEEEXO_APP_PROFILE_IMU_ONLY)
    ESP_LOGI(TAG, "Profile: IMU_ONLY  (CSV stream over USB-CDC)");
    profile_imu_only_main();

#elif defined(CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST)
    ESP_LOGI(TAG, "Profile: MOTOR_TEST  (enable + feedback + direction calibration)");
    profile_motor_test_main();

#else
    ESP_LOGE(TAG, "No KNEEEXO_APP_PROFILE selected! Run idf.py menuconfig.");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
#endif
}

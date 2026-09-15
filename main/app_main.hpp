#pragma once

#include "rs02_motor.h"
#include "witmotion_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// 全局电机句柄（仅在 normal profile 下使用，由 profile_normal.cpp 定义）
// 在其他 profile 下不引用、不会触发链接。
// =============================================================================
extern rs02_handle_t g_motor;
extern float g_user_zero_offset_rad;

// =============================================================================
// 各 profile 的入口（由 main/profiles/profile_*.cpp 实现）
// app_main.cpp 根据 sdkconfig 选择调用其中之一。
// =============================================================================
void profile_normal_main(void);
void profile_twai_loopback_main(void);
void profile_imu_only_main(void);
void profile_motor_test_main(void);

// 100Hz 控制循环（profile_normal 专用）
void control_task(void *arg);
void command_task(void *arg);

// Read-only identity of the running image; also printed before profile startup.
void print_firmware_info(void);

#ifdef __cplusplus
}
#endif

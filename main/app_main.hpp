#pragma once

#include "rs02_motor.h"
#include "witmotion_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

extern rs02_handle_t g_motor;

void control_task(void *arg);

#ifdef __cplusplus
}
#endif

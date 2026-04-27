// control_task.cpp
// 100Hz 控制循环 demo（profile = normal 时启动）：
//
// 仅在 normal profile 下编译；其他 profile 不引用 g_motor。
//   - 读取 IMU 最新数据
//   - 读取电机反馈
//   - 穿戴调试：透明模式 / 小阻尼模式 / 小助力跟随 + 落地缓冲
//   - 每 0.5s 打印一行日志（关节坐标 + 大腿俯仰）
//
// 后续替换思路：
//   1. 从 IMU 估算大腿姿态 / 步态相位
//   2. 由相位机决定 (target_joint_pos, torque_ff)
//   3. 经 joint_to_motor_rad/_torque 转回电机坐标，调 rs02_motion_control

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_NORMAL

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

#include "config.h"
#include "rs02_motor.h"
#include "witmotion_imu.h"
#include "app_main.hpp"

using namespace ExoConfig;

static const char *TAG = "control";

static constexpr int   LOOP_HZ    = CONFIG_KNEEEXO_CONTROL_HZ;
static constexpr float SOFT_LIMIT_MARGIN_RAD = 0.05f;

enum class GaitPhase : uint8_t {
    Unknown = 0,
    Stance,
    Swing,
    Landing,
};

static const char *gait_phase_name(GaitPhase phase)
{
    switch (phase) {
    case GaitPhase::Stance: return "stance";
    case GaitPhase::Swing:  return "swing";
    case GaitPhase::Landing:return "landing";
    default:                return "unknown";
    }
}

struct GaitEstimate {
    GaitPhase phase = GaitPhase::Unknown;
    bool landing_event = false;
    float acc_norm_g = 0.0f;
    float impact_g = 0.0f;
    float thigh_pitch_deg = 0.0f;
    float thigh_pitch_rate_dps = 0.0f;
};

static inline float clamp_float(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static GaitEstimate estimate_gait_from_imu(const witmotion_data_t &imu)
{
    GaitEstimate est{};
#if CONFIG_KNEEEXO_GAIT_IMU_ENABLE
    static float acc_norm_lp = 1.0f;
    static int landing_cooldown_loops = 0;
    static int landing_hold_loops = 0;
    static GaitPhase last_non_landing = GaitPhase::Unknown;

    est.thigh_pitch_deg =
        IMU_THIGH_PITCH_SIGN * imu.euler_deg[IMU_THIGH_PITCH_AXIS];
    est.thigh_pitch_rate_dps =
        IMU_THIGH_PITCH_SIGN * imu.gyro_dps[IMU_THIGH_PITCH_AXIS];
    est.acc_norm_g = sqrtf(imu.accel_g[0] * imu.accel_g[0]
                         + imu.accel_g[1] * imu.accel_g[1]
                         + imu.accel_g[2] * imu.accel_g[2]);

    // 低通代表“当前重力/慢变化基线”，高通尖刺代表冲击/落地候选。
    acc_norm_lp = 0.95f * acc_norm_lp + 0.05f * est.acc_norm_g;
    est.impact_g = est.acc_norm_g - acc_norm_lp;

    if (landing_cooldown_loops > 0) {
        landing_cooldown_loops--;
    }
    if (landing_hold_loops > 0) {
        landing_hold_loops--;
        est.phase = GaitPhase::Landing;
        return est;
    }

    const float impact_thresh = (float)CONFIG_KNEEEXO_GAIT_IMPACT_SPIKE_X100 / 100.0f;
    const int cooldown_loops = (CONFIG_KNEEEXO_GAIT_LANDING_COOLDOWN_MS * LOOP_HZ) / 1000;
    if (est.impact_g > impact_thresh && landing_cooldown_loops == 0) {
        est.phase = GaitPhase::Landing;
        est.landing_event = true;
        landing_cooldown_loops = cooldown_loops > 1 ? cooldown_loops : 1;
        landing_hold_loops = LOOP_HZ / 4; // hold "landing" on logs for ~250ms
        last_non_landing = GaitPhase::Stance;
        ESP_LOGW(TAG, "GAIT landing spike: acc=%.2fg impact=%+.2fg pitch=%+.1f rate=%+.1f",
                 est.acc_norm_g, est.impact_g,
                 est.thigh_pitch_deg, est.thigh_pitch_rate_dps);
        return est;
    }

    // Experimental placeholder:
    // 大腿快速前摆通常对应 swing；角速度小且不在落地尖刺冷却期时先视为 stance。
    // 后续需要结合膝关节角速度、足底压力或多周期状态机做去抖。
    if (est.thigh_pitch_rate_dps > (float)CONFIG_KNEEEXO_GAIT_SWING_RATE_DPS
        && est.thigh_pitch_deg > -30.0f) {
        est.phase = GaitPhase::Swing;
        last_non_landing = est.phase;
        return est;
    }
    if (fabsf(est.thigh_pitch_rate_dps) < 20.0f) {
        est.phase = GaitPhase::Stance;
        last_non_landing = est.phase;
        return est;
    }
    est.phase = last_non_landing;
    return est;
#else
    (void)imu;
    return est;
#endif
}

extern "C" void control_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t loop_count = 0;
    const uint32_t log_every = LOOP_HZ / 2;  // 每 0.5s 打印一次
    witmotion_data_t  imu{};
    rs02_feedback_t   fb{};

#if CONFIG_KNEEEXO_WEAR_MODE_DAMPING
    const float damping_b = (float)CONFIG_KNEEEXO_DAMPING_B_X100 / 100.0f;
    const float damping_torque_max = (float)CONFIG_KNEEEXO_DAMPING_MAX_TORQUE_X100 / 100.0f;
    ESP_LOGI(TAG, "control loop @ %dHz, mode=DAMPING, B=%.2f Nm/(rad/s), clamp=%.2f Nm, user_zero=%+.3f rad",
             LOOP_HZ, damping_b, damping_torque_max, g_user_zero_offset_rad);
#elif CONFIG_KNEEEXO_WEAR_MODE_ASSIST
    const float assist_b = (float)CONFIG_KNEEEXO_ASSIST_FOLLOW_B_X100 / 100.0f;
    const float assist_torque_max = (float)CONFIG_KNEEEXO_ASSIST_MAX_TORQUE_X100 / 100.0f;
    const float assist_vel_deadband = (float)CONFIG_KNEEEXO_ASSIST_VEL_DEADBAND_MRAD_S / 1000.0f;
    const float buffer_b = (float)CONFIG_KNEEEXO_LANDING_BUFFER_B_X100 / 100.0f;
    const float buffer_torque_max = (float)CONFIG_KNEEEXO_LANDING_BUFFER_MAX_TORQUE_X100 / 100.0f;
    const float buffer_near_ext = (float)CONFIG_KNEEEXO_LANDING_BUFFER_NEAR_EXT_MRAD / 1000.0f;
    const float buffer_min_flex_vel = (float)CONFIG_KNEEEXO_LANDING_BUFFER_MIN_FLEX_VEL_MRAD_S / 1000.0f;
    ESP_LOGI(TAG,
             "control loop @ %dHz, mode=ASSIST, follow_B=%.2f clamp=%.2f, "
             "buffer_B=%.2f buffer_clamp=%.2f, user_zero=%+.3f rad",
             LOOP_HZ, assist_b, assist_torque_max,
             buffer_b, buffer_torque_max, g_user_zero_offset_rad);
#else
    ESP_LOGI(TAG, "control loop @ %dHz, mode=TRANSPARENT, user_zero=%+.3f rad",
             LOOP_HZ, g_user_zero_offset_rad);
#endif

    while (true) {
        witmotion_get_latest(&imu);
        rs02_get_feedback(&g_motor, &fb, 0);

        const float joint_mech_pos = motor_to_joint_rad(fb.position_rad);
        const float joint_user_pos = joint_mech_pos - g_user_zero_offset_rad;
        const float joint_vel = motor_to_joint_vel(fb.velocity_rad_s);
        const GaitEstimate gait = estimate_gait_from_imu(imu);

        float joint_torque_ff = 0.0f;
#if CONFIG_KNEEEXO_WEAR_MODE_DAMPING
        joint_torque_ff = -damping_b * joint_vel;
        joint_torque_ff = clamp_float(joint_torque_ff, -damping_torque_max, damping_torque_max);
#elif CONFIG_KNEEEXO_WEAR_MODE_ASSIST
        if (fabsf(joint_vel) > assist_vel_deadband) {
            joint_torque_ff = assist_b * joint_vel;
            joint_torque_ff = clamp_float(joint_torque_ff, -assist_torque_max, assist_torque_max);
        }

        // 落地缓冲的无 IMU 近似：
        // 关节靠近伸膝端，且正在较快屈膝时，给伸膝方向的反向阻尼，减小“砸弯”感。
        const bool near_extension = joint_mech_pos > (KNEE_EXT_LIMIT_RAD + SOFT_LIMIT_MARGIN_RAD)
                                 && joint_mech_pos < (KNEE_EXT_LIMIT_RAD + buffer_near_ext);
        const bool fast_flexion = joint_vel > buffer_min_flex_vel;
        if (near_extension && fast_flexion) {
            const float buffer_torque = -clamp_float(buffer_b * joint_vel,
                                                     0.0f,
                                                     buffer_torque_max);
            joint_torque_ff += buffer_torque;
        }
#endif

        // 软限位保护使用机械 joint 坐标，不受 user zero 影响。
        if (joint_mech_pos <= KNEE_EXT_LIMIT_RAD + SOFT_LIMIT_MARGIN_RAD && joint_torque_ff < 0.0f) {
            joint_torque_ff = 0.0f;
        }
        if (joint_mech_pos >= KNEE_FLEX_LIMIT_RAD - SOFT_LIMIT_MARGIN_RAD && joint_torque_ff > 0.0f) {
            joint_torque_ff = 0.0f;
        }

        rs02_motion_control(&g_motor,
                            joint_to_motor_torque(joint_torque_ff),
                            fb.position_rad,
                            0.0f,
                            0.0f,
                            0.0f);

        if ((loop_count++ % log_every) == 0) {
            const float thigh_pitch =
                IMU_THIGH_PITCH_SIGN * imu.euler_deg[IMU_THIGH_PITCH_AXIS];
            ESP_LOGI(TAG,
                     "thigh_pitch=%+6.2f deg | joint_user=%+6.3f rad (%+6.1f deg) "
                     "joint_mech=%+6.3f vel=%+6.3f cmd_tq=%+5.2f fb_tq=%+5.2f "
                     "gait=%s%s acc=%.2fg impact=%+.2fg T=%4.1f%s",
                     thigh_pitch,
                     joint_user_pos, joint_user_pos * 180.0f / (float)M_PI,
                     joint_mech_pos,
                     joint_vel,
                     joint_torque_ff,
                     motor_to_joint_torque(fb.torque_nm),
                     gait_phase_name(gait.phase),
                     gait.landing_event ? "!" : "",
                     gait.acc_norm_g,
                     gait.impact_g,
                     fb.temperature_c,
                     fb.has_fault ? " FAULT!" : "");
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_NORMAL

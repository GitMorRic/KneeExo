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
#include <stdio.h>

#include "config.h"
#include "rs02_motor.h"
#include "witmotion_imu.h"
#include "app_main.hpp"

using namespace ExoConfig;

#ifndef CONFIG_KNEEEXO_GAIT_IMPACT_SPIKE_X100
#define CONFIG_KNEEEXO_GAIT_IMPACT_SPIKE_X100 18
#endif
#ifndef CONFIG_KNEEEXO_GAIT_LANDING_COOLDOWN_MS
#define CONFIG_KNEEEXO_GAIT_LANDING_COOLDOWN_MS 350
#endif
#ifndef CONFIG_KNEEEXO_GAIT_SWING_RATE_DPS
#define CONFIG_KNEEEXO_GAIT_SWING_RATE_DPS 35
#endif

static const char *TAG = "control";

static constexpr int   LOOP_HZ    = CONFIG_KNEEEXO_CONTROL_HZ;
static constexpr float SOFT_LIMIT_MARGIN_RAD = 0.05f;
static constexpr float DEG_TO_RAD = (float)M_PI / 180.0f;
static constexpr float RAD_TO_DEG = 180.0f / (float)M_PI;

enum class ExoState : uint8_t {
    SafePassive = 0,
    InitialDoubleSupport,
    SingleSupport,
    InitialSwing,
    MidSwing,
    TerminalSwing,
};

static const char *exo_state_name(ExoState state)
{
    switch (state) {
    case ExoState::SafePassive:          return "SAFE";
    case ExoState::InitialDoubleSupport: return "IDS";
    case ExoState::SingleSupport:        return "SINGLE";
    case ExoState::InitialSwing:         return "ISWING";
    case ExoState::MidSwing:             return "MSWING";
    case ExoState::TerminalSwing:        return "TSWING";
    default:                             return "SAFE";
    }
}

struct MotionFeatures {
    bool imu_alive = false;
    bool motor_alive = false;
    bool sensor_fault = false;
    bool landing_event = false;
    bool knee_flexion_peak = false;
    bool shank_vertical_cross = false;
    float acc_norm_g = 0.0f;
    float acc_norm_lp_g = 1.0f;
    float impact_g = 0.0f;
    float shank_pitch_deg = 0.0f;
    float shank_pitch_rate_dps = 0.0f;
    float shank_pitch_rate_lp_dps = 0.0f;
    float knee_angle_rad = 0.0f;
    float knee_vel_rad_s = 0.0f;
    float knee_vel_lp_rad_s = 0.0f;
    float prev_knee_vel_lp_rad_s = 0.0f;
    float prev_shank_pitch_deg = 0.0f;
};

static inline float clamp_float(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float gravity_comp_torque(float shank_pitch_rad)
{
    if (!SHANK_GRAVITY_COMP_ENABLE) {
        return 0.0f;
    }
    const float link_torque = SHANK_GRAVITY_G_NM * sinf(shank_pitch_rad + SHANK_GRAVITY_PHI_RAD)
                            + SHANK_GRAVITY_BIAS_NM;
    return clamp_float(-link_torque, -SHANK_GRAVITY_MAX_NM, SHANK_GRAVITY_MAX_NM);
}

static void log_gravity_id_points(void)
{
    const float lo = KNEE_EXT_LIMIT_RAD + GRAVITY_ID_MARGIN_RAD;
    const float hi = KNEE_FLEX_LIMIT_RAD - GRAVITY_ID_MARGIN_RAD;
    if (hi <= lo) {
        ESP_LOGW(TAG, "gravity-id points unavailable: limits too narrow (lo=%.3f hi=%.3f)",
                 lo, hi);
        return;
    }

    ESP_LOGW(TAG,
             "gravity-id joint targets are generated from safe limits only: "
             "joint=[%.3f, %.3f] rad, margin=%.3f rad, points=%d",
             lo, hi, GRAVITY_ID_MARGIN_RAD, GRAVITY_ID_POINT_COUNT);
    for (int i = 0; i < GRAVITY_ID_POINT_COUNT; i++) {
        const float a = (GRAVITY_ID_POINT_COUNT <= 1)
                      ? 0.5f
                      : (float)i / (float)(GRAVITY_ID_POINT_COUNT - 1);
        const float joint = lo + (hi - lo) * a;
        ESP_LOGW(TAG, "  GID point %d: joint=%+.3f rad (%+.1f deg), motor=%+.3f rad",
                 i + 1, joint, joint * RAD_TO_DEG, joint_to_motor_rad(joint));
    }
}

static MotionFeatures update_features(const witmotion_data_t &imu,
                                      const rs02_feedback_t &fb,
                                      bool imu_ok,
                                      bool motor_ok,
                                      float joint_mech_pos,
                                      float joint_vel)
{
    static MotionFeatures f{};
    static int landing_cooldown_loops = 0;
    const float prev_knee_vel_lp = f.knee_vel_lp_rad_s;
    const float prev_shank_pitch = f.shank_pitch_deg;

    f.imu_alive = imu_ok && imu.ts_us > 0;
    f.motor_alive = motor_ok && fb.ts_us > 0;
    f.sensor_fault = !f.imu_alive || !f.motor_alive || fb.has_fault;
    f.knee_angle_rad = joint_mech_pos;
    f.knee_vel_rad_s = joint_vel;
    f.prev_knee_vel_lp_rad_s = prev_knee_vel_lp;
    f.knee_vel_lp_rad_s = 0.75f * f.knee_vel_lp_rad_s + 0.25f * joint_vel;
    f.prev_shank_pitch_deg = prev_shank_pitch;

    f.shank_pitch_deg =
        IMU_SHANK_PITCH_SIGN * imu.euler_deg[IMU_SHANK_PITCH_AXIS];
    f.shank_pitch_rate_dps =
        IMU_SHANK_PITCH_SIGN * imu.gyro_dps[IMU_SHANK_PITCH_AXIS];
    f.shank_pitch_rate_lp_dps =
        0.75f * f.shank_pitch_rate_lp_dps + 0.25f * f.shank_pitch_rate_dps;
    f.acc_norm_g = sqrtf(imu.accel_g[0] * imu.accel_g[0]
                       + imu.accel_g[1] * imu.accel_g[1]
                       + imu.accel_g[2] * imu.accel_g[2]);

    // 约 30~80ms 量级的平滑/高通特征，避免单点噪声触发状态跳变。
    f.acc_norm_lp_g = 0.95f * f.acc_norm_lp_g + 0.05f * f.acc_norm_g;
    f.impact_g = f.acc_norm_g - f.acc_norm_lp_g;

    if (landing_cooldown_loops > 0) {
        landing_cooldown_loops--;
    }

    const float impact_thresh = (float)CONFIG_KNEEEXO_GAIT_IMPACT_SPIKE_X100 / 100.0f;
    const bool knee_flexing_fast = f.knee_vel_lp_rad_s > 0.45f;
    const bool knee_angle_reasonable = f.knee_angle_rad > -0.05f
                                    && f.knee_angle_rad < 0.75f;
    const bool shank_reasonable = f.shank_pitch_deg > -45.0f
                               && f.shank_pitch_deg < 45.0f;
    f.landing_event = f.impact_g > impact_thresh
                   && knee_flexing_fast
                   && knee_angle_reasonable
                   && shank_reasonable
                   && landing_cooldown_loops == 0;
    if (f.landing_event) {
        const int cooldown_loops = (CONFIG_KNEEEXO_GAIT_LANDING_COOLDOWN_MS * LOOP_HZ) / 1000;
        landing_cooldown_loops = cooldown_loops > 1 ? cooldown_loops : 1;
    }

    f.knee_flexion_peak = prev_knee_vel_lp > 0.10f
                       && f.knee_vel_lp_rad_s <= -0.05f
                       && f.knee_angle_rad > 0.45f;
    f.shank_vertical_cross = prev_shank_pitch < 0.0f
                          && f.shank_pitch_deg >= 0.0f;
    return f;
}

struct StateMachine {
    ExoState state = ExoState::SafePassive;
    int loops_in_state = 0;
    int good_cycles = 0;
};

static void transition_to(StateMachine &sm, ExoState next, const char *why)
{
    if (sm.state == next) {
        return;
    }
    ESP_LOGW(TAG, "STATE %s -> %s (%s)",
             exo_state_name(sm.state), exo_state_name(next), why);
    sm.state = next;
    sm.loops_in_state = 0;
}

static void update_state_machine(StateMachine &sm, const MotionFeatures &f)
{
    sm.loops_in_state++;

    const bool unsafe_angle = f.knee_angle_rad < -5.0f * DEG_TO_RAD
                           || f.knee_angle_rad > 100.0f * DEG_TO_RAD;
    const bool unsafe_speed = fabsf(f.knee_vel_rad_s) > 8.0f;
    if (f.sensor_fault || unsafe_angle || unsafe_speed) {
        transition_to(sm, ExoState::SafePassive, "global safety");
        return;
    }

    const int timeout_safe = LOOP_HZ * 3;
    const int timeout_stance = LOOP_HZ * 2;
    const int timeout_swing = LOOP_HZ * 3 / 10; // 300ms
    const bool swing_like = f.knee_vel_lp_rad_s > 0.35f
                         && f.shank_pitch_rate_lp_dps > (float)CONFIG_KNEEEXO_GAIT_SWING_RATE_DPS;

    switch (sm.state) {
    case ExoState::SafePassive:
        if (f.landing_event) {
            sm.good_cycles = sm.good_cycles < 2 ? sm.good_cycles + 1 : sm.good_cycles;
            transition_to(sm, ExoState::InitialDoubleSupport, "trusted landing");
        } else if (sm.loops_in_state > timeout_safe && swing_like) {
            transition_to(sm, ExoState::InitialSwing, "safe observed swing");
        }
        break;

    case ExoState::InitialDoubleSupport:
        if (swing_like) {
            transition_to(sm, ExoState::InitialSwing, "early swing while loading");
        } else if (f.knee_angle_rad > 0.12f && fabsf(f.knee_vel_lp_rad_s) < 0.25f) {
            transition_to(sm, ExoState::SingleSupport, "knee loaded/stable");
        } else if (sm.loops_in_state > LOOP_HZ / 2) {
            transition_to(sm, ExoState::SingleSupport, "IDS timeout");
        }
        break;

    case ExoState::SingleSupport:
        if (swing_like) {
            transition_to(sm, ExoState::InitialSwing, "toe-off/swing features");
        } else if (f.landing_event) {
            transition_to(sm, ExoState::InitialDoubleSupport, "new landing");
        } else if (sm.loops_in_state > timeout_stance) {
            transition_to(sm, ExoState::SafePassive, "single support timeout");
        }
        break;

    case ExoState::InitialSwing:
        if (f.landing_event) {
            transition_to(sm, ExoState::InitialDoubleSupport, "landing during initial swing");
        } else if (f.knee_flexion_peak) {
            transition_to(sm, ExoState::MidSwing, "knee flexion peak");
        } else if (sm.loops_in_state > timeout_swing) {
            transition_to(sm, ExoState::MidSwing, "initial swing timeout");
        }
        break;

    case ExoState::MidSwing:
        if (f.landing_event) {
            transition_to(sm, ExoState::InitialDoubleSupport, "landing during mid swing");
        } else if (f.shank_vertical_cross || f.knee_vel_lp_rad_s < -0.25f) {
            transition_to(sm, ExoState::TerminalSwing, "shank vertical/extension");
        } else if (sm.loops_in_state > timeout_swing) {
            transition_to(sm, ExoState::TerminalSwing, "mid swing timeout");
        }
        break;

    case ExoState::TerminalSwing:
        if (f.landing_event) {
            transition_to(sm, ExoState::InitialDoubleSupport, "heel strike");
        } else if (f.knee_angle_rad < -0.02f) {
            transition_to(sm, ExoState::SafePassive, "hyper-extension guard");
        } else if (sm.loops_in_state > timeout_swing) {
            transition_to(sm, ExoState::SafePassive, "terminal swing timeout");
        }
        break;
    }
}

struct ImpedanceParams {
    float k_nm_per_rad;
    float b_nm_s_per_rad;
    float theta_eq_rad;
    float torque_ff_nm;
    float torque_limit_nm;
};

static ImpedanceParams params_for_state(ExoState state, int loops_in_state)
{
    (void)loops_in_state;
    switch (state) {
    case ExoState::InitialDoubleSupport:
        return {8.0f, 1.4f, 8.0f * DEG_TO_RAD, 0.0f, 1.2f};
    case ExoState::SingleSupport:
        return {14.0f, 1.6f, 10.0f * DEG_TO_RAD, 0.0f, 1.5f};
    case ExoState::InitialSwing:
        return {1.5f, 0.25f, 45.0f * DEG_TO_RAD, 0.15f, 0.8f};
    case ExoState::MidSwing:
        return {0.0f, 0.08f, 0.0f, 0.0f, 0.4f};
    case ExoState::TerminalSwing: {
        const float a = clamp_float((float)loops_in_state / (float)(LOOP_HZ * 3 / 10), 0.0f, 1.0f);
        return {17.0f, 0.6f + 3.0f * a, 3.0f * DEG_TO_RAD, 0.0f, 1.0f};
    }
    case ExoState::SafePassive:
    default:
        return {0.0f, 0.08f, 0.0f, 0.0f, 0.4f};
    }
}

extern "C" void control_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t loop_count = 0;
    const uint32_t log_every = LOOP_HZ / 2;  // 每 0.5s 打印一次
    const uint32_t telemetry_every = LOOP_HZ / 10; // 10Hz CSV 给上位机
    witmotion_data_t  imu{};
    rs02_feedback_t   fb{};
    StateMachine sm{};

    log_gravity_id_points();

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
        const bool imu_ok = (witmotion_get_latest(&imu) == ESP_OK);
        const bool motor_ok = (rs02_get_feedback(&g_motor, &fb, 0) == ESP_OK);

        const float joint_mech_pos = motor_to_joint_rad(fb.position_rad);
        const float joint_user_pos = joint_mech_pos - g_user_zero_offset_rad;
        const float joint_vel = motor_to_joint_vel(fb.velocity_rad_s);
        const MotionFeatures feat = update_features(imu, fb, imu_ok, motor_ok,
                                                    joint_mech_pos, joint_vel);
        update_state_machine(sm, feat);

        const float shank_pitch_rad = feat.shank_pitch_deg * DEG_TO_RAD;
        const float joint_gravity_ff = gravity_comp_torque(shank_pitch_rad);
        float joint_torque_ff = joint_gravity_ff;
        float joint_impedance_torque = 0.0f;

#if CONFIG_KNEEEXO_GAIT_IMU_ENABLE
        const ImpedanceParams ip = params_for_state(sm.state, sm.loops_in_state);
#else
        const ImpedanceParams ip = params_for_state(ExoState::SafePassive, sm.loops_in_state);
#endif
        joint_impedance_torque =
            ip.k_nm_per_rad * (ip.theta_eq_rad - joint_mech_pos)
          - ip.b_nm_s_per_rad * joint_vel
          + ip.torque_ff_nm;
        joint_impedance_torque = clamp_float(joint_impedance_torque,
                                             -ip.torque_limit_nm,
                                             ip.torque_limit_nm);
        joint_torque_ff += joint_impedance_torque;

#if CONFIG_KNEEEXO_WEAR_MODE_DAMPING
        const float wear_torque = clamp_float(-damping_b * joint_vel,
                                              -damping_torque_max,
                                              damping_torque_max);
        joint_torque_ff += wear_torque;
#elif CONFIG_KNEEEXO_WEAR_MODE_ASSIST
        if (fabsf(joint_vel) > assist_vel_deadband) {
            const float wear_torque = clamp_float(assist_b * joint_vel,
                                                  -assist_torque_max,
                                                  assist_torque_max);
            joint_torque_ff += wear_torque;
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
            ESP_LOGI(TAG,
                     "state=%s shank=%+6.2f deg | joint_user=%+6.3f rad (%+6.1f deg) "
                     "joint_mech=%+6.3f vel=%+6.3f tq_cmd=%+5.2f grav=%+5.2f imp=%+5.2f "
                     "fb_tq=%+5.2f acc=%.2fg impact=%+.2fg T=%4.1f%s",
                     exo_state_name(sm.state),
                     feat.shank_pitch_deg,
                     joint_user_pos, joint_user_pos * RAD_TO_DEG,
                     joint_mech_pos,
                     joint_vel,
                     joint_torque_ff,
                     joint_gravity_ff,
                     joint_impedance_torque,
                     motor_to_joint_torque(fb.torque_nm),
                     feat.acc_norm_g,
                     feat.impact_g,
                     fb.temperature_c,
                     fb.has_fault ? " FAULT!" : "");
        }

        if ((loop_count % telemetry_every) == 0) {
            printf("$EXO,%lld,%s,"
                   "%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%d,%d,%d\n",
                   (long long)esp_timer_get_time(),
                   exo_state_name(sm.state),
                   feat.shank_pitch_deg,
                   feat.shank_pitch_rate_dps,
                   feat.acc_norm_g,
                   joint_mech_pos,
                   joint_vel,
                   joint_gravity_ff,
                   joint_impedance_torque,
                   joint_torque_ff,
                   motor_to_joint_torque(fb.torque_nm),
                   ip.k_nm_per_rad,
                   ip.b_nm_s_per_rad,
                   ip.theta_eq_rad,
                   feat.landing_event ? 1 : 0,
                   feat.knee_flexion_peak ? 1 : 0,
                   feat.shank_vertical_cross ? 1 : 0);
            fflush(stdout);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_NORMAL

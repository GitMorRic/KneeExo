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
#include "nvs.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static volatile bool  g_req_zero_imu = false;
static volatile bool  g_req_zero_motor = false;
static volatile bool  g_force_safe = false;
static float          g_shank_pitch_zero_deg = 0.0f;
static float          g_gravity_G_nm = SHANK_GRAVITY_G_NM;
static float          g_gravity_phi_rad = SHANK_GRAVITY_PHI_RAD;
static float          g_gravity_bias_nm = SHANK_GRAVITY_BIAS_NM;
static bool           g_gravity_enabled = SHANK_GRAVITY_COMP_ENABLE;
static bool           g_state_impedance_enabled = true;
static bool           g_state_lock_enabled = false;
static uint8_t        g_state_lock_id = 0;
static float          g_landing_impact_thresh_g =
    (float)CONFIG_KNEEEXO_GAIT_IMPACT_SPIKE_X100 / 100.0f;
static float          g_swing_rate_thresh_dps =
    (float)CONFIG_KNEEEXO_GAIT_SWING_RATE_DPS;
static bool           g_params_loaded = false;

static void load_runtime_params_once(void)
{
    if (g_params_loaded) {
        return;
    }
    g_params_loaded = true;
    nvs_handle_t h;
    if (nvs_open("exo", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t len = sizeof(float);
    nvs_get_blob(h, "imu_zero", &g_shank_pitch_zero_deg, &len);
    len = sizeof(float);
    nvs_get_blob(h, "grav_G", &g_gravity_G_nm, &len);
    len = sizeof(float);
    nvs_get_blob(h, "grav_phi", &g_gravity_phi_rad, &len);
    len = sizeof(float);
    nvs_get_blob(h, "grav_bias", &g_gravity_bias_nm, &len);
    len = sizeof(float);
    nvs_get_blob(h, "impact_th", &g_landing_impact_thresh_g, &len);
    len = sizeof(float);
    nvs_get_blob(h, "swing_th", &g_swing_rate_thresh_dps, &len);
    uint8_t en = g_gravity_enabled ? 1 : 0;
    nvs_get_u8(h, "grav_en", &en);
    g_gravity_enabled = en != 0;
    nvs_close(h);
    ESP_LOGI(TAG, "runtime params: imu_zero=%.3f deg, G=%.4f phi=%.4f bias=%.4f en=%d impact=%.3fg swing=%.1fdps",
             g_shank_pitch_zero_deg,
             g_gravity_G_nm,
             g_gravity_phi_rad,
             g_gravity_bias_nm,
             g_gravity_enabled ? 1 : 0,
             g_landing_impact_thresh_g,
             g_swing_rate_thresh_dps);
}

static esp_err_t save_runtime_params(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("exo", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, "imu_zero", &g_shank_pitch_zero_deg, sizeof(float));
    if (err == ESP_OK) err = nvs_set_blob(h, "grav_G", &g_gravity_G_nm, sizeof(float));
    if (err == ESP_OK) err = nvs_set_blob(h, "grav_phi", &g_gravity_phi_rad, sizeof(float));
    if (err == ESP_OK) err = nvs_set_blob(h, "grav_bias", &g_gravity_bias_nm, sizeof(float));
    if (err == ESP_OK) err = nvs_set_blob(h, "impact_th", &g_landing_impact_thresh_g, sizeof(float));
    if (err == ESP_OK) err = nvs_set_blob(h, "swing_th", &g_swing_rate_thresh_dps, sizeof(float));
    if (err == ESP_OK) err = nvs_set_u8(h, "grav_en", g_gravity_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

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

static bool parse_exo_state(const char *name, ExoState *out)
{
    if (strcmp(name, "SAFE") == 0) {
        *out = ExoState::SafePassive;
    } else if (strcmp(name, "IDS") == 0) {
        *out = ExoState::InitialDoubleSupport;
    } else if (strcmp(name, "SINGLE") == 0) {
        *out = ExoState::SingleSupport;
    } else if (strcmp(name, "ISWING") == 0) {
        *out = ExoState::InitialSwing;
    } else if (strcmp(name, "MSWING") == 0) {
        *out = ExoState::MidSwing;
    } else if (strcmp(name, "TSWING") == 0) {
        *out = ExoState::TerminalSwing;
    } else {
        return false;
    }
    return true;
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
    float prev_shank_pitch_rate_lp_dps = 0.0f;
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
    if (!g_gravity_enabled) {
        return 0.0f;
    }
    const float compensation_torque = g_gravity_G_nm * sinf(shank_pitch_rad + g_gravity_phi_rad)
                                    + g_gravity_bias_nm;
    return clamp_float(compensation_torque, -SHANK_GRAVITY_MAX_NM, SHANK_GRAVITY_MAX_NM);
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
    const float prev_shank_rate_lp = f.shank_pitch_rate_lp_dps;

    f.imu_alive = imu_ok && imu.ts_us > 0;
    f.motor_alive = motor_ok && fb.ts_us > 0;
    f.sensor_fault = !f.imu_alive || !f.motor_alive || fb.has_fault;
    f.knee_angle_rad = joint_mech_pos;
    f.knee_vel_rad_s = joint_vel;
    f.prev_knee_vel_lp_rad_s = prev_knee_vel_lp;
    f.knee_vel_lp_rad_s = 0.75f * f.knee_vel_lp_rad_s + 0.25f * joint_vel;
    f.prev_shank_pitch_deg = prev_shank_pitch;

    f.shank_pitch_deg =
        IMU_SHANK_PITCH_SIGN * imu.euler_deg[IMU_SHANK_PITCH_AXIS] - g_shank_pitch_zero_deg;
    f.shank_pitch_rate_dps =
        IMU_SHANK_PITCH_SIGN * imu.gyro_dps[IMU_SHANK_PITCH_AXIS];
    f.shank_pitch_rate_lp_dps =
        0.75f * f.shank_pitch_rate_lp_dps + 0.25f * f.shank_pitch_rate_dps;
    f.prev_shank_pitch_rate_lp_dps = prev_shank_rate_lp;
    f.acc_norm_g = sqrtf(imu.accel_g[0] * imu.accel_g[0]
                       + imu.accel_g[1] * imu.accel_g[1]
                       + imu.accel_g[2] * imu.accel_g[2]);

    // 约 30~80ms 量级的平滑/高通特征，避免单点噪声触发状态跳变。
    f.acc_norm_lp_g = 0.95f * f.acc_norm_lp_g + 0.05f * f.acc_norm_g;
    f.impact_g = f.acc_norm_g - f.acc_norm_lp_g;

    if (landing_cooldown_loops > 0) {
        landing_cooldown_loops--;
    }

    const bool knee_motion_plausible = fabsf(f.knee_vel_lp_rad_s) < 4.0f;
    const bool knee_angle_reasonable = f.knee_angle_rad > -0.05f
                                    && f.knee_angle_rad < 1.65f;
    const bool shank_reasonable = f.shank_pitch_deg > -85.0f
                               && f.shank_pitch_deg < 75.0f;
    f.landing_event = f.impact_g > g_landing_impact_thresh_g
                   && knee_motion_plausible
                   && knee_angle_reasonable
                   && shank_reasonable
                   && landing_cooldown_loops == 0;
    if (f.landing_event) {
        const int cooldown_loops = (CONFIG_KNEEEXO_GAIT_LANDING_COOLDOWN_MS * LOOP_HZ) / 1000;
        landing_cooldown_loops = cooldown_loops > 1 ? cooldown_loops : 1;
    }

    f.knee_flexion_peak = (prev_knee_vel_lp > 0.10f
                        && f.knee_vel_lp_rad_s <= -0.05f
                        && f.knee_angle_rad > 0.45f)
                       || (prev_shank_rate_lp < -8.0f
                        && f.shank_pitch_rate_lp_dps >= 3.0f
                        && f.shank_pitch_deg < -8.0f);
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
    if (g_force_safe || f.sensor_fault || unsafe_angle || unsafe_speed) {
        transition_to(sm, ExoState::SafePassive, "global safety");
        return;
    }

    if (g_state_lock_enabled) {
        transition_to(sm, static_cast<ExoState>(g_state_lock_id), "manual state lock");
        return;
    }

    const int timeout_stance = LOOP_HZ * 2;
    const int timeout_swing = LOOP_HZ * 3 / 10; // 300ms
    const bool backward_swing = f.shank_pitch_rate_lp_dps < -g_swing_rate_thresh_dps
                             && f.shank_pitch_deg < -6.0f;
    const bool knee_flexing = f.knee_vel_lp_rad_s > 0.18f
                           && f.knee_angle_rad < 1.25f;
    const bool swing_like = backward_swing || knee_flexing;
    const bool terminal_swing_like = f.shank_pitch_rate_lp_dps > 0.35f * g_swing_rate_thresh_dps
                                  && f.knee_vel_lp_rad_s < -0.08f
                                  && f.knee_angle_rad < 0.75f;

    switch (sm.state) {
    case ExoState::SafePassive:
        if (f.landing_event) {
            sm.good_cycles = sm.good_cycles < 2 ? sm.good_cycles + 1 : sm.good_cycles;
            transition_to(sm, ExoState::InitialDoubleSupport, "trusted landing");
        } else if (swing_like) {
            transition_to(sm, ExoState::InitialSwing, "safe observed swing");
        }
        break;

    case ExoState::InitialDoubleSupport:
        if (swing_like) {
            transition_to(sm, ExoState::InitialSwing, "early swing while loading");
        } else if (f.knee_angle_rad > 0.12f && fabsf(f.knee_vel_lp_rad_s) < 0.25f) {
            transition_to(sm, ExoState::SingleSupport, "knee loaded/stable");
        } else if (sm.loops_in_state > LOOP_HZ / 2) {
            transition_to(sm, ExoState::SafePassive, "IDS timeout guard");
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
            transition_to(sm, ExoState::MidSwing, "swing reversal/flexion peak");
        } else if (sm.loops_in_state > timeout_swing) {
            transition_to(sm, ExoState::SafePassive, "initial swing timeout guard");
        }
        break;

    case ExoState::MidSwing:
        if (f.landing_event) {
            transition_to(sm, ExoState::InitialDoubleSupport, "landing during mid swing");
        } else if (terminal_swing_like || f.shank_vertical_cross) {
            transition_to(sm, ExoState::TerminalSwing, "extension/forward swing");
        } else if (sm.loops_in_state > timeout_swing) {
            transition_to(sm, ExoState::SafePassive, "mid swing timeout guard");
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
        return {2.5f, 0.45f, 8.0f * DEG_TO_RAD, 0.0f, 0.45f};
    case ExoState::SingleSupport:
        return {3.5f, 0.35f, 10.0f * DEG_TO_RAD, 0.0f, 0.50f};
    case ExoState::InitialSwing:
        return {2.5f, 0.10f, 60.0f * DEG_TO_RAD, 0.10f, 0.65f};
    case ExoState::MidSwing:
        return {0.0f, 0.03f, 0.0f, 0.0f, 0.25f};
    case ExoState::TerminalSwing: {
        const float a = clamp_float((float)loops_in_state / (float)(LOOP_HZ * 3 / 10), 0.0f, 1.0f);
        return {2.0f, 0.35f + 0.65f * a, 3.0f * DEG_TO_RAD, 0.0f, 0.55f};
    }
    case ExoState::SafePassive:
    default:
        return {0.0f, 0.04f, 0.0f, 0.0f, 0.25f};
    }
}

static void print_imu_telemetry(const char *tag, const witmotion_data_t &d)
{
    printf("%s,%lld,"
           "%.4f,%.4f,%.4f,"
           "%.2f,%.2f,%.2f,"
           "%.2f,%.2f,%.2f,"
           "%.1f\n",
           tag,
           (long long)d.ts_us,
           d.accel_g[0], d.accel_g[1], d.accel_g[2],
           d.gyro_dps[0], d.gyro_dps[1], d.gyro_dps[2],
           d.euler_deg[0], d.euler_deg[1], d.euler_deg[2],
           d.temp_c);
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
    witmotion_data_t  thigh_imu{};
    witmotion_data_t  shank_imu{};
    rs02_feedback_t   fb{};
    StateMachine sm{};
    float joint_torque_cmd_filtered = 0.0f;

    load_runtime_params_once();
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
        const bool thigh_ok = (witmotion_get_latest_channel(IMU1_UART_NUM, &thigh_imu) == ESP_OK);
        const bool shank_ok = (witmotion_get_latest_channel(IMU2_UART_NUM, &shank_imu) == ESP_OK);
        imu = shank_imu;
        const bool imu_ok = shank_ok;
        const bool motor_ok = (rs02_get_feedback(&g_motor, &fb, 0) == ESP_OK);

        const float joint_mech_pos = motor_to_joint_rad(fb.position_rad);
        const float joint_user_pos = joint_mech_pos - g_user_zero_offset_rad;
        const float joint_vel = motor_to_joint_vel(fb.velocity_rad_s);

        if (g_req_zero_motor && motor_ok) {
            g_user_zero_offset_rad = joint_mech_pos;
            g_req_zero_motor = false;
            ESP_LOGW(TAG, "CMD zero motor: user_zero=%+.4f rad", g_user_zero_offset_rad);
            printf("$ACK,ZERO_MOTOR,%.5f\n", g_user_zero_offset_rad);
        }
        if (g_req_zero_imu && shank_ok) {
            g_shank_pitch_zero_deg =
                IMU_SHANK_PITCH_SIGN * shank_imu.euler_deg[IMU_SHANK_PITCH_AXIS];
            g_req_zero_imu = false;
            ESP_LOGW(TAG, "CMD zero IMU: shank_zero=%+.3f deg", g_shank_pitch_zero_deg);
            printf("$ACK,ZERO_IMU,%.5f\n", g_shank_pitch_zero_deg);
        }

        const MotionFeatures feat = update_features(imu, fb, imu_ok, motor_ok,
                                                    joint_mech_pos, joint_vel);
        update_state_machine(sm, feat);

        const float shank_pitch_rad = feat.shank_pitch_deg * DEG_TO_RAD;
        const float joint_gravity_ff = gravity_comp_torque(shank_pitch_rad);
        float joint_torque_ff = joint_gravity_ff;
        float joint_impedance_torque = 0.0f;

        const ImpedanceParams ip = g_state_impedance_enabled
                                 ? params_for_state(sm.state, sm.loops_in_state)
                                 : ImpedanceParams{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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

        const float max_torque_step = 2.0f / (float)LOOP_HZ; // Nm per control tick.
        const float torque_delta = clamp_float(joint_torque_ff - joint_torque_cmd_filtered,
                                               -max_torque_step,
                                               max_torque_step);
        joint_torque_cmd_filtered += torque_delta;

        rs02_motion_control(&g_motor,
                            joint_to_motor_torque(joint_torque_cmd_filtered),
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
                     joint_torque_cmd_filtered,
                     joint_gravity_ff,
                     joint_impedance_torque,
                     motor_to_joint_torque(fb.torque_nm),
                     feat.acc_norm_g,
                     feat.impact_g,
                     fb.temperature_c,
                     fb.has_fault ? " FAULT!" : "");
        }

        if ((loop_count % telemetry_every) == 0) {
            if (thigh_ok) {
                print_imu_telemetry("$IMU1", thigh_imu);
            }
            if (shank_ok) {
                print_imu_telemetry("$IMU2", shank_imu);
            }
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
                   joint_torque_cmd_filtered,
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

extern "C" void command_task(void *arg)
{
    (void)arg;
    char line[128];
    size_t n = 0;
    ESP_LOGI(TAG, "command task ready: $CMD,ZERO_IMU / ZERO_MOTOR / SET_GRAV,G,phi,bias / SAFE,0|1");

    while (true) {
        int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch != '\n' && n < sizeof(line) - 1) {
            line[n++] = (char)ch;
            continue;
        }

        line[n] = '\0';
        n = 0;
        if (strncmp(line, "$CMD,", 5) != 0) {
            continue;
        }

        char *cmd = line + 5;
        if (strcmp(cmd, "ZERO_IMU") == 0) {
            g_req_zero_imu = true;
            printf("$ACK,REQ_ZERO_IMU\n");
        } else if (strcmp(cmd, "ZERO_MOTOR") == 0) {
            g_req_zero_motor = true;
            printf("$ACK,REQ_ZERO_MOTOR\n");
        } else if (strncmp(cmd, "SAFE,", 5) == 0) {
            g_force_safe = atoi(cmd + 5) != 0;
            printf("$ACK,SAFE,%d\n", g_force_safe ? 1 : 0);
        } else if (strncmp(cmd, "SET_GRAV,", 9) == 0) {
            float G = 0.0f;
            float phi = 0.0f;
            float bias = 0.0f;
            if (sscanf(cmd + 9, "%f,%f,%f", &G, &phi, &bias) == 3) {
                g_gravity_G_nm = G;
                g_gravity_phi_rad = phi;
                g_gravity_bias_nm = bias;
                printf("$ACK,SET_GRAV,%.6f,%.6f,%.6f\n",
                       g_gravity_G_nm, g_gravity_phi_rad, g_gravity_bias_nm);
            } else {
                printf("$ERR,SET_GRAV,parse\n");
            }
        } else if (strcmp(cmd, "SAVE_PARAMS") == 0) {
            esp_err_t err = save_runtime_params();
            if (err == ESP_OK) {
                printf("$ACK,SAVE_PARAMS\n");
            } else {
                printf("$ERR,SAVE_PARAMS,%s\n", esp_err_to_name(err));
            }
        } else if (strncmp(cmd, "GRAV_ENABLE,", 12) == 0) {
            g_gravity_enabled = atoi(cmd + 12) != 0;
            printf("$ACK,GRAV_ENABLE,%d,G=%.6f,phi=%.6f,bias=%.6f\n",
                   g_gravity_enabled ? 1 : 0,
                   g_gravity_G_nm,
                   g_gravity_phi_rad,
                   g_gravity_bias_nm);
        } else if (strncmp(cmd, "STATE_CTRL,", 11) == 0) {
            g_state_impedance_enabled = atoi(cmd + 11) != 0;
            printf("$ACK,STATE_CTRL,%d\n", g_state_impedance_enabled ? 1 : 0);
        } else if (strncmp(cmd, "STATE_LOCK,", 11) == 0) {
            const char *arg_state = cmd + 11;
            if (strcmp(arg_state, "OFF") == 0 || strcmp(arg_state, "AUTO") == 0) {
                g_state_lock_enabled = false;
                printf("$ACK,STATE_LOCK,OFF\n");
            } else {
                ExoState requested = ExoState::SafePassive;
                if (parse_exo_state(arg_state, &requested)) {
                    g_state_lock_id = static_cast<uint8_t>(requested);
                    g_state_lock_enabled = true;
                    printf("$ACK,STATE_LOCK,%s\n", exo_state_name(requested));
                } else {
                    printf("$ERR,STATE_LOCK,unknown\n");
                }
            }
        } else if (strncmp(cmd, "SET_THRESH,", 11) == 0) {
            float impact = 0.0f;
            float swing = 0.0f;
            if (sscanf(cmd + 11, "%f,%f", &impact, &swing) == 2
                && impact >= 0.02f && impact <= 2.0f
                && swing >= 5.0f && swing <= 250.0f) {
                g_landing_impact_thresh_g = impact;
                g_swing_rate_thresh_dps = swing;
                printf("$ACK,SET_THRESH,%.4f,%.2f\n",
                       g_landing_impact_thresh_g,
                       g_swing_rate_thresh_dps);
            } else {
                printf("$ERR,SET_THRESH,parse_or_range\n");
            }
        } else if (strcmp(cmd, "GET_PARAMS") == 0) {
            printf("$ACK,PARAMS,imu_zero=%.6f,G=%.6f,phi=%.6f,bias=%.6f,grav_en=%d,state_ctrl=%d,state_lock=%d,impact=%.6f,swing=%.3f\n",
                   g_shank_pitch_zero_deg,
                   g_gravity_G_nm,
                   g_gravity_phi_rad,
                   g_gravity_bias_nm,
                   g_gravity_enabled ? 1 : 0,
                   g_state_impedance_enabled ? 1 : 0,
                   g_state_lock_enabled ? 1 : 0,
                   g_landing_impact_thresh_g,
                   g_swing_rate_thresh_dps);
        } else {
            printf("$ERR,UNKNOWN,%s\n", cmd);
        }
        fflush(stdout);
    }
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_NORMAL

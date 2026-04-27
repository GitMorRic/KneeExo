// profile_motor_test.cpp
// 电机调通 + 方向标定 profile：
//   阶段 A（默认 KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN=n）：
//     - 初始化 CAN + RS02
//     - 不使能电机，仅每 100ms 打印一次反馈 (mech_pos / vel / torque / temp)
//     - 用于检查 CAN 接线、电机供电、ID 正确
//
//   阶段 B（KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN=y）：
//     - 上电后等 3s（人离手）
//     - set_mode(MOTION) -> enable
//     - 记录当前位置作 zero_ref
//     - 软软地以 ±0.1 rad 来回摆动 8 个周期 (kp=2, kd=0.5)
//     - 每帧打印 motor_pos 与 joint_pos（通过 motor_to_joint_rad 换算）
//     - 结束后 stop(false) 退出
//   ⚠ AUTOSPIN 时务必把电机悬空、未连腿，方便观察转向；
//     转向不对 → 改 config.h 的 JOINT_DIR_SIGN。
//
//   阶段 C（KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB=y）：
//     - 仅限离体悬空机构使用
//     - 低速向负方向/正方向分别搜索机械限位
//     - 用“力矩升高 + 速度停滞 + 位置不再变化 + 持续时间”判定碰限位
//     - 只打印标定结果，不自动写入 flash / config.h

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <math.h>
#include <string.h>

#include "config.h"
#include "can_bus.h"
#include "rs02_motor.h"
#include "app_main.hpp"

using namespace ExoConfig;

static const char *TAG = "profile.motor";

// =============================================================================
// CAN 总线扫描：在 rs02_init (启动 rx_task) 之前调用，监听所有帧。
// 两阶段：
//   Pass 1：先单独发给默认 ID 0x7F，快速确认是否直接通；
//   Pass 2：若未找到，逐个扫描 0x01~0xFE（约 5s）。
// =============================================================================
static void scan_motor_ids(void)
{
    // --- 构造 type-17 读参数帧 (read mech_pos) ---
    auto make_req = [](twai_message_t *req, uint8_t motor_id) {
        memset(req, 0, sizeof(*req));
        req->extd = 1;
        req->data_length_code = 8;
        // make_ext_id(17, host_id, motor_id)
        req->identifier = ((uint32_t)(17u & 0x1Fu) << 24)
                        | ((uint32_t)RS02_HOST_ID   << 8)
                        | (uint32_t)motor_id;
        req->data[0] = RS02_IDX_MECH_POS & 0xFF;
        req->data[1] = (RS02_IDX_MECH_POS >> 8) & 0xFF;
    };

    // --- 收到一帧：尝试提取电机 ID 并打印 ---
    auto report_frame = [](const twai_message_t *r, uint8_t queried_id) -> uint8_t {
        if (!r->extd) return 0;
        uint8_t resp_type   = (r->identifier >> 24) & 0x1Fu;
        // RS02 响应帧：bit[15:8] = 电机自身 CAN_ID，bit[7:0] = 主机 CAN_ID
        uint8_t resp_motor  = (r->identifier >> 8) & 0xFFu;
        uint8_t resp_target = r->identifier & 0xFFu;
        ESP_LOGI("can.scan",
                 "[FOUND] queried=0x%02X → frame_id=0x%08X  type=%d  motor_id=0x%02X  target=0x%02X",
                 queried_id, r->identifier, resp_type, resp_motor, resp_target);
        return resp_motor;
    };

    // --- Pass 1：默认 ID 0x7F ---
    ESP_LOGI("can.scan", "=== CAN 电机扫描 Pass-1: 探测默认 ID 0x7F ===");
    twai_message_t req, resp;
    make_req(&req, 0x7Fu);
    can_bus_tx(&req, pdMS_TO_TICKS(10));
    if (can_bus_rx(&resp, pdMS_TO_TICKS(100)) == ESP_OK) {
        report_frame(&resp, 0x7Fu);
        ESP_LOGI("can.scan", "Pass-1 成功！默认 ID 0x7F 有响应，跳过 Pass-2。");
        return;
    }
    ESP_LOGW("can.scan", "Pass-1 未收到回应 → 开始 Pass-2 全范围扫描 (约5s)…");

    // --- Pass 2：扫描 0x01~0xFE ---
    ESP_LOGI("can.scan", "=== CAN 电机扫描 Pass-2: 逐ID探测 0x01-0xFE ===");
    int found = 0;
    for (uint16_t id = 1; id <= 0xFEu; id++) {
        make_req(&req, (uint8_t)id);
        can_bus_tx(&req, pdMS_TO_TICKS(5));
        if (can_bus_rx(&resp, pdMS_TO_TICKS(15)) == ESP_OK) {
            uint8_t mid = report_frame(&resp, (uint8_t)id);
            (void)mid;
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW("can.scan", "!! 全范围扫描未发现电机，请检查：");
        ESP_LOGW("can.scan", "   1) 电机 24~60V 主电是否已上电？");
        ESP_LOGW("can.scan", "   2) CANH/CANL 是否已从收发器连到电机？");
        ESP_LOGW("can.scan", "   3) 电机端 CANH-CANL 之间是否有 120Ω 终端电阻？");
        ESP_LOGW("can.scan", "   4) CAN 收发器 3.3V 供电是否正常？");
    } else {
        ESP_LOGI("can.scan", "=== Pass-2 共发现 %d 台电机 ===", found);
    }
}

static void log_feedback(const char *tag_extra, const rs02_feedback_t *fb)
{
    const float joint_rad = motor_to_joint_rad(fb->position_rad);
    ESP_LOGI(TAG,
             "%smotor pos=%+6.3f vel=%+6.3f tq=%+5.2f T=%4.1f mode=%d%s | "
             "joint pos=%+6.3f rad (%+6.1f deg)",
             tag_extra ? tag_extra : "",
             fb->position_rad, fb->velocity_rad_s,
             fb->torque_nm, fb->temperature_c,
             fb->mode_state, fb->has_fault ? " FAULT!" : "",
             joint_rad, joint_rad * 180.0f / (float)M_PI);
}

static esp_err_t read_mech_pos(rs02_handle_t *m, float *out)
{
    return rs02_read_param_f32(m, RS02_IDX_MECH_POS, out, pdMS_TO_TICKS(200));
}

#if CONFIG_KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB
static esp_err_t move_soft_to(rs02_handle_t *m, float target_pos)
{
    const float speed = 0.25f;  // rad/s, return-to-center speed
    const float kp = 3.0f;
    const float kd = 0.5f;
    const int loop_hz = 100;
    const TickType_t dt = pdMS_TO_TICKS(1000 / loop_hz);
    TickType_t last = xTaskGetTickCount();

    float start = 0.0f;
    if (read_mech_pos(m, &start) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }

    const float travel = target_pos - start;
    const int steps = (int)(fabsf(travel) / speed * loop_hz) + 1;
    const int max_steps = steps > 600 ? 600 : steps;

    for (int i = 0; i <= max_steps; i++) {
        const float a = (max_steps == 0) ? 1.0f : (float)i / (float)max_steps;
        const float cmd = start + travel * a;
        rs02_motion_control(m, 0.0f, cmd, 0.0f, kp, kd);
        vTaskDelayUntil(&last, dt);
    }

    for (int i = 0; i < 50; i++) {
        rs02_motion_control(m, 0.0f, target_pos, 0.0f, kp, kd);
        vTaskDelayUntil(&last, dt);
    }
    return ESP_OK;
}

static esp_err_t find_limit(rs02_handle_t *m, int dir, float start_pos,
                            const char *name, float *limit_out)
{
    *limit_out = NAN;
    const float torque_thresh = (float)CONFIG_KNEEEXO_LIMIT_CALIB_TORQUE_X100 / 100.0f;
    const float search_speed = (float)CONFIG_KNEEEXO_LIMIT_CALIB_SPEED_MRAD_S / 1000.0f;
    const float max_travel = (float)CONFIG_KNEEEXO_LIMIT_CALIB_MAX_TRAVEL_MRAD / 1000.0f;
    const int hold_loops_need = CONFIG_KNEEEXO_LIMIT_CALIB_HOLD_MS / 10;
    const float vel_stop = 0.15f;        // rad/s, allow gearbox jitter near hard stop
    const float pos_stall_eps = 0.015f;  // rad per 10ms loop
    const float follow_error_min = 0.25f; // cmd-pos error proves the axis cannot follow
    const float kp = 3.0f;
    const float kd = 0.8f;
    const int loop_hz = 100;
    const TickType_t dt = pdMS_TO_TICKS(1000 / loop_hz);

    int hold_loops = 0;
    float last_pos = start_pos;
    float best_candidate = NAN;
    float best_torque = 0.0f;
    TickType_t last = xTaskGetTickCount();

    ESP_LOGW(TAG,
             "Limit search %s: dir=%+d speed=%.3f rad/s torque_thresh=%.2f Nm "
             "max_travel=%.3f rad hold=%dms",
             name, dir, search_speed, torque_thresh, max_travel,
             CONFIG_KNEEEXO_LIMIT_CALIB_HOLD_MS);

    for (int i = 0; i < (int)(max_travel / search_speed * loop_hz); i++) {
        const float cmd = start_pos + (float)dir * search_speed * ((float)i / (float)loop_hz);
        rs02_motion_control(m, 0.0f, cmd, 0.0f, kp, kd);

        rs02_feedback_t fb;
        if (rs02_get_feedback(m, &fb, pdMS_TO_TICKS(5)) == ESP_OK) {
            const float torque_abs = fabsf(fb.torque_nm);
            const float vel_abs = fabsf(fb.velocity_rad_s);
            const float pos_delta = fabsf(fb.position_rad - last_pos);
            const float follow_error = fabsf(cmd - fb.position_rad);
            const bool torque_high = torque_abs >= torque_thresh;
            const bool stopped_by_encoder = (vel_abs <= vel_stop) && (pos_delta <= pos_stall_eps);
            const bool cannot_follow_cmd = follow_error >= follow_error_min;
            const bool stalled = torque_high && (stopped_by_encoder || cannot_follow_cmd);

            if (torque_high && (isnan(best_candidate) || torque_abs >= best_torque)) {
                best_candidate = fb.position_rad;
                best_torque = torque_abs;
            }

            if ((i % 25) == 0) {
                ESP_LOGI(TAG, "  %s cmd=%+.3f pos=%+.3f err=%.3f vel=%+.3f tq=%+.2f hold=%d/%d",
                         name, cmd, fb.position_rad, follow_error, fb.velocity_rad_s,
                         fb.torque_nm, hold_loops, hold_loops_need);
            }

            if (stalled) {
                hold_loops++;
                if (hold_loops >= hold_loops_need) {
                    *limit_out = fb.position_rad;
                    ESP_LOGW(TAG, "  %s LIMIT detected at motor_pos=%+.4f rad (tq=%+.2f Nm)",
                             name, *limit_out, fb.torque_nm);
                    return ESP_OK;
                }
            } else {
                hold_loops = 0;
            }
            last_pos = fb.position_rad;
        }

        vTaskDelayUntil(&last, dt);
    }

    ESP_LOGE(TAG, "%s search exceeded max_travel %.3f rad without detecting a hard stop",
             name, max_travel);
    if (!isnan(best_candidate)) {
        *limit_out = best_candidate;
        ESP_LOGW(TAG, "  %s candidate kept at motor_pos=%+.4f rad (max observed |tq|=%.2f Nm)",
                 name, best_candidate, best_torque);
    }
    return ESP_ERR_NOT_FOUND;
}

static void run_limit_calibration(rs02_handle_t *m)
{
    ESP_LOGW(TAG, "Phase C: LIMIT_CALIB enabled. MOTOR/MECHANISM MUST BE OFF-BODY AND SUSPENDED.");
    ESP_LOGW(TAG, "Emergency plan: cut motor power if anything looks wrong.");
    for (int i = 5; i > 0; i--) {
        ESP_LOGW(TAG, "  calibration starts in %d ...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(rs02_set_mode(m, RS02_MODE_MOTION));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(rs02_enable(m));
    vTaskDelay(pdMS_TO_TICKS(50));

    float start_pos = 0.0f;
    if (read_mech_pos(m, &start_pos) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read start mech_pos; abort calibration.");
        rs02_stop(m, false);
        return;
    }
    ESP_LOGI(TAG, "Calibration start motor_pos=%+.4f rad", start_pos);
    move_soft_to(m, start_pos);

    float neg_limit = 0.0f;
    float pos_limit = 0.0f;
    esp_err_t neg_ok = find_limit(m, -1, start_pos, "NEG", &neg_limit);
    rs02_stop(m, false);
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_ERROR_CHECK(rs02_enable(m));
    move_soft_to(m, start_pos);
    vTaskDelay(pdMS_TO_TICKS(300));

    esp_err_t pos_ok = find_limit(m, +1, start_pos, "POS", &pos_limit);
    rs02_stop(m, false);

    const bool neg_have = !isnan(neg_limit);
    const bool pos_have = !isnan(pos_limit);
    ESP_LOGW(TAG, "=== LIMIT CALIB RAW SUMMARY ===");
    ESP_LOGW(TAG, "NEG result: %s, motor_pos=%s%+.5f",
             esp_err_to_name(neg_ok), neg_have ? "" : "(none) ",
             neg_have ? neg_limit : 0.0f);
    ESP_LOGW(TAG, "POS result: %s, motor_pos=%s%+.5f",
             esp_err_to_name(pos_ok), pos_have ? "" : "(none) ",
             pos_have ? pos_limit : 0.0f);

    if (!neg_have || !pos_have) {
        ESP_LOGE(TAG, "Limit calibration incomplete: at least one side has no usable candidate.");
        ESP_LOGE(TAG, "Try lower torque threshold or larger max travel only after confirming the hard stop is safe.");
        return;
    }

    if (neg_ok != ESP_OK || pos_ok != ESP_OK) {
        ESP_LOGW(TAG, "At least one side is a CANDIDATE, not a strict detection. Visually verify before copying values.");
    }

    if (neg_limit > pos_limit) {
        const float tmp = neg_limit;
        neg_limit = pos_limit;
        pos_limit = tmp;
    }

    const float range = pos_limit - neg_limit;
    const float margin = 0.05f;
    ESP_LOGW(TAG, "=== LIMIT CALIB RESULT (raw motor coordinates) ===");
    ESP_LOGW(TAG, "motor_negative_limit_rad = %+.5f", neg_limit);
    ESP_LOGW(TAG, "motor_positive_limit_rad = %+.5f", pos_limit);
    ESP_LOGW(TAG, "mechanical_range_rad     = %.5f rad (%.1f deg)",
             range, range * 180.0f / (float)M_PI);
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "If NEG side is straight/extension and POS side is flexion:");
    ESP_LOGW(TAG, "  edit: components/config/include/config.h");
    ESP_LOGW(TAG, "  raw motor extension limit = %+.5f", neg_limit);
    ESP_LOGW(TAG, "  raw motor flexion   limit = %+.5f", pos_limit);
    ESP_LOGW(TAG, "  JOINT_DIR_SIGN        = +1.0f");
    ESP_LOGW(TAG, "  JOINT_ZERO_OFFSET_RAD = %+.5f", neg_limit);
    ESP_LOGW(TAG, "  KNEE_EXT_LIMIT_RAD    = 0.0f     // joint coord: straight leg");
    ESP_LOGW(TAG, "  KNEE_FLEX_LIMIT_RAD   = %.5f   // joint coord, with %.2f rad margin",
             range - margin, margin);
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "If POS side is straight/extension and NEG side is flexion:");
    ESP_LOGW(TAG, "  edit: components/config/include/config.h");
    ESP_LOGW(TAG, "  raw motor extension limit = %+.5f", pos_limit);
    ESP_LOGW(TAG, "  raw motor flexion   limit = %+.5f", neg_limit);
    ESP_LOGW(TAG, "  JOINT_DIR_SIGN        = -1.0f");
    ESP_LOGW(TAG, "  JOINT_ZERO_OFFSET_RAD = %+.5f", pos_limit);
    ESP_LOGW(TAG, "  KNEE_EXT_LIMIT_RAD    = 0.0f     // joint coord: straight leg");
    ESP_LOGW(TAG, "  KNEE_FLEX_LIMIT_RAD   = %.5f   // joint coord, with %.2f rad margin",
             range - margin, margin);
    ESP_LOGW(TAG, "Copy the matching JOINT-space option into config.h after visual confirmation.");
}
#endif

extern "C" void profile_motor_test_main(void)
{
    ESP_ERROR_CHECK(can_bus_init(PIN_TWAI_TX, PIN_TWAI_RX, CAN_BITRATE_HZ));

    // 扫描必须在 rs02_init 之前：rx_task 启动后会独占所有收帧
    scan_motor_ids();

    static rs02_handle_t m;
    ESP_ERROR_CHECK(rs02_init(&m, RS02_CAN_ID, RS02_HOST_ID));
    vTaskDelay(pdMS_TO_TICKS(200));

    // ---------------- 阶段 A：只读反馈 ----------------
    ESP_LOGI(TAG, "Phase A: feedback-only (motor NOT enabled). 5s window.");
    for (int i = 0; i < 50; i++) {
        rs02_feedback_t fb;
        if (rs02_get_feedback(&m, &fb, pdMS_TO_TICKS(150)) == ESP_OK) {
            if (i % 5 == 0) log_feedback("A: ", &fb);
        } else {
            // 没反馈：rs02_motion_control 不发，电机不会主动上报；这是正常的。
            // 我们手动主动读一次 mech_pos 看看 CAN 通不通。
            float mech = 0.0f;
            esp_err_t er = rs02_read_param_f32(&m, RS02_IDX_MECH_POS, &mech, pdMS_TO_TICKS(200));
            if (er == ESP_OK) {
                ESP_LOGI(TAG, "A: read_param mech_pos=%+6.3f rad (CAN OK, motor idle)", mech);
            } else if (i == 0) {
                ESP_LOGW(TAG, "A: no feedback & read_param timeout (%s) — check CAN wiring / power",
                         esp_err_to_name(er));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

#if CONFIG_KNEEEXO_PROFILE_MOTOR_LIMIT_CALIB
    run_limit_calibration(&m);
#elif CONFIG_KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN
    // ---------------- 阶段 B：自动 ±0.1 rad 摆动 ----------------
    ESP_LOGW(TAG, "Phase B: AUTOSPIN enabled. 3s pause, KEEP HANDS OFF MOTOR!");
    for (int i = 3; i > 0; i--) {
        ESP_LOGW(TAG, "  enabling motor in %d ...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(rs02_set_mode(&m, RS02_MODE_MOTION));
    vTaskDelay(pdMS_TO_TICKS(20));

    // 读当前位置作零参考
    rs02_feedback_t fb0 = {};
    rs02_get_feedback(&m, &fb0, pdMS_TO_TICKS(200));
    const float pos0 = fb0.position_rad;
    ESP_LOGI(TAG, "zero ref motor_pos = %+6.3f rad", pos0);

    ESP_ERROR_CHECK(rs02_enable(&m));
    vTaskDelay(pdMS_TO_TICKS(20));

    const float amp     = 0.10f;   // ±0.1 rad
    const float Kp      = 2.0f;    // 软
    const float Kd      = 0.5f;
    const int   periods = 8;
    const int   loop_hz = 100;
    const int   total_loops = periods * loop_hz;  // 8s @ 100Hz
    const TickType_t dt = pdMS_TO_TICKS(1000 / loop_hz);
    TickType_t last = xTaskGetTickCount();

    for (int i = 0; i < total_loops; i++) {
        // 1Hz 正弦摆动
        const float phase = 2.0f * (float)M_PI * (float)i / (float)loop_hz;
        const float pos_cmd = pos0 + amp * sinf(phase);
        rs02_motion_control(&m, 0.0f, pos_cmd, 0.0f, Kp, Kd);

        rs02_feedback_t fb;
        if (rs02_get_feedback(&m, &fb, 0) == ESP_OK && (i % 25 == 0)) {
            ESP_LOGI(TAG, "B: cmd=%+6.3f  motor=%+6.3f  joint=%+6.3f rad",
                     pos_cmd, fb.position_rad,
                     motor_to_joint_rad(fb.position_rad));
        }
        vTaskDelayUntil(&last, dt);
    }

    rs02_stop(&m, false);
    ESP_LOGI(TAG, "Phase B done, motor stopped.");
    ESP_LOGW(TAG, "标定结论方法：观察电机在 cmd 增大时是屈膝(+) 还是伸膝(-)。");
    ESP_LOGW(TAG, "  屈膝 => JOINT_DIR_SIGN = +1.0f (当前默认)");
    ESP_LOGW(TAG, "  伸膝 => 改 config.h JOINT_DIR_SIGN = -1.0f, 重新 build flash。");
#else
    ESP_LOGI(TAG, "AUTOSPIN/LIMIT_CALIB disabled. Use '.\\tools\\flash.ps1 limit COMx' for off-body limit calibration.");
#endif

    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST

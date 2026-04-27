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

#include "sdkconfig.h"

#if CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <math.h>

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

#if CONFIG_KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN
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
    ESP_LOGI(TAG, "AUTOSPIN disabled. Set KNEEEXO_PROFILE_MOTOR_TEST_AUTOSPIN=y to spin.");
#endif

    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // CONFIG_KNEEEXO_APP_PROFILE_MOTOR_TEST

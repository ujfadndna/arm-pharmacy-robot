#include "new_thread0.h"
#include "hal_data.h"
#include "common_data.h"
#include "app_common.h"
#include "w800_driver.h"
#include "debug_uart.h"
/* #include "llm_action.h" */  /* LLM disabled */
#include "motion_controller.h"
#include "visual_servo.h"
#include "gripper.h"
#include "watchdog.h"
#include "rtmon.h"
#include "taskmon.h"
#include "degradation.h"
#include "dummy_arm_cmd.h"
#include "usb_host_cdc.h"
#include "audio.h"
#include "drv_iic0.h"
#include "maixcam_uart.h"
#include "slot_teach.h"
#include "vacuum_pump.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ========== 看门狗配�?========== */
#define WATCHDOG_ENABLED    1   /* 1=启用看门�? 0=禁用(调试�? */

#define LED_PIN BSP_IO_PORT_04_PIN_00

/* ========== LLM响应缓冲�?(LLM disabled) ========== */
#if 0
#define LLM_RESP_BUF_SIZE 1024
static char g_llm_response[LLM_RESP_BUF_SIZE];
static volatile uint16_t g_llm_resp_idx = 0;
static volatile bool g_llm_resp_ready = false;
#endif

/* ========== 动作序列 (LLM disabled) ========== */
#if 0
static llm_action_sequence_t g_action_seq;
static bool g_executing_sequence = false;
#endif
static bool g_arm_passthrough_enabled = false; /* 是否允许!/#/>/@直通命�?*/

/* ========== 硬件定时器运动控�?========== */
static volatile bool g_motion_tick = false;  /* 5ms定时器触发标�?*/

extern void vs_on_align_ok(const mc_align_ok_t *r);
extern void vs_on_align_fail(const mc_align_fail_t *r);

static maixcam_uart_callbacks_t g_mc_callbacks = {
    .on_align_ok   = vs_on_align_ok,
    .on_align_fail = vs_on_align_fail,
};

/**
 * @brief GPT定时器回�?- 5ms周期触发
 * 保留用于看门狗喂狗和心跳LED
 */
void motion_timer_callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event) {
        rtmon_tick(5000);  /* Record timestamp for jitter monitoring, 5ms=5000us */
        g_motion_tick = true;

        /* LED 心跳 — 在定时器 ISR 中直接控制，独立于主循环阻塞
         * R_IOPORT_PinWrite 是纯寄存器操作，ISR 安全。
         * 周期：100 × 5ms = 500ms；ON = 0~49tick，OFF = 50~99tick */
        static uint32_t s_led_counter = 0;
        s_led_counter++;
        if (s_led_counter >= 100U) {
            s_led_counter = 0U;
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN, BSP_IO_LEVEL_HIGH);
        } else if (s_led_counter == 50U) {
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN, BSP_IO_LEVEL_LOW);
        }
    }
}

/* W800接收数据回调 - LLM disabled, unused */
#if 0
static void w800_data_received(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        char ch = (char)data[i];

        if (g_llm_resp_idx < LLM_RESP_BUF_SIZE - 1) {
            g_llm_response[g_llm_resp_idx++] = ch;
            g_llm_response[g_llm_resp_idx] = '\0';
        }
    }
    if (g_llm_resp_idx > 0) {
        char last = g_llm_response[g_llm_resp_idx - 1];
        if (last == ']' || last == '}') {
            g_llm_resp_ready = true;
        }
    }
}
#endif

/* LLM disabled: execute_action / execute_next_action / process_llm_response / send_command_to_llm */
#if 0
/* 执行单个动作 (返回true表示需要等待运动完�? */
static bool execute_action(const llm_action_t *action)
{
    debug_print("[EXEC] ");
    debug_println(llm_action_type_str(action->type));

    switch (action->type) {
        case ACTION_MOVE:
            debug_print("  -> x=");
            debug_print_int((int)action->params.move.x);
            debug_print(" y=");
            debug_print_int((int)action->params.move.y);
            debug_print(" z=");
            debug_print_int((int)action->params.move.z);
            debug_println("");
            /* 调用运动控制�?(已切换到USB后端) */
            if (motion_move_to_xyz(action->params.move.x,
                                   action->params.move.y,
                                   action->params.move.z) == 0) {
                /* motion_move_to_xyz内部会等待到位后返回�?
                 * 这里不需要额外异步等�?*/
                return false;
            }
            break;

        case ACTION_GRAB:
            debug_println("  -> Gripper CLOSE");
            gripper_close();
            break;

        case ACTION_RELEASE:
            debug_println("  -> Gripper OPEN");
            gripper_open();
            break;

        case ACTION_SPEAK:
            debug_print("  -> Say: ");
            debug_println(action->params.speak.text);
            maixcam_uart_send_speak(action->params.speak.text,
                                    (uint8_t)strlen(action->params.speak.text));
            break;

        case ACTION_SCAN_QR:
            debug_println("  -> Scanning QR...");
            maixcam_uart_send_req_qr(5000);
            break;

        default:
            debug_println("  -> Unknown action");
            break;
    }
    return false;  /* 不需要等�?*/
}

/* 执行下一个动�?*/
static void execute_next_action(void)
{
    while (g_action_seq.current < g_action_seq.count) {
        bool need_wait = execute_action(&g_action_seq.actions[g_action_seq.current]);
        g_action_seq.current++;

        if (need_wait) {
            /* 需要等待运动完�?*/
            return;
        }
        /* 不需要等待，继续下一个动�?*/
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* 所有动作完�?*/
    g_executing_sequence = false;
    debug_println("[SEQ] All actions done");
}

/* 处理LLM返回的JSON动作序列 */
static void process_llm_response(const char *json)
{
    debug_print("[LLM] Parsing: ");
    debug_println(json);

    /* 解析JSON */
    int ret = llm_parse_json(json, &g_action_seq);
    if (ret != 0 || g_action_seq.count == 0) {
        debug_println("[LLM] Parse failed or empty!");
        return;
    }

    debug_print("[LLM] Parsed ");
    debug_print_int(g_action_seq.count);
    debug_println(" action(s)");

    /* 开始执行动作序�?*/
    g_action_seq.current = 0;
    g_executing_sequence = true;
    execute_next_action();
}

/* 发送用户指令到云端LLM (通过W800直连) */
static int send_command_to_llm(const char *cmd)
{
    if (w800_get_state() != W800_STATE_TRANSPARENT) {
        debug_println("[ERR] WiFi not connected");
        return -1;
    }

    debug_print("[LLM] Query: ");
    debug_println(cmd);
    debug_println("[LLM] Calling cloud LLM (may take 5-15 seconds)...");

    /* 调用W800直连LLM */
    char response[1024];
    int ret = w800_call_llm_with_retry(cmd, response, sizeof(response), 2);

    if (ret > 0) {
        debug_print("[LLM] Response (");
        debug_print_int(ret);
        debug_println(" bytes):");
        debug_println(response);

        /* 解析并执行动�?*/
        process_llm_response(response);
        return 0;
    } else {
        debug_print("[LLM] Failed: ");
        debug_print_int(ret);
        debug_println("");
        return ret;
    }
}
#endif /* LLM disabled */

static void log_motion_command_result(const char *tag, int ret)
{
    debug_print("[");
    debug_print(tag);
    debug_print("] ");

    if (ret == 0) {
        debug_println("Motion command accepted.");
    } else {
        debug_print("Motion command failed, ret=");
        debug_print_int(ret);
        debug_println("");
    }
}

/* Print current joint positions from Dummy ARM for debug */
static void print_joint_state(const char *label)
{
    if (!usb_cdc_is_ready()) return;
    dummy_joint_pos_t jp;
    if (dummy_arm_get_joint_pos(&jp) == 0) {
        /* 同步最新关节角到软件缓存，防止后续命令使用过期数据 */
        float joints[6] = {jp.j[0], jp.j[1], jp.j[2], jp.j[3], jp.j[4], jp.j[5]};
        motion_set_current_joints(joints);

        char buf[128];
        snprintf(buf, sizeof(buf), "[%s] J: %.1f %.1f %.1f %.1f %.1f %.1f",
                 label, jp.j[0], jp.j[1], jp.j[2], jp.j[3], jp.j[4], jp.j[5]);
        debug_println(buf);
    }
}

/**
 * @brief 等待运动到位后打印关节状�?
 * 轮询关节位置，连�?次读数变�?< 0.3 度视为到�?
 * 查询失败(电机�?时跳过继续轮询，不计数不放弃
 * 最多等�?15 �?(50 × 300ms) 覆盖 HOME/REST 等长距离运动
 */
static void wait_and_print_after(void)
{
    if (!usb_cdc_is_ready()) return;

    dummy_joint_pos_t prev, cur;
    int stable = 0;
    int got_reading = 0;
    int consecutive_fail = 0;
    const int MAX_POLLS = 50;   /* 50 × 300ms = 15s 上限 */

    for (int i = 0; i < MAX_POLLS; i++) {
        vTaskDelay(pdMS_TO_TICKS(300));
        watchdog_refresh();   /* 防止长等待触发看门狗复位 */

        if (!usb_cdc_is_ready()) break;  /* USB断线，立即退�?*/

        if (dummy_arm_get_joint_pos(&cur) != 0) {
            consecutive_fail++;
            if (consecutive_fail >= 3) {
                debug_println("[AFTER] USB query failed 3x, giving up");
                debug_println("[AFTER] >> USB pipe may be stuck. Try: re-plug cable or reset board <<");
                break;
            }
            continue;  /* 查询失败(电机�?，跳过继续轮�?*/
        }
        consecutive_fail = 0;

        if (!got_reading) {
            got_reading = 1;
            prev = cur;
            continue;   /* 第一次读数作为基准，继续 */
        }

        /* 检查所有轴变化�?*/
        float max_diff = 0.0f;
        for (int j = 0; j < 6; j++) {
            float d = cur.j[j] - prev.j[j];
            if (d < 0) d = -d;
            if (d > max_diff) max_diff = d;
        }

        if (max_diff < 0.3f) {
            stable++;
            if (stable >= 3) break;  /* 连续3次稳�?�?到位 */
        } else {
            stable = 0;
        }
        prev = cur;
    }

    if (got_reading) {
        char buf[128];
        snprintf(buf, sizeof(buf), "[AFTER] J: %.1f %.1f %.1f %.1f %.1f %.1f",
                 cur.j[0], cur.j[1], cur.j[2], cur.j[3], cur.j[4], cur.j[5]);
        debug_println(buf);
    }
}

/**
 * @brief 本地测试命令处理
 * @return true 如果是测试命令并已处�?
 */
static bool handle_test_command(const char *cmd)
{
    /* 调试：打印原始命令，便于观察行结束符/空白等问�?*/
    debug_print("[CMD] RAW: ");
    debug_println(cmd);

    /* ========== 透传调试开�? debug arm on/off ========== */
    if (strcmp(cmd, "debug arm on") == 0) {
        g_arm_passthrough_enabled = true;
        debug_println("[DEBUG] ARM passthrough enabled (!/#/>/@).");
        return true;
    }
    if (strcmp(cmd, "debug arm off") == 0) {
        g_arm_passthrough_enabled = false;
        debug_println("[DEBUG] ARM passthrough disabled.");
        return true;
    }

    /* ========== 网络连通性测试命�? nettest ========== */
    if (strncmp(cmd, "nettest", 7) == 0) {
        debug_println("[CMD] Running network connectivity test...");
        w800_test_network();
        return true;
    }

    /* ========== 调试：查看累计RX字节�? uartdebug ========== */
    if (strcmp(cmd, "uartdebug") == 0) {
        char buf[64];
        uint32_t rx_bytes = debug_uart_get_rx_bytes();
        snprintf(buf, sizeof(buf), "[DEBUG] RX bytes: %lu", (unsigned long)rx_bytes);
        debug_println(buf);
        return true;
    }

    /* ========== 调试串口状�? uartstat ========== */
    if (strcmp(cmd, "uartstat") == 0) {
        debug_uart_stats_t st = {0};
        debug_uart_get_stats(&st);

        debug_print("[UART4] queue=");
        debug_print_int((int)st.queue_count);
        debug_print(" dropped=");
        debug_print_int((int)st.dropped_lines);
        debug_print(" overflow=");
        debug_print_int((int)st.overflow_chars);
        debug_print(" errors=");
        debug_print_int((int)st.uart_errors);
        debug_println("");
        return true;
    }

    /* ========== JSON命令测试: json <cmd> ========== */
    if (strncmp(cmd, "json ", 5) == 0) {
        const char *json = cmd + 5;
        debug_print("[TEST] Direct JSON cmd: ");
        debug_println(json);
        w800_cmd_handle(json);
        return true;
    }

    /* 实时性监测命�? rtmon [reset] */
    if (strncmp(cmd, "rtmon", 5) == 0) {
        if (strstr(cmd, "reset") != NULL) {
            rtmon_reset();
            rtmon_wcet_reset();
            debug_println("[RTMON] Statistics reset.");
        } else {
            char rtmon_buf[512];
            rtmon_print_report(rtmon_buf, sizeof(rtmon_buf));
            debug_println(rtmon_buf);
        }
        return true;
    }

    /* 任务监控命令: taskmon [reset] */
    if (strncmp(cmd, "taskmon", 7) == 0) {
        if (strstr(cmd, "reset") != NULL) {
            taskmon_reset();
            debug_println("[TASKMON] Statistics reset.");
        } else {
            taskmon_update("main_thread", 0);
            char taskmon_buf[512];
            taskmon_print_report(taskmon_buf, sizeof(taskmon_buf));
            debug_println(taskmon_buf);
        }
        return true;
    }

    /* HTTP LLM测试命令: http <query> - LLM disabled */
#if 0
    if (strncmp(cmd, "http ", 5) == 0) {
        const char *query = cmd + 5;
        char response[1024];
        int ret = w800_call_llm_with_retry(query, response, sizeof(response), 3);
        if (ret > 0) {
            process_llm_response(response);
        }
        return true;
    }
#endif

    /* 急停命令 (走USB) */
    if (strncmp(cmd, "stop", 4) == 0) {
        print_joint_state("BEFORE");
        debug_println("[STOP] Emergency stop!");
        motion_stop();
        wait_and_print_after();
        return true;
    }

    /* 回HOME: home (统一走motion_controller) */
    if (strcmp(cmd, "home") == 0) {
        static const float HOME_POSE[6] = {0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f};
        print_joint_state("BEFORE");
        int ret = motion_move_to_joints(HOME_POSE);
        log_motion_command_result("HOME", ret);
        wait_and_print_after();
        return true;
    }

    /* 回REST: rest (统一走motion_controller) */
    if (strcmp(cmd, "rest") == 0) {
        static const float REST_POSE[6] = {0.0f, -73.0f, 180.0f, 0.0f, 0.0f, 0.0f};
        print_joint_state("BEFORE");
        int ret = motion_move_to_joints(REST_POSE);
        log_motion_command_result("REST", ret);
        wait_and_print_after();
        return true;
    }

    /* 单轴运动: j <joint> <angle> (统一走motion_controller �?USB) */
    if (cmd[0] == 'j' && cmd[1] == ' ') {
        int joint = -1;
        float angle = 0.0f;
        if (sscanf(cmd, "j %d %f", &joint, &angle) != 2) {
            debug_println("[JOINT] Usage: j <id> <angle>");
            return true;
        }
        if (joint < 1 || joint > 6) {
            debug_println("[JOINT] Error: joint should be 1-6");
            return true;
        }

        float target_joints[6];
        motion_get_current_joints(target_joints);
        target_joints[joint - 1] = angle;

        debug_print("[JOINT] J");
        debug_print_int(joint);
        debug_print(" -> ");
        debug_print_int((int) angle);
        debug_println(" deg");

        print_joint_state("BEFORE");
        int ret = motion_move_to_joints(target_joints);
        log_motion_command_result("JOINT", ret);
        wait_and_print_after();
        return true;
    }

    /* 相对关节运动: jr <id> <delta> (走USB) */
    if (cmd[0] == 'j' && cmd[1] == 'r' && cmd[2] == ' ') {
        int joint = -1;
        float delta = 0.0f;
        if (sscanf(cmd, "jr %d %f", &joint, &delta) != 2) {
            debug_println("[JR] Usage: jr <id> <delta>");
            return true;
        }
        if (joint < 1 || joint > 6) {
            debug_println("[JR] Error: joint should be 1-6");
            return true;
        }

        float target_joints[6];
        motion_get_current_joints(target_joints);
        float new_angle = target_joints[joint - 1] + delta;
        target_joints[joint - 1] = new_angle;

        char buf[64];
        snprintf(buf, sizeof(buf), "[JR] J%d %+.1f -> %.1f deg", joint, delta, new_angle);
        debug_println(buf);

        print_joint_state("BEFORE");
        int ret = motion_move_to_joints(target_joints);
        log_motion_command_result("JR", ret);
        wait_and_print_after();
        return true;
    }

    /* 六轴同步运动: m <j0> <j1> <j2> <j3> <j4> <j5> (走USB) */
    if (cmd[0] == 'm' && cmd[1] == ' ') {
        float target_joints[6];
        if (sscanf(cmd, "m %f %f %f %f %f %f",
                   &target_joints[0], &target_joints[1], &target_joints[2],
                   &target_joints[3], &target_joints[4], &target_joints[5]) != 6) {
            debug_println("[MOTION] Usage: m <j0> <j1> <j2> <j3> <j4> <j5>");
            return true;
        }

        debug_println("[MOTION] Multi-joint move...");
        print_joint_state("BEFORE");
        int ret = motion_move_to_joints(target_joints);
        log_motion_command_result("MOTION", ret);
        wait_and_print_after();
        return true;
    }

    /* 笛卡尔相对运�? x <dx> <dy> <dz> (走USB) */
    if (cmd[0] == 'x' && cmd[1] == ' ') {
        float dx = 0.0f, dy = 0.0f, dz = 0.0f;
        if (sscanf(cmd, "x %f %f %f", &dx, &dy, &dz) != 3) {
            debug_println("[MOTION] Usage: x <dx> <dy> <dz>");
            return true;
        }

        debug_print("[MOTION] DXYZ -> ");
        debug_print_int((int) dx);
        debug_print(" ");
        debug_print_int((int) dy);
        debug_print(" ");
        debug_print_int((int) dz);
        debug_println("");

        print_joint_state("BEFORE");
        int ret = motion_move_by_xyz(dx, dy, dz);
        log_motion_command_result("MOTION", ret);
        wait_and_print_after();
        return true;
    }

    /* 夹爪控制: grip open/close/vacuum on/off */
    if (strncmp(cmd, "grip ", 5) == 0) {
        const char *p = cmd + 5;
        if (strncmp(p, "open", 4) == 0) {
            debug_println("[GRIP] Opening gripper...");
            gripper_open();
        } else if (strncmp(p, "close", 5) == 0) {
            debug_println("[GRIP] Closing gripper...");
            gripper_close();
        } else if (strncmp(p, "vacuum on", 9) == 0) {
            debug_println("[GRIP] Vacuum ON...");
            vacuum_on();
        } else if (strncmp(p, "vacuum off", 10) == 0) {
            debug_println("[GRIP] Vacuum OFF...");
            vacuum_off();
        } else {
            debug_println("[GRIP] Usage: grip open/close/vacuum on/vacuum off");
        }
        return true;
    }

    /* 格子示教再现: slot teach/goto/grab/list/clear <id> */
    if (strncmp(cmd, "slot ", 5) == 0) {
        const char *p = cmd + 5;
        int id = -1;

        if (strncmp(p, "teach ", 6) == 0) {
            if (sscanf(p + 6, "%d", &id) == 1)
                slot_teach((uint8_t)id);
            else
                debug_println("[SLOT] Usage: slot teach <0-15>");
        } else if (strncmp(p, "goto ", 5) == 0) {
            if (sscanf(p + 5, "%d", &id) == 1) {
                slot_goto((uint8_t)id);
                wait_and_print_after();
            } else {
                debug_println("[SLOT] Usage: slot goto <0-15>");
            }
        } else if (strncmp(p, "grab ", 5) == 0) {
            if (sscanf(p + 5, "%d", &id) == 1)
                slot_grab((uint8_t)id);
            else
                debug_println("[SLOT] Usage: slot grab <0-15>");
        } else if (strncmp(p, "fetch ", 6) == 0) {
            if (sscanf(p + 6, "%d", &id) == 1)
                slot_fetch((uint8_t)id);
            else
                debug_println("[SLOT] Usage: slot fetch <0-15>");
        } else if (strncmp(p, "list", 4) == 0) {
            slot_list();
        } else if (strncmp(p, "clear ", 6) == 0) {
            if (sscanf(p + 6, "%d", &id) == 1)
                slot_clear((uint8_t)id);
            else
                debug_println("[SLOT] Usage: slot clear <0-15>");
        } else {
            debug_println("[SLOT] Commands:");
            debug_println("  slot teach <id>  - record current joints as slot id");
            debug_println("  slot goto  <id>  - move to slot id");
            debug_println("  slot grab  <id>  - move + vacuum on + retract");
            debug_println("  slot fetch <id>  - full pick: grab + off + release + home");
            debug_println("  slot list        - show all taught slots");
            debug_println("  slot clear <id>  - clear slot id");
        }
        return true;
    }

    /* 清除故障: clear */
    if (strncmp(cmd, "clear", 5) == 0) {
        motion_clear_stall(0xFF);
        degradation_clear();
        debug_println("[CLEAR] Fault cleared.");
        return true;
    }

    /* 重新初始化臂: reinit �?重发 #CMDMODE 2 + !START */
    if (strncmp(cmd, "reinit", 6) == 0) {
        if (!usb_cdc_is_ready()) {
            debug_println("[REINIT] USB not connected");
            return true;
        }
        debug_println("[REINIT] Re-initializing arm (mode + enable)...");
        for (int attempt = 0; attempt < 8; attempt++) {
            int r1 = dummy_arm_set_mode(DUMMY_MODE_INTERRUPTABLE);
            int r2 = dummy_arm_enable();

            debug_print("[REINIT] Attempt ");
            debug_print_int(attempt);
            debug_print(": mode=");
            debug_print_int(r1);
            debug_print(" enable=");
            debug_print_int(r2);
            debug_println("");

            if (r1 == 0 && r2 == 0) {
                debug_println("[REINIT] OK �?arm in INTERRUPTABLE mode, enabled");
                motion_sync_from_usb();
                return true;
            }
            watchdog_refresh();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        debug_println("[REINIT] FAILED after 8 attempts");
        return true;
    }

    /* Degradation status command: degrade */
    if (strncmp(cmd, "degrade", 7) == 0) {
        degradation_status_t status;
        degradation_get_status(&status);
        debug_println("[DEGRADE] Status:");
        debug_print("  Level: ");
        debug_println(degradation_level_str(status.level));
        debug_print("  Last fault: ");
        debug_println(degradation_fault_str(status.last_fault));
        debug_print("  Fault joint: ");
        if (status.fault_joint < 6) {
            debug_print_int(status.fault_joint);
        } else {
            debug_print("N/A");
        }
        debug_println("");
        debug_print("  Speed scale: ");
        debug_print_int((int)(status.speed_scale * 100));
        debug_println("%");
        debug_print("  Disabled joints: 0x");
        char hex[3];
        hex[0] = "0123456789ABCDEF"[(status.disabled_joints >> 4) & 0x0F];
        hex[1] = "0123456789ABCDEF"[status.disabled_joints & 0x0F];
        hex[2] = '\0';
        debug_println(hex);
        debug_print("  Consecutive failures: ");
        debug_print_int(status.consecutive_failures);
        debug_println("");
        return true;
    }

    /* 自动扫描命令 */
    if (strncmp(cmd, "scan", 4) == 0) {
        int step = 50;
        int full_mode = 0;

        if (strstr(cmd, "fine") != NULL) {
            step = 30;
            debug_println("[SCAN] Fine mode (step=30mm)");
        }
        if (strstr(cmd, "full") != NULL) {
            full_mode = 1;
            debug_println("[SCAN] Full mode (front + back)");
        }
        if (!strstr(cmd, "fine") && !strstr(cmd, "full")) {
            debug_println("[SCAN] Normal mode (step=50mm, front only)");
        }

        debug_println("[SCAN] Testing reachable workspace...");
        if (full_mode) {
            debug_println("Range: X[-300,300] Y[-400,-100]&[100,400] Z[-200,200]");
        } else {
            debug_println("Range: X[-300,300] Y[100,400] Z[-200,200]");
        }
        debug_println("Format: x,y,z -> OK");
        debug_println("");

        int ok_count = 0;
        int fail_count = 0;

        for (int x = -300; x <= 300; x += step) {
            for (int y = 100; y <= 400; y += step) {
                for (int z = -200; z <= 200; z += step) {
                    int ret = motion_test_ik((float)x, (float)y, (float)z);
                    if (ret == 0) {
                        debug_print_int(x);
                        debug_print(",");
                        debug_print_int(y);
                        debug_print(",");
                        debug_print_int(z);
                        debug_println(" OK");
                        ok_count++;
                    } else {
                        fail_count++;
                    }
                }
            }
            watchdog_refresh();  /* scan 可持续数十秒，每行 x 喂一次狗 */

            if (full_mode) {
                for (int y = -400; y <= -100; y += step) {
                    for (int z = -200; z <= 200; z += step) {
                        int ret = motion_test_ik((float)x, (float)y, (float)z);
                        if (ret == 0) {
                            debug_print_int(x);
                            debug_print(",");
                            debug_print_int(y);
                            debug_print(",");
                            debug_print_int(z);
                            debug_println(" OK");
                            ok_count++;
                        } else {
                            fail_count++;
                        }
                    }
                }
                watchdog_refresh();  /* full_mode 负向 y 段也喂一次 */
            }
        }

        debug_println("");
        debug_print("[SCAN] Done. OK=");
        debug_print_int(ok_count);
        debug_print(" FAIL=");
        debug_print_int(fail_count);
        debug_print(" (total=");
        debug_print_int(ok_count + fail_count);
        debug_println(")");

        return true;
    }

    /* 查询笛卡尔位�? lpos */
    if (strncmp(cmd, "lpos", 4) == 0) {
        if (!usb_cdc_is_ready()) {
            debug_println("[USB] Not connected");
            return true;
        }
        dummy_cart_pos_t cp;
        if (dummy_arm_get_cart_pos(&cp) == 0) {
            char buf[128];
            snprintf(buf, sizeof(buf), "[LPOS] X=%.1f Y=%.1f Z=%.1f A=%.1f B=%.1f C=%.1f",
                     cp.x, cp.y, cp.z, cp.a, cp.b, cp.c);
            debug_println(buf);
        } else {
            debug_println("[LPOS] Query failed");
        }
        return true;
    }

    /* 画圆: circle [loops] �?YZ平面画圆，基于HOME位姿 */
    if (strncmp(cmd, "circle", 6) == 0) {
        if (!usb_cdc_is_ready()) {
            debug_println("[USB] Not connected");
            return true;
        }

        int loops = 1;
        if (cmd[6] == ' ') loops = atoi(cmd + 7);
        if (loops < 1) loops = 1;
        if (loops > 5) loops = 5;

        /* 1. 先回HOME */
        debug_println("[CIRCLE] Homing first...");
        dummy_arm_home();
        {
            static const float HOME_POSE[6] = {0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f};
            motion_set_current_joints(HOME_POSE);
        }
        wait_and_print_after();

        /* 2. 获取HOME笛卡尔位置作为圆�?*/
        dummy_cart_pos_t center;
        if (dummy_arm_get_cart_pos(&center) != 0) {
            center = (dummy_cart_pos_t){ .x=89.4f, .y=0.0f, .z=146.7f, .a=0, .b=0, .c=0 };
            debug_println("[CIRCLE] GETLPOS failed, using HOME fallback");
        }

        const float R = 20.0f;          /* 半径 20mm (避开底部奇异区域) */
        const int N = 20;               /* 20个点 */
        const float circle_speed = 40.0f;    /* 40mm/s, 慢速画圆更稳定 */
        const float arrive_thresh_sq = 25.0f; /* 5mm到位阈�?(平方) */
        const int poll_max = 30;  /* 最多轮�?0�?× 100ms = 3s */

        char info[80];
        snprintf(info, sizeof(info), "[CIRCLE] center=(%.1f,%.1f,%.1f) R=%.0f loops=%d Yoff=%.0f",
                 center.x, center.y, center.z, R, loops, R + 10.0f);
        debug_println(info);
        print_joint_state("BEFORE");

        /* 3. YZ平面画圆: x恒定, y=cy+R*sin(t), z=cz+R*cos(t) */
        for (int loop = 0; loop < loops; loop++) {
            for (int i = 0; i < N; i++) {
                float t = 2.0f * 3.14159f * (float)i / (float)N;

                dummy_cart_pos_t target;
                target.x = center.x;                    /* X恒定 */
                target.y = center.y + (R + 10.0f) + R * sinf(t);  /* Y振荡, +R+10避开y=0奇异 */
                target.z = center.z + R + R * cosf(t);  /* Z振荡, 上移R避开底部奇异 */
                target.a = center.a;
                target.b = center.b;
                target.c = center.c;

                int ret = dummy_arm_move_l(&target, circle_speed);
                if (ret != 0) {
                    debug_println("[CIRCLE] MoveL fail, skip");
                    continue;
                }

                /* 轮询等到�?(最�?s) */
                for (int w = 0; w < poll_max; w++) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    watchdog_refresh();
                    dummy_cart_pos_t cur;
                    if (dummy_arm_get_cart_pos(&cur) != 0) continue;
                    float dx = cur.x - target.x;
                    float dy = cur.y - target.y;
                    float dz = cur.z - target.z;
                    if (dx*dx + dy*dy + dz*dz < arrive_thresh_sq) break;
                }
            }
        }

        /* 4. 画完回HOME */
        debug_println("[CIRCLE] Done, homing...");
        vTaskDelay(pdMS_TO_TICKS(2000));         /* 等FIFO自然排空(40mm/s�?s足够) */
        dummy_arm_home();
        {
            static const float HOME_POSE[6] = {0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f};
            motion_set_current_joints(HOME_POSE);
        }
        wait_and_print_after();
        debug_println("[CIRCLE] Complete!");
        return true;
    }

    /* ========== 编码器零点校�?(断电后恢�? ========== */
    if (strcmp(cmd, "calibrate") == 0) {
        if (!usb_cdc_is_ready()) { debug_println("[CAL] Not connected"); return true; }
        debug_println("[CAL] === CALIBRATION MODE ===");
        debug_println("[CAL] Sending !CALIBRATION to firmware...");
        debug_println("[CAL] Firmware will: disable -> set zero -> move to REST -> reboot");
        char resp[128];
        int r = dummy_arm_raw_cmd("!CALIBRATION", resp, sizeof(resp), 10000);
        if (r >= 0) {
            debug_print("[CAL] Resp: "); debug_println(resp);
        } else {
            debug_println("[CAL] Timeout (motor reboot in progress)");
        }
        debug_println("[CAL] Waiting 5s for motor reboot...");
        for (int _cal_i = 0; _cal_i < 5; _cal_i++) {   /* 分段延迟，每秒喂一次狗 */
            watchdog_refresh();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        r = dummy_arm_raw_cmd("!START", resp, sizeof(resp), 2000);
        if (r >= 0) {
            debug_println("[CAL] Motors re-enabled.");
        } else {
            debug_println("[CAL] START timeout, try 'usb raw !START' manually");
        }
        static const float HOME_POSE[6] = {0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f};
        motion_set_current_joints(HOME_POSE);
        debug_println("[CAL] Done. Run 'usb home' to verify.");
        return true;
    }

    /* ========== USB Dummy Arm 测试命令 ========== */
    if (strncmp(cmd, "usb", 3) == 0) {
        /* 去尾部空�? 复制命令并trim */
        char ucmd[64];
        strncpy(ucmd, cmd, sizeof(ucmd) - 1);
        ucmd[sizeof(ucmd) - 1] = '\0';
        { /* trim trailing spaces */
            int end = (int)strlen(ucmd) - 1;
            while (end >= 0 && ucmd[end] == ' ') ucmd[end--] = '\0';
        }

        if (strcmp(ucmd, "usb status") == 0 || strcmp(ucmd, "usb") == 0) {
            usb_cdc_state_t st = usb_cdc_get_state();
            const char *names[] = {"DISCONNECTED", "ATTACHED", "CONFIGURED", "READY"};
            debug_print("[USB] State: ");
            debug_print(names[st]);
            debug_print("  addr=");
            debug_print_int(usb_cdc_get_device_address());
            debug_println("");
        } else if (strcmp(ucmd, "usb enable") == 0) {
            if (!usb_cdc_is_ready()) { debug_println("[USB] Not connected"); return true; }
            int r = dummy_arm_enable();
            debug_print("[USB] Enable: ");
            debug_println(r == 0 ? "OK" : "FAIL");
        } else if (strcmp(ucmd, "usb disable") == 0) {
            if (!usb_cdc_is_ready()) { debug_println("[USB] Not connected"); return true; }
            dummy_arm_disable();
            debug_println("[USB] Disabled");
        } else if (strcmp(ucmd, "usb home") == 0) {
            if (!usb_cdc_is_ready()) { debug_println("[USB] Not connected"); return true; }
            print_joint_state("BEFORE");
            debug_println("[USB] Homing...");
            dummy_arm_home();
            /* HOME位置已知，直接设置避免查询超�?*/
            {
                static const float HOME_POSE[6] = {0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f};
                motion_set_current_joints(HOME_POSE);
            }
            debug_println("[USB] Home done");
            /* 位置已由 motion_set_current_joints 同步，无需再查�?*/
        } else if (strcmp(ucmd, "usb rest") == 0) {
            if (!usb_cdc_is_ready()) { debug_println("[USB] Not connected"); return true; }
            print_joint_state("BEFORE");
            dummy_arm_rest();
            /* REST位置已知，直接设�?*/
            {
                static const float REST_POSE[6] = {0.0f, -73.0f, 180.0f, 0.0f, 0.0f, 0.0f};
                motion_set_current_joints(REST_POSE);
            }
            debug_println("[USB] Rest done");
            /* 位置已由 motion_set_current_joints 同步，无需再查�?*/
        } else if (strcmp(ucmd, "usb stop") == 0) {
            if (!usb_cdc_is_ready()) { debug_println("[USB] Not connected"); return true; }
            print_joint_state("BEFORE");
            dummy_arm_stop();
            /* 急停后位置未知，从USB查询实际位置 */
            motion_sync_from_usb();
            debug_println("[USB] Stopped");
            wait_and_print_after();
        } else if (strcmp(ucmd, "usb pos") == 0) {
            if (!usb_cdc_is_ready()) { debug_println("[USB] Not connected"); return true; }
            dummy_joint_pos_t jp;
            if (dummy_arm_get_joint_pos(&jp) == 0) {
                char buf[128];
                snprintf(buf, sizeof(buf), "[USB] J: %.1f %.1f %.1f %.1f %.1f %.1f",
                         jp.j[0], jp.j[1], jp.j[2], jp.j[3], jp.j[4], jp.j[5]);
                debug_println(buf);
            } else {
                debug_println("[USB] Query failed");
            }
        } else if (strncmp(ucmd, "usb move ", 9) == 0) {
            if (!usb_cdc_is_ready()) { debug_println("[USB] Not connected"); return true; }
            float j[6] = {0};
            int n = sscanf(ucmd + 9, "%f %f %f %f %f %f", &j[0], &j[1], &j[2], &j[3], &j[4], &j[5]);
            if (n >= 1) {
                print_joint_state("BEFORE");
                debug_println("[USB] Moving...");
                dummy_arm_move_joints(j[0], j[1], j[2], j[3], j[4], j[5], 30.0f);
                /* 目标已知，直接同步到motion_controller */
                motion_set_current_joints(j);
                debug_println("[USB] Move sent");
                wait_and_print_after();
            } else {
                debug_println("[USB] Usage: usb move j1 j2 j3 j4 j5 j6");
            }
        } else if (strncmp(ucmd, "usb raw ", 8) == 0) {
            if (!usb_cdc_is_ready()) { debug_println("[USB] Not connected"); return true; }
            char resp[128];
            int r = dummy_arm_raw_cmd(ucmd + 8, resp, sizeof(resp), 2000);
            if (r >= 0) {
                debug_print("[USB] Resp: ");
                debug_println(resp);
            } else {
                debug_println("[USB] No response");
            }
        } else {
            debug_println("[USB] Commands: usb, usb enable, usb disable, usb home, usb rest, usb stop, usb pos, usb move j1..j6, usb raw <cmd>");
        }
        return true;
    }

    /* ========== I2C 诊断: i2c scan/gpio/reg ========== */
    if (strncmp(cmd, "i2c", 3) == 0) {
        const char *sub = cmd + 3;
        while (*sub == ' ') sub++;
        if (strncmp(sub, "scan", 4) == 0) {
            drv_iic0_scan();
        } else if (strncmp(sub, "gpio", 4) == 0) {
            drv_iic0_gpio_test();
        } else if (strncmp(sub, "reg", 3) == 0) {
            drv_iic0_dump_regs();
        } else if (strncmp(sub, "init", 4) == 0) {
            debug_println("[I2C] Manual init...");
            drv_iic0_init();
            debug_println("[I2C] Init done. Run 'i2c reg' to verify.");
        } else {
            debug_println("[I2C] Commands: i2c init, i2c scan, i2c gpio, i2c reg");
        }
        return true;
    }

    /* ========== 音频命令: audio init/tone/rec/vol/diag ========== */
    if (strncmp(cmd, "audio", 5) == 0) {
        const char *sub = cmd + 5;
        while (*sub == ' ') sub++;

        if (strncmp(sub, "init", 4) == 0) {
            int r = audio_init();
            if (r == 0) {
                debug_println("[AUDIO] Init OK");
            } else {
                debug_println("[AUDIO] Init FAILED");
            }
        } else if (strncmp(sub, "tone", 4) == 0) {
            if (!audio_is_ready()) {
                debug_println("[AUDIO] Not initialized. Run 'audio init' first.");
                return true;
            }
            uint32_t ms = 1000;
            if (sub[4] == ' ') ms = (uint32_t)atoi(sub + 5);
            if (ms < 100) ms = 100;
            if (ms > 5000) ms = 5000;
            debug_print("[AUDIO] Playing 1kHz tone for ");
            debug_print_int((int)ms);
            debug_println("ms...");
            audio_play_tone(ms);
            debug_println("[AUDIO] Tone done");
        } else if (strncmp(sub, "rec", 3) == 0) {
            if (!audio_is_ready()) {
                debug_println("[AUDIO] Not initialized. Run 'audio init' first.");
                return true;
            }
            uint32_t ms = 1000;
            if (sub[3] == ' ') ms = (uint32_t)atoi(sub + 4);
            if (ms < 100) ms = 100;
            if (ms > 5000) ms = 5000;
            uint32_t samples = (48000 * ms) / 1000;
            /* 使用静态缓冲区, 最多录 ~1s (48000 × 2ch × 2bytes = 192KB, 太大)
             * 限制为缓冲区大小 */
            static int16_t rec_buf[960 * 2];  /* 20ms = 960 samples × 2ch */
            if (samples > 960) samples = 960;
            debug_print("[AUDIO] Recording ");
            debug_print_int((int)samples);
            debug_println(" samples...");
            int got = audio_record(rec_buf, samples);
            debug_print("[AUDIO] Recorded ");
            debug_print_int(got);
            debug_println(" samples");
            /* 回放录音 */
            debug_println("[AUDIO] Playing back...");
            audio_play(rec_buf, (uint32_t)got);
            debug_println("[AUDIO] Playback done");
        } else if (strncmp(sub, "vol", 3) == 0) {
            if (sub[3] == ' ') {
                int v = atoi(sub + 4);
                if (v < 0) v = 0;
                if (v > 100) v = 100;
                audio_set_volume((uint8_t)v);
                debug_print("[AUDIO] Volume set to ");
                debug_print_int(v);
                debug_println("");
            } else {
                debug_println("[AUDIO] Usage: audio vol <0-100>");
            }
        } else if (strncmp(sub, "diag", 4) == 0) {
            audio_diag();
        } else {
            debug_println("[AUDIO] Commands: audio init, audio tone [ms], audio rec [ms], audio vol <0-100>, audio diag");
        }
        return true;
    }

    /* 旧命令已移除: t dx dy dz -> x dx dy dz */
    if (cmd[0] == 't' && cmd[1] == ' ') {
        debug_println("[CMD] 't' removed. Use: x <dx> <dy> <dz>");
        return true;
    }

    /* ========== 底层透传命令 (!/#/>/@): 默认禁用, 仅调试模式开�?========== */
    if (cmd[0] == '!' || cmd[0] == '#' || cmd[0] == '>' || cmd[0] == '@') {
        if (!g_arm_passthrough_enabled) {
            debug_println("[CMD] Passthrough disabled. Run 'debug arm on' to enable.");
            return true;
        }
        if (!usb_cdc_is_ready()) {
            debug_println("[USB] Not connected");
            return true;
        }

        if (cmd[0] == '!') {
            print_joint_state("BEFORE");
            char resp[128];
            int r = dummy_arm_raw_cmd(cmd, resp, sizeof(resp), 2000);
            if (r >= 0) {
                debug_print("[USB] ");
                debug_println(resp);
            } else {
                debug_println("[USB] No response (timeout)");
            }
            /* !命令可能改变臂位�?HOME/RESET/START�?，同步motion_controller */
            motion_sync_from_usb();
            wait_and_print_after();
            return true;
        }

        if (cmd[0] == '#') {
            char resp[128];
            int r = dummy_arm_raw_cmd(cmd, resp, sizeof(resp), 2000);
            if (r >= 0) {
                debug_print("[USB] ");
                debug_println(resp);
            } else {
                debug_println("[USB] No response (timeout)");
            }
            return true;
        }

        /* > �?@ 运动命令直接透传�?Dummy ARM */
        print_joint_state("BEFORE");
        char resp[128];
        int r = dummy_arm_raw_cmd(cmd, resp, sizeof(resp), 2000);
        if (r >= 0) {
            debug_print("[USB] ");
            debug_println(resp);
        } else {
            debug_println("[USB] No response (timeout)");
        }
        /* 直接运动命令绕过了motion_controller，同步实际位�?*/
        motion_sync_from_usb();
        wait_and_print_after();
        return true;
    }

    return false;  /* 不是本地测试命令 */
}

/*
 * 主线程入�?
 */
void new_thread0_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 初始化公共模�?*/
    app_common_init();

    /* 初始化调试串�?*/
    debug_uart_init();
    debug_println("=== RA6M5 AI Pharmacy Robot (USB Mode) ===");

    /* 检查是否因看门狗复�?*/
    if (watchdog_was_reset()) {
        debug_println("[WDT] System recovered from watchdog reset!");
    }

    /* 初始化看门狗 */
    watchdog_init(WATCHDOG_ENABLED);
    if (WATCHDOG_ENABLED) {
        debug_println("[WDT] Watchdog enabled (~2.7s timeout)");
    }

    debug_println("Type command and press Enter to send to LLM");
    debug_println("");

    /* 初始化USB Host CDC (连接Dummy机械�? */
    debug_println("[USB] Initializing USB Host CDC...");
    usb_cdc_init();
    debug_println("[USB] USB Host initialized (non-blocking)");

    /* 初始化运动控制器 (USB后端) */
    motion_init();

    visual_servo_init();
    maixcam_uart_init(&g_mc_callbacks);
    R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);

    /* 初始化降级策略模�?*/
    degradation_init();
    debug_println("[DEGRADE] Degradation module initialized");

    /* 初始化实时性监�?(DWT cycle counter) */
    rtmon_init();
    debug_println("[RTMON] Real-time monitor initialized");

    /* 初始化任务监�?*/
    taskmon_init();
    taskmon_register("main_thread");
    debug_println("[TASKMON] Task monitor initialized");

    /* 初始化格子示教系�?*/
    slot_teach_init();
    debug_println("[SLOT] Slot teach system initialized");

    /* 初始化真空泵 GPIO (P600) */
    vacuum_pump_init();
    debug_println("[PUMP] Vacuum pump GPIO initialized");

    /* 启动定时�?(5ms周期, 用于看门狗喂狗和心跳LED) */
    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    R_GPT_Start(&g_timer0_ctrl);
    debug_println("[TIMER] Timer started (5ms, 200Hz)");

    /* 打开UART6 (W800通信) */
    fsp_err_t err = R_SCI_UART_Open(&g_uart6_ctrl, &g_uart6_cfg);
    if (err != FSP_SUCCESS)
    {
        g_system_state = STATE_ERROR;
        g_wifi_error_code = -10;
        debug_println("[ERR] UART6 open failed");
    }
    else
    {
        /* 初始化W800驱动 */
        w800_init(NULL); /* LLM callback disabled */

        /* 配置WiFi */
        w800_wifi_config_t wifi_cfg = {
            .ssid        = "yamaRedmi K70",
            .password    = "12345678",
            .remote_ip   = "192.168.7.86",
            .remote_port = 8888,
            .local_port  = 8889
        };

        /* WiFi连接重试循环 */
        int wifi_retry;
        int wifi_connected = 0;
        for (wifi_retry = 0; wifi_retry < 3 && !wifi_connected; wifi_retry++) {
            if (wifi_retry > 0) {
                debug_print("[WIFI] Retry ");
                debug_print_int(wifi_retry);
                debug_println("/3...");
                watchdog_refresh();
                vTaskDelay(pdMS_TO_TICKS(1000));
                watchdog_refresh();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            /* 硬件复位W800模块 (通过P508引脚) */
            debug_println("[WIFI] Hardware resetting W800...");
            w800_hardware_reset();

            /* 等待W800初始化完�?(拆分延时防止看门狗复�? */
            debug_println("[WIFI] Waiting for W800 to initialize (3s)...");
            for (int d = 0; d < 3; d++) {
                watchdog_refresh();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            /* 等待W800就绪 */
            debug_println("[WIFI] Checking W800 status...");
            int w800_ready = 0;
            for (int i = 0; i < 5; i++) {
                w800_status_t status = w800_get_status();
                if (status == W800_STATUS_NO_WIFI || status == W800_STATUS_READY) {
                    debug_println("[WIFI] W800 ready!");
                    w800_ready = 1;
                    break;
                }
                debug_println("[WIFI] W800 not ready, retrying...");
                watchdog_refresh();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            if (!w800_ready) {
                debug_println("[WIFI] W800 not responding, will retry...");
                continue;
            }

            /* 尝试连接WiFi */
            debug_println("[WIFI] Calling w800_connect_wifi...");
            int ret = w800_connect_wifi(&wifi_cfg);
            debug_print("[WIFI] w800_connect_wifi returned: ");
            debug_print_int(ret);
            debug_println("");

            if (ret == 0) {
                wifi_connected = 1;
                debug_println("[WIFI] Connected!");
            } else {
                debug_print("[WIFI] Failed, error code: ");
                debug_print_int(ret);
                debug_println("");
            }
        }
    }   /* end else (UART6 open succeeded) */


    {   /* 无条件打印命令列�?*/
        debug_println("");
        debug_println("Commands:");
            debug_println("  === Motion ===");
            debug_println("  home          - Home position (0,0,90,0,0,0)");
            debug_println("  rest          - Rest position (0,-73,180,0,0,0)");
            debug_println("  x dx dy dz    - Cartesian delta (relative)");
            debug_println("  m j0..j5      - Multi-joint move");
            debug_println("  j id angle    - Single joint (absolute)");
            debug_println("  jr id delta   - Single joint (relative)");
            debug_println("  scan          - Auto scan workspace");
            debug_println("  lpos          - Query cartesian position");
            debug_println("  circle [n]    - Draw circle in YZ plane (n loops)");
            debug_println("  stop          - Emergency stop");
            debug_println("  clear         - Clear fault state");
            debug_println("  reinit        - Re-init arm (mode+enable)");
            debug_println("");
            debug_println("  calibrate     - Recalibrate encoder zero (after power cycle)");
            debug_println("");
            debug_println("  === USB Debug ===");
            debug_println("  debug arm on/off - Enable/disable !/#/>/@ passthrough");
            debug_println("  usb           - USB connection status");
            debug_println("  usb pos       - Query joint positions");
            debug_println("  usb move j1..j6 - Direct USB move");
            debug_println("  usb raw <cmd> - Send raw USB command");
            if (g_arm_passthrough_enabled) {
                debug_println("  !/#/>/@       - Raw passthrough enabled");
            }
            debug_println("");
            debug_println("  === Gripper ===");
            debug_println("  grip open/close/vacuum on/off");
            debug_println("");
            debug_println("  === Audio ===");
            debug_println("  audio init    - Init audio (IIC0+WM8960+SSI0)");
            debug_println("  audio tone [ms] - Play 1kHz test tone");
            debug_println("  audio rec [ms]  - Record and playback");
            debug_println("  audio vol <0-100> - Set volume");
            debug_println("  audio diag    - Dump SSI/WM8960 diagnostic state");
            debug_println("");
            debug_println("  === Diagnostics ===");
            debug_println("  rtmon         - Show real-time stats");
            debug_println("  taskmon       - Show task CPU usage");
            debug_println("  uartstat      - Show UART4 RX/queue error stats");
            debug_println("  degrade       - Show degradation status");
            debug_println("  i2c init      - Manual IIC2 init");
            debug_println("  i2c scan      - Scan I2C bus (0x03-0x77)");
            debug_println("  i2c gpio      - Read SDA/SCL pin levels");
            debug_println("  i2c reg       - Dump IIC2 registers");
            debug_println("");
            /* debug_println("  ai: <text>    - Send to LLM"); */  /* LLM disabled */
            debug_println("");
            debug_print("> ");
    }   /* 命令列表打印块结�?*/


    char cmd_buf[256];
    uint32_t heartbeat_counter = 0;
    uint32_t taskmon_counter = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 处理来自 W800 TCP 透传的 JSON 命令 */
        w800_process_pending_cmd();
        w800_poll_notifications();


        /* 检查用户输�?*/
        if (debug_has_line()) {
            int len = debug_read_line(cmd_buf, sizeof(cmd_buf));
            if (len > 0) {
                /* 先检查是否是本地测试命令 */
                if (!handle_test_command(cmd_buf)) {
                    /* LLM disabled: ai: / ai commands removed */
                    debug_print("[CMD] Unknown: ");
                    debug_println(cmd_buf);
                }
            }
            debug_print("> ");
        }

        /* LLM响应检查已禁用 */
#if 0
        if (g_llm_resp_ready) {
            process_llm_response(g_llm_response);
            g_llm_resp_ready = false;
            g_llm_resp_idx = 0;
            debug_print("> ");
        }
#endif

        /* 5ms定时器触�?- 看门狗喂�?+ 心跳LED */
        if (g_motion_tick) {
            g_motion_tick = false;

            /* 动作序列完成检�?(LLM disabled) */
#if 0
            if (g_executing_sequence) {
                motion_state_t mstate = motion_get_state();
                if (mstate == MOTION_DONE || mstate == MOTION_ERROR) {
                    execute_next_action();
                }
            }
#endif

            /* 心跳LED */
            heartbeat_counter++;
            if (heartbeat_counter >= 100) {  /* 5ms * 100 = 500ms */
                heartbeat_counter = 0;

                /* 喂狗 (�?00ms刷新一次，超时�?.7�? */
                watchdog_refresh();

                /* 任务监控更新 (每秒一�? */
                taskmon_counter++;
                if (taskmon_counter >= 2) {  /* 500ms * 2 = 1s */
                    taskmon_counter = 0;
                    taskmon_update("main_thread", 0);
                }

            }
            /* LED 闪烁已移至 motion_timer_callback ISR，此处无需操作 */
        }


        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));

    }
}

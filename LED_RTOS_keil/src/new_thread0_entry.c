#include "new_thread0.h"
#include "hal_data.h"
#include "common_data.h"
#include "app_common.h"
#include "w800_driver.h"
#include "debug_uart.h"
#include "llm_action.h"
#include "motion_controller.h"
#include "motor_ctrl_step.h"
#include "gripper.h"
#include "watchdog.h"
#include "zdt_test.h"
#include "rtmon.h"
#include "taskmon.h"
#include "degradation.h"
#include "motor_test.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ========== 看门狗配置 ========== */
#define WATCHDOG_ENABLED    1   /* 1=启用看门狗, 0=禁用(调试时) */

#define LED_PIN BSP_IO_PORT_04_PIN_00

/* ========== LLM响应缓冲区 ========== */
#define LLM_RESP_BUF_SIZE 1024  /* 原512，增大以支持云端LLM响应 */
static char g_llm_response[LLM_RESP_BUF_SIZE];
static volatile uint16_t g_llm_resp_idx = 0;
static volatile bool g_llm_resp_ready = false;

/* ========== 动作序列 ========== */
static llm_action_sequence_t g_action_seq;
static bool g_executing_sequence = false;  /* 是否正在执行动作序列 */
static bool g_collision_detected = false;  /* 碰撞检测标志 */

/* ========== 碰撞检测配置 ========== */
#define COLLISION_CONFIRM_COUNT  2   /* 连续检测次数阈值 (2次 x 10ms = 20ms) */
static uint8_t g_stall_count[6] = {0};  /* 每个电机的连续堵转计数 */

/* ========== 硬件定时器运动控制 ========== */
static volatile bool g_motion_tick = false;  /* 5ms定时器触发标志 */

/**
 * @brief GPT定时器回调 - 5ms周期触发运动控制
 * 在中断中设置标志，主循环中处理
 */
void motion_timer_callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event) {
        rtmon_tick(5000);  /* Record timestamp for jitter monitoring, 5ms=5000us */
        g_motion_tick = true;
    }
}

/* W800接收数据回调 - 存储从PC返回的LLM响应 */
static void w800_data_received(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        char ch = (char)data[i];

        /* 调试：打印收到的每个字符 */
        if (g_llm_resp_idx == 0) {
            /* 首次收到数据，打印提示 */
        }

        if (g_llm_resp_idx < LLM_RESP_BUF_SIZE - 1) {
            g_llm_response[g_llm_resp_idx++] = ch;
            g_llm_response[g_llm_resp_idx] = '\0';
        }
    }
    /* 简单判断：收到 ']' 或 '}' 结尾可能是JSON完整 */
    if (g_llm_resp_idx > 0) {
        char last = g_llm_response[g_llm_resp_idx - 1];
        if (last == ']' || last == '}') {
            g_llm_resp_ready = true;
        }
    }
}

/* 执行单个动作 (返回true表示需要等待运动完成) */
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
            /* 调用运动控制器 */
            if (motion_move_to_xyz(action->params.move.x,
                                   action->params.move.y,
                                   action->params.move.z) == 0) {
                return true;  /* 需要等待运动完成 */
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
            /* TODO: TTS播报 */
            break;

        case ACTION_SCAN_QR:
            debug_println("  -> Scanning QR...");
            /* TODO: 触发MaixCam2扫描 */
            break;

        default:
            debug_println("  -> Unknown action");
            break;
    }
    return false;  /* 不需要等待 */
}

/* 执行下一个动作 */
static void execute_next_action(void)
{
    while (g_action_seq.current < g_action_seq.count) {
        bool need_wait = execute_action(&g_action_seq.actions[g_action_seq.current]);
        g_action_seq.current++;

        if (need_wait) {
            /* 需要等待运动完成 */
            return;
        }
        /* 不需要等待，继续下一个动作 */
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* 所有动作完成 */
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

    /* 开始执行动作序列 */
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

        /* 解析并执行动作 */
        process_llm_response(response);
        return 0;
    } else {
        debug_print("[LLM] Failed: ");
        debug_print_int(ret);
        debug_println("");
        return ret;
    }
}

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

static int ctrl_step_get_stall_status(uint8_t joint_index)
{
    if (joint_index >= MOTOR_CTRL_STEP_NUM_JOINTS) {
        return -1;
    }

    motor_ctrl_step_feedback_t feedback = {0};
    if (!motor_ctrl_step_get_feedback(joint_index, &feedback) || !feedback.online) {
        return 0;
    }

    return ((feedback.state_byte & 0x10U) != 0U) ? 1 : 0;
}

static bool ctrl_step_ping_joint(uint8_t joint_index, uint32_t timeout_ms, motor_ctrl_step_feedback_t *out_feedback)
{
    if (joint_index >= MOTOR_CTRL_STEP_NUM_JOINTS) {
        return false;
    }

    TickType_t request_tick = xTaskGetTickCount();
    if (motor_ctrl_step_query_position(joint_index) != 0) {
        return false;
    }

    TickType_t start_tick = request_tick;
    TickType_t timeout_tick = pdMS_TO_TICKS(timeout_ms);
    if (timeout_tick == 0U) {
        timeout_tick = 1U;
    }

    while ((xTaskGetTickCount() - start_tick) <= timeout_tick) {
        (void) motor_ctrl_step_poll_rx();

        motor_ctrl_step_feedback_t feedback = {0};
        if (motor_ctrl_step_get_feedback(joint_index, &feedback) && feedback.online) {
            if (feedback.last_rx_tick >= request_tick) {
                if (out_feedback != NULL) {
                    *out_feedback = feedback;
                }
                return true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return false;
}

/**
 * @brief 本地测试命令处理
 * 格式: "t x y z"     - 测试IK坐标
 * 格式: "x x y z"     - 笛卡尔运动 (dummy-auk标准命令)
 * 格式: "m j0..j5"    - 六轴同步关节运动
 * 格式: "j id angle"  - 单轴运动 (0-5, 角度)
 * 格式: "home id"     - 电机归零 (当前编码器位置清零)
 * 格式: "clear id"    - 清除堵转保护
 * 格式: "enable id"   - 使能电机
 * 格式: "query id"    - 查询电机位置反馈
 * 格式: "scan"        - 自动扫描可达空间
 * 格式: "acc val"     - 设置加速度 (500-5000)
 * 格式: "pid id kp kv ki" - 设置PID
 * 格式: "fast"        - 快速模式
 * 格式: "smooth"      - 平稳模式
 * 格式: "status"      - 读取电机状态
 * 格式: "grip open/close" - 夹爪控制
 * 格式: "stop"        - 急停
 * 例如: "t 0 0 0" 或 "j 0 90"
 * @return true 如果是测试命令并已处理
 */
static bool handle_test_command(const char *cmd)
{
    /* 网络连通性测试命令: nettest */
    if (strncmp(cmd, "nettest", 7) == 0) {
        debug_println("[CMD] Running network connectivity test...");
        w800_test_network();
        return true;
    }

    /* 实时性监测命令: rtmon [reset] */
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
            taskmon_update("main_thread", 0);  /* 先更新统计 */
            char taskmon_buf[512];
            taskmon_print_report(taskmon_buf, sizeof(taskmon_buf));
            debug_println(taskmon_buf);
        }
        return true;
    }

    /* HTTP LLM测试命令: http <query> */
    if (strncmp(cmd, "http ", 5) == 0) {
        const char *query = cmd + 5;
        debug_print("[HTTP] Calling cloud LLM: ");
        debug_println(query);
        debug_println("[HTTP] (Render.com may take 30+ seconds on cold start...)");

        char response[1024];  /* 增大缓冲区 */
        int ret = w800_call_llm_with_retry(query, response, sizeof(response), 3);

        if (ret > 0) {
            debug_print("[HTTP] Response (");
            debug_print_int(ret);
            debug_println(" bytes):");
            debug_println(response);

            /* 解析并执行动作 */
            process_llm_response(response);
        } else {
            debug_print("[HTTP] Failed after 3 retries: ");
            debug_print_int(ret);
            debug_println("");
        }
        return true;
    }

    /* dummy-auk标准归零命令: home <joint_id> */
    if (strncmp(cmd, "home", 4) == 0) {
        int joint = -1;
        if (sscanf(cmd, "home %d", &joint) != 1) {
            debug_println("[HOME] Usage: home <id> (id=0..5)");
            return true;
        }
        if (joint < 0 || joint > 5) {
            debug_println("[HOME] Error: joint should be 0-5");
            return true;
        }

        debug_print("[HOME] Reset encoder zero, J");
        debug_print_int(joint);
        debug_println("...");

        int ret = motor_ctrl_step_home((uint8_t) joint);
        if (ret == 0) {
            debug_println("[HOME] Done.");
        } else {
            debug_print("[HOME] Failed, ret=");
            debug_print_int(ret);
            debug_println("");
        }
        return true;
    }

    /* 急停命令 */
    if (strncmp(cmd, "stop", 4) == 0) {
        debug_println("[STOP] Emergency stop!");
        (void) motor_ctrl_step_set_enable(MOTOR_CTRL_STEP_ALL_JOINTS, false);
        motion_stop();
        return true;
    }

    /* 快速模式 */
    if (strncmp(cmd, "fast", 4) == 0) {
        debug_println("[MODE] Switching to FAST mode...");
        debug_println("  Acc=3000, High PID gains");
        /* CtrlStep协议不支持运行时FAST模式参数切换（加速度/PID配置已删除）。 */
        debug_println("[MODE] CtrlStep does not support FAST profile config, skipped.");
        debug_println("[MODE] Done!");
        return true;
    }

    /* 平稳模式 */
    if (strncmp(cmd, "smooth", 6) == 0) {
        debug_println("[MODE] Switching to SMOOTH mode...");
        debug_println("  Acc=1500, Low PID gains");
        /* CtrlStep协议不支持运行时SMOOTH模式参数切换（加速度/PID配置已删除）。 */
        debug_println("[MODE] CtrlStep does not support SMOOTH profile config, skipped.");
        debug_println("[MODE] Done!");
        return true;
    }

    /* 设置加速度: acc <value> */
    if (strncmp(cmd, "acc ", 4) == 0) {
        int acc = atoi(cmd + 4);
        if (acc < 100 || acc > 10000) {
            debug_println("[ACC] Error: value should be 100-10000");
            return true;
        }
        debug_print("[ACC] Setting acceleration to ");
        debug_print_int(acc);
        debug_println("");
        /* CtrlStep协议无set_acceleration接口，该配置项已删除。 */
        debug_println("[ACC] CtrlStep does not support acceleration config, skipped.");
        debug_println("[ACC] Done!");
        return true;
    }

    /* 设置PID: pid <joint> <kp> <kv> <ki> */
    if (strncmp(cmd, "pid ", 4) == 0) {
        int joint = 0, kp = 0, kv = 0, ki = 0;
        const char *p = cmd + 4;
        joint = atoi(p);
        while (*p && *p != ' ') p++; while (*p == ' ') p++;
        kp = atoi(p);
        while (*p && *p != ' ') p++; while (*p == ' ') p++;
        kv = atoi(p);
        while (*p && *p != ' ') p++; while (*p == ' ') p++;
        ki = atoi(p);

        if (joint < 0 || joint > 5) {
            debug_println("[PID] Error: joint should be 0-5");
            return true;
        }

        debug_print("[PID] Joint ");
        debug_print_int(joint);
        debug_print(": Kp=");
        debug_print_int(kp);
        debug_print(" Kv=");
        debug_print_int(kv);
        debug_print(" Ki=");
        debug_print_int(ki);
        debug_println("");

        /* CtrlStep协议无set_pid接口，该配置项已删除。 */
        debug_println("[PID] CtrlStep does not support PID config, skipped.");
        debug_println("[PID] Done!");
        return true;
    }

    /* 单轴运动: j <joint> <angle> (统一走motion_controller) */
    if (cmd[0] == 'j' && cmd[1] == ' ') {
        int joint = -1;
        float angle = 0.0f;
        if (sscanf(cmd, "j %d %f", &joint, &angle) != 2) {
            debug_println("[JOINT] Usage: j <id> <angle>");
            return true;
        }
        if (joint < 0 || joint > 5) {
            debug_println("[JOINT] Error: joint should be 0-5");
            return true;
        }

        float target_joints[6];
        motion_get_current_joints(target_joints);
        target_joints[joint] = angle;

        debug_print("[JOINT] J");
        debug_print_int(joint);
        debug_print(" -> ");
        debug_print_int((int) angle);
        debug_println(" deg");

        int ret = motion_move_to_joints(target_joints);
        log_motion_command_result("JOINT", ret);
        return true;
    }

    /* 六轴同步运动: m <j0> <j1> <j2> <j3> <j4> <j5> */
    if (cmd[0] == 'm' && cmd[1] == ' ') {
        float target_joints[6];
        if (sscanf(cmd, "m %f %f %f %f %f %f",
                   &target_joints[0], &target_joints[1], &target_joints[2],
                   &target_joints[3], &target_joints[4], &target_joints[5]) != 6) {
            debug_println("[MOTION] Usage: m <j0> <j1> <j2> <j3> <j4> <j5>");
            return true;
        }

        debug_println("[MOTION] Multi-joint move...");
        int ret = motion_move_to_joints(target_joints);
        log_motion_command_result("MOTION", ret);
        return true;
    }

    /* 笛卡尔运动: x <x> <y> <z> */
    if (cmd[0] == 'x' && cmd[1] == ' ') {
        float x = 0.0f, y = 0.0f, z = 0.0f;
        if (sscanf(cmd, "x %f %f %f", &x, &y, &z) != 3) {
            debug_println("[MOTION] Usage: x <x> <y> <z>");
            return true;
        }

        debug_print("[MOTION] XYZ -> ");
        debug_print_int((int) x);
        debug_print(" ");
        debug_print_int((int) y);
        debug_print(" ");
        debug_print_int((int) z);
        debug_println("");

        int ret = motion_move_to_xyz(x, y, z);
        log_motion_command_result("MOTION", ret);
        return true;
    }

    /* 调试命令: enable <joint_id> */
    if (strncmp(cmd, "enable", 6) == 0) {
        int joint = -1;
        if (sscanf(cmd, "enable %d", &joint) != 1) {
            debug_println("[ENABLE] Usage: enable <id> (id=0..5)");
            return true;
        }
        if (joint < 0 || joint > 5) {
            debug_println("[ENABLE] Error: joint should be 0-5");
            return true;
        }

        int ret = motor_ctrl_step_set_enable((uint8_t) joint, true);
        if (ret == 0) {
            debug_println("[ENABLE] Done.");
        } else {
            debug_print("[ENABLE] Failed, ret=");
            debug_print_int(ret);
            debug_println("");
        }
        return true;
    }

    /* 调试命令: query <joint_id> */
    if (strncmp(cmd, "query", 5) == 0) {
        int joint = -1;
        if (sscanf(cmd, "query %d", &joint) != 1) {
            debug_println("[QUERY] Usage: query <id> (id=0..5)");
            return true;
        }
        if (joint < 0 || joint > 5) {
            debug_println("[QUERY] Error: joint should be 0-5");
            return true;
        }

        int ret = motor_ctrl_step_query_position((uint8_t) joint);
        if (ret != 0) {
            debug_print("[QUERY] Request failed, ret=");
            debug_print_int(ret);
            debug_println("");
            return true;
        }

        motor_ctrl_step_feedback_t feedback = {0};
        if (motor_ctrl_step_get_feedback((uint8_t) joint, &feedback)) {
            debug_print("[QUERY] J");
            debug_print_int(joint);
            debug_print(" angle=");
            debug_print_int((int) feedback.feedback_angle_deg);
            debug_print(" state=0x");
            debug_print_hex(feedback.state_byte);
            debug_print(" online=");
            debug_print_int(feedback.online ? 1 : 0);
            debug_println("");
        } else {
            debug_println("[QUERY] Failed to read cached feedback.");
        }
        return true;
    }

    /* 读取状态: status */
    if (strncmp(cmd, "status", 6) == 0) {
        debug_println("[STATUS] Motor states:");
        for (int i = 0; i < 6; i++) {
            (void) motor_ctrl_step_query_position((uint8_t) i);
            (void) motor_ctrl_step_poll_rx();

            motor_ctrl_step_feedback_t feedback = {0};
            bool has_feedback = motor_ctrl_step_get_feedback((uint8_t) i, &feedback);
            debug_print("  J");
            debug_print_int(i);
            debug_print(": angle=");
            debug_print_int(has_feedback ? (int) feedback.feedback_angle_deg : 0);
            debug_print(" target=");
            debug_print_int(has_feedback ? (int) feedback.target_angle_deg : 0);
            debug_print(" reached=");
            debug_print_int((has_feedback && feedback.finished) ? 1 : 0);
            debug_print(" stall=");
            debug_print_int(ctrl_step_get_stall_status((uint8_t) i));
            debug_println("");
        }
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

    /* 清除堵转保护: clear <joint_id> */
    if (strncmp(cmd, "clear", 5) == 0) {
        int joint = -1;
        if (sscanf(cmd, "clear %d", &joint) != 1) {
            debug_println("[CLEAR] Usage: clear <id> (id=0..5)");
            return true;
        }
        if (joint < 0 || joint > 5) {
            debug_println("[CLEAR] Error: joint should be 0-5");
            return true;
        }

        /* 调用motion_controller的清除堵转函数 */
        int ret = motion_clear_stall((uint8_t)joint);
        if (ret != 0) {
            debug_print("[CLEAR] Failed, ret=");
            debug_print_int(ret);
            debug_println("");
            return true;
        }

        /* 同步清除软件层故障标志，避免控制流程卡在保护态 */
        degradation_clear();
        g_collision_detected = false;
        g_stall_count[joint] = 0U;
        debug_println("[CLEAR] Done.");
        return true;
    }

    /* 电机测试命令: mtest [test_id] */
    if (strncmp(cmd, "mtest", 5) == 0) {
        if (cmd[5] == '\0') {
            /* 运行所有测试 */
            debug_println("[MTEST] Running all motor tests...");
            motor_test_run_all();
        } else if (cmd[5] == ' ') {
            /* 运行单个测试 */
            int test_id = atoi(cmd + 6);
            if (test_id < 1 || test_id > 7) {
                debug_println("[MTEST] Usage: mtest [1-7]");
                debug_println("  1: Enable/Disable");
                debug_println("  2: Position Control");
                debug_println("  3: Velocity Control");
                debug_println("  4: Position Query");
                debug_println("  5: Temperature Query");
                debug_println("  6: Multi-Joint Sync");
                debug_println("  7: Parameter Config");
                return true;
            }
            debug_print("[MTEST] Running test ");
            debug_print_int(test_id);
            debug_println("...");
            motor_test_run_single((uint8_t)test_id);
        }
        return true;
    }

    /* 手动电机控制: mmove <joint> <angle> */
    if (strncmp(cmd, "mmove ", 6) == 0) {
        int joint = -1;
        float angle = 0.0f;
        if (sscanf(cmd, "mmove %d %f", &joint, &angle) != 2) {
            debug_println("[MMOVE] Usage: mmove <joint> <angle>");
            debug_println("  joint: 0-5");
            debug_println("  angle: degrees");
            return true;
        }
        if (joint < 0 || joint > 5) {
            debug_println("[MMOVE] Error: joint should be 0-5");
            return true;
        }
        debug_print("[MMOVE] Moving joint ");
        debug_print_int(joint);
        debug_print(" to ");
        debug_print_int((int)angle);
        debug_println(" degrees...");
        motor_test_move_to((uint8_t)joint, angle);
        return true;
    }

    /* 超时检测测试: mtimeout */
    if (strncmp(cmd, "mtimeout", 8) == 0) {
        debug_println("[MTEST] Running timeout check test...");
        motor_test_timeout_check();
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

    /* CAN调试命令: can ping/enable/disable */
    if (strncmp(cmd, "can ", 4) == 0) {
        const char *sub = cmd + 4;

        if (strncmp(sub, "ping ", 5) == 0) {
            int id = atoi(sub + 5);
            if (id < 0 || id > 5) {
                debug_println("[CAN] Error: id should be 0-5");
                return true;
            }
            debug_print("[CAN] Pinging motor ");
            debug_print_int(id);
            debug_println("...");

            motor_ctrl_step_feedback_t feedback = {0};
            if (ctrl_step_ping_joint((uint8_t) id, 500U, &feedback)) {
                debug_print("[CAN] OK! State=0x");
                debug_print_hex(feedback.state_byte);
                debug_println("");
            } else {
                debug_println("[CAN] FAILED - no response (timeout 500ms)");
            }
        }
        else if (strncmp(sub, "enable", 6) == 0) {
            debug_println("[CAN] Enabling all motors (one by one)...");
            /* 逐个发送使能命令到每个电机 */
            for (int i = 0; i < 6; i++) {
                uint8_t addr = 1 + i;  /* 电机地址1-6 */
                uint32_t can_id = (uint32_t)addr << 8;
                uint8_t data[5] = {0xF3, 0xAB, 0x01, 0x00, 0x6B};

                debug_print("[CAN] Motor ");
                debug_print_int(i);
                debug_print(" (addr=");
                debug_print_int(addr);
                debug_print(") ID=0x");
                debug_print_int((int)can_id);
                debug_println("");

                /* 直接调用FSP发送 */
                can_frame_t frame = {
                    .id = can_id,
                    .id_mode = CAN_ID_MODE_EXTENDED,
                    .type = CAN_FRAME_TYPE_DATA,
                    .data_length_code = 5,
                    .options = 0
                };
                memcpy(frame.data, data, 5);
                fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, 0, &frame);
                debug_print("  result=");
                debug_print_int((int)err);
                debug_println(err == 0 ? " (OK)" : " (FAIL)");

                vTaskDelay(pdMS_TO_TICKS(10));
            }
            /* 重置超时计时器 */
            /* CtrlStep超时检测API待P0-2任务实现，这里不再调用重置接口。 */
            debug_println("[CAN] Done");
        }
        else if (strncmp(sub, "disable", 7) == 0) {
            debug_println("[CAN] Disabling all motors...");
            (void) motor_ctrl_step_set_enable(MOTOR_CTRL_STEP_ALL_JOINTS, false);
            debug_println("[CAN] Done");
        }
        else if (strncmp(sub, "raw", 3) == 0) {
            /* 发送原始CAN帧测试: can raw
             * 发送到地址1: ID=0x100, Data=F3 AB 01 00 6B (使能命令)
             */
            debug_println("[CAN] Sending raw frame to motor 1...");
            debug_println("  ID=0x00000100 (Extended)");
            debug_println("  Data=F3 AB 01 00 6B");

            can_frame_t frame = {
                .id = 0x100,  /* 地址1 << 8 = 0x100 */
                .id_mode = CAN_ID_MODE_EXTENDED,
                .type = CAN_FRAME_TYPE_DATA,
                .data_length_code = 5,
                .options = 0  /* 普通CAN帧，不是CAN FD */
            };
            frame.data[0] = 0xF3;  /* 功能码 */
            frame.data[1] = 0xAB;  /* 参数 */
            frame.data[2] = 0x01;  /* 使能 */
            frame.data[3] = 0x00;  /* 同步标志 */
            frame.data[4] = 0x6B;  /* 校验 */

            fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, 0, &frame);
            debug_print("  R_CANFD_Write result=");
            debug_print_int((int)err);
            if (err == FSP_SUCCESS) {
                debug_println(" (OK)");
            } else if (err == FSP_ERR_CAN_TRANSMIT_NOT_READY) {
                debug_println(" (TX busy)");
            } else {
                debug_println(" (ERROR)");
            }
        }
        else if (strncmp(sub, "std", 3) == 0) {
            /* 🕵️ 嫌疑四测试: 用标准帧发送使能命令
             * 有些ZDT固件只监听标准帧，ID直接是0x01而不是0x100
             */
            debug_println("[CAN] Sending STANDARD frame to motor 1...");
            debug_println("  ID=0x01 (Standard 11-bit)");
            debug_println("  Data=F3 AB 01 00 6B");

            can_frame_t frame = {
                .id = 0x01,  /* 标准帧ID直接是地址 */
                .id_mode = CAN_ID_MODE_STANDARD,  /* 标准帧! */
                .type = CAN_FRAME_TYPE_DATA,
                .data_length_code = 5,
                .options = 0
            };
            frame.data[0] = 0xF3;
            frame.data[1] = 0xAB;
            frame.data[2] = 0x01;
            frame.data[3] = 0x00;
            frame.data[4] = 0x6B;

            fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, 0, &frame);
            debug_print("  R_CANFD_Write result=");
            debug_print_int((int)err);
            debug_println(err == 0 ? " (OK)" : " (FAIL)");
            debug_println("");
            debug_println("If motor locks shaft now -> Standard frame works!");
            debug_println("Try to rotate motor shaft by hand.");
        }
        else if (strncmp(sub, "move", 4) == 0) {
            /* 发送速度模式命令让电机转动 */
            debug_println("[CAN] Sending VELOCITY command to motor 1...");
            debug_println("  Speed: 100 RPM (Emm protocol)");
            debug_println("  Direction: CW");
            debug_println("");

            /* 先使能电机 */
            debug_println("  Step 1: Enable motor...");
            {
                can_frame_t frame = {
                    .id = 0x100,
                    .id_mode = CAN_ID_MODE_EXTENDED,
                    .type = CAN_FRAME_TYPE_DATA,
                    .data_length_code = 5,
                    .options = 0
                };
                frame.data[0] = 0xF3;  /* 功能码: 使能 */
                frame.data[1] = 0xAB;  /* 参数 */
                frame.data[2] = 0x01;  /* 使能 */
                frame.data[3] = 0x00;  /* 同步标志 */
                frame.data[4] = 0x6B;  /* 校验 */
                R_CANFD_Write(&g_can0_ctrl, 0, &frame);
            }
            vTaskDelay(pdMS_TO_TICKS(100));

            /* 发送速度模式命令 */
            debug_println("  Step 2: Send velocity command...");
            {
                /* Emm速度模式: 0xF6 + 方向 + 速度(2B) + 加速度(1B) + 同步 + 校验 */
                /* 速度 = 100 RPM = 0x0064 */
                /* 加速度 = 10 */
                can_frame_t frame = {
                    .id = 0x100,
                    .id_mode = CAN_ID_MODE_EXTENDED,
                    .type = CAN_FRAME_TYPE_DATA,
                    .data_length_code = 7,
                    .options = 0
                };
                frame.data[0] = 0xF6;  /* 功能码: 速度模式 */
                frame.data[1] = 0x00;  /* 方向: 0=CW */
                frame.data[2] = 0x00;  /* 速度高字节 (100 RPM) */
                frame.data[3] = 0x64;  /* 速度低字节 */
                frame.data[4] = 0x0A;  /* 加速度 (10) */
                frame.data[5] = 0x00;  /* 同步标志 */
                frame.data[6] = 0x6B;  /* 校验 */
                R_CANFD_Write(&g_can0_ctrl, 0, &frame);
            }

            debug_println("");
            debug_println("  Motor should be spinning now!");
            debug_println("  Use 'can stop' to stop the motor.");
        }
        else if (strncmp(sub, "stop", 4) == 0) {
            /* 发送急停命令 */
            debug_println("[CAN] Sending STOP command to motor 1...");
            {
                can_frame_t frame = {
                    .id = 0x100,
                    .id_mode = CAN_ID_MODE_EXTENDED,
                    .type = CAN_FRAME_TYPE_DATA,
                    .data_length_code = 5,
                    .options = 0
                };
                frame.data[0] = 0xFE;  /* 功能码: 急停 */
                frame.data[1] = 0x98;  /* 参数 */
                frame.data[2] = 0x00;  /* 同步标志 */
                frame.data[3] = 0x6B;  /* 校验 */
                frame.data[4] = 0x00;
                R_CANFD_Write(&g_can0_ctrl, 0, &frame);
            }
            debug_println("[CAN] Stop command sent.");
        }
        /* 'can test' command removed - dummy-auk uses CtrlStep protocol, not Emm protocol */
        else if (strncmp(sub, "info", 4) == 0) {
            /* CtrlStep未提供TX/error flags统计接口，改为输出反馈缓存状态。 */
            debug_println("[CAN] CtrlStep feedback cache:");
            for (int i = 0; i < 6; i++) {
                motor_ctrl_step_feedback_t feedback = {0};
                bool has_feedback = motor_ctrl_step_get_feedback((uint8_t) i, &feedback);

                debug_print("  J");
                debug_print_int(i);
                debug_print(" online=");
                debug_print_int((has_feedback && feedback.online) ? 1 : 0);
                debug_print(" state=0x");
                debug_print_hex(has_feedback ? feedback.state_byte : 0U);
                debug_print(" last_rx_tick=");
                debug_print_int(has_feedback ? (int) feedback.last_rx_tick : 0);
                debug_println("");
            }
        }
        else if (strncmp(sub, "clear", 5) == 0) {
            debug_println("[CAN] Clearing CtrlStep clog latch for all joints...");
            int ret = motor_ctrl_step_clear_clog(MOTOR_CTRL_STEP_ALL_JOINTS);
            if (ret == 0) {
                debug_println("[CAN] Done");
            } else {
                debug_print("[CAN] Failed, ret=");
                debug_print_int(ret);
                debug_println("");
            }
        }
        else {
            debug_println("[CAN] Usage:");
            debug_println("  can move       - Spin motor 1 at 100 RPM");
            debug_println("  can stop       - Stop motor 1");
            debug_println("  can enable     - Enable all motors");
            debug_println("  can disable    - Disable all motors");
            debug_println("  can ping <id>  - Test motor communication (0-5)");
            debug_println("  can raw        - Send Extended frame (ID=0x100)");
            debug_println("  can std        - Send Standard frame (ID=0x01)");
            debug_println("  can test       - Full diagnostic test");
            debug_println("  can info       - Show CAN debug info");
            debug_println("  can clear      - Clear error flags");
        }
        return true;
    }

    /* 自动扫描命令 */
    /* scan       - 默认扫描，只测前方 (步进50mm) */
    /* scan full  - 全向扫描，前后都测 (步进50mm) */
    /* scan fine  - 精细扫描 (步进30mm) */
    if (strncmp(cmd, "scan", 4) == 0) {
        int step = 50;  /* 默认步进 */
        int full_mode = 0;  /* 是否测试前后两面 */

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

        /*
         * 测试范围 (基于ZERO机械臂臂长约400mm):
         * X: -300 ~ 300 mm (左右)
         * Y: ±100 ~ ±400 mm (前后，避开基座附近)
         * Z: -200 ~ 200 mm (上下)
         */
        for (int x = -300; x <= 300; x += step) {
            /* 前方 Y: 100~400 */
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

            /* 后方 Y: -400~-100 (仅full模式) */
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

    /* ========== ZDT官方源码测试命令 ========== */
    if (strncmp(cmd, "zdt ", 4) == 0) {
        const char *sub = cmd + 4;

        if (strncmp(sub, "init", 4) == 0) {
            zdt_test_init();
        }
        else if (strncmp(sub, "vel", 3) == 0) {
            /* 速度模式测试 */
            zdt_test_motor1_velocity();
        }
        else if (strncmp(sub, "pos", 3) == 0) {
            /* 位置模式测试 */
            zdt_test_motor1_position();
        }
        else if (strncmp(sub, "on", 2) == 0) {
            /* 使能电机 */
            zdt_test_motor1_enable(true);
        }
        else if (strncmp(sub, "off", 3) == 0) {
            /* 失能电机 */
            zdt_test_motor1_enable(false);
        }
        else if (strncmp(sub, "stop", 4) == 0) {
            /* 停止电机 */
            zdt_test_motor1_stop();
        }
        else if (strncmp(sub, "status", 6) == 0) {
            /* 读取状态 */
            zdt_test_motor1_read_status();
        }
        else {
            debug_println("[ZDT] Usage:");
            debug_println("  zdt init   - Initialize ZDT test module");
            debug_println("  zdt vel    - Test velocity mode (100RPM)");
            debug_println("  zdt pos    - Test position mode (3600deg)");
            debug_println("  zdt on     - Enable motor 1");
            debug_println("  zdt off    - Disable motor 1");
            debug_println("  zdt stop   - Stop motor 1");
            debug_println("  zdt status - Read motor 1 status");
            debug_println("");
            debug_println("  Note: Motor 1 is 50:1 geared stepper");
            debug_println("  100RPM motor = 2RPM output shaft");
        }
        return true;
    }

    if (cmd[0] != 't' || cmd[1] != ' ') {
        return false;  /* 不是测试命令 */
    }

    /* 解析 "t x y z" */
    int x = 0, y = 0, z = 0;
    const char *p = cmd + 2;

    /* 跳过空格 */
    while (*p == ' ') p++;
    x = atoi(p);

    /* 找下一个数字 */
    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;
    y = atoi(p);

    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;
    z = atoi(p);

    debug_print("[TEST] IK for offset: x=");
    debug_print_int(x);
    debug_print(" y=");
    debug_print_int(y);
    debug_print(" z=");
    debug_print_int(z);
    debug_println("");

    /* 调用运动控制器测试IK */
    int ret = motion_move_to_xyz((float)x, (float)y, (float)z);
    if (ret == 0) {
        debug_println("[TEST] IK OK! (motion started)");
    } else {
        debug_println("[TEST] IK FAILED");
    }

    return true;

    /* 未匹配任何命令，返回false让LLM处理 */
    return false;
}

/*
 * 主线程入口
 */
void new_thread0_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 初始化公共模块 */
    app_common_init();

    /* 初始化调试串口 */
    debug_uart_init();
    debug_println("=== RA6M5 AI Pharmacy Robot ===");

    /* 检查是否因看门狗复位 */
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

    debug_println("[DBG] motion_init...");
    /* 初始化运动控制器 */
    motion_init();
    debug_println("[DBG] motion_init done");

    /* 初始化降级策略模块 */
    degradation_init();
    debug_println("[DEGRADE] Degradation module initialized");

    /* 初始化实时性监测 (DWT cycle counter) */
    rtmon_init();
    debug_println("[RTMON] Real-time monitor initialized");

    /* 初始化任务监控 */
    taskmon_init();
    taskmon_register("main_thread");  /* 注册主线程 */
    debug_println("[TASKMON] Task monitor initialized");

    debug_println("[DBG] GPT open...");
    /* 启动运动控制定时器 (5ms周期, 200Hz) */
    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    R_GPT_Start(&g_timer0_ctrl);
    debug_println("[TIMER] Motion timer started (5ms, 200Hz)");

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
        w800_init(w800_data_received);

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
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

            /* 硬件复位W800模块 (通过P508引脚) */
            debug_println("[WIFI] Hardware resetting W800...");
            w800_hardware_reset();

            /* 等待W800初始化完成 (UART命令接口需要时间启动) */
            debug_println("[WIFI] Waiting for W800 to initialize (3s)...");
            vTaskDelay(pdMS_TO_TICKS(3000));

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

        if (wifi_connected) {
            debug_println("");
            debug_println("Commands:");
            debug_println("  === Motion ===");
            debug_println("  x x y z       - Cartesian move (e.g. x 0 0 50)");
            debug_println("  m j0..j5      - Multi-joint move");
            debug_println("  j id angle    - Single joint (e.g. j 0 90)");
            debug_println("  home id       - Set current encoder as zero");
            debug_println("  clear id      - Clear stall protection");
            debug_println("  enable id     - Enable one motor");
            debug_println("  query id      - Query one motor position");
            debug_println("  t x y z       - Legacy alias of x command");
            debug_println("  scan          - Auto scan workspace");
            debug_println("  stop          - Emergency stop");
            debug_println("");
            debug_println("  === Motor Tuning ===");
            debug_println("  fast          - Fast mode (high acc/PID)");
            debug_println("  smooth        - Smooth mode (low acc/PID)");
            debug_println("  acc val       - Set acceleration (100-10000)");
            debug_println("  pid j kp kv ki- Set PID (e.g. pid 0 2000 1500 500)");
            debug_println("  status        - Show motor states");
            debug_println("");
            debug_println("  === CAN Debug ===");
            debug_println("  can ping <id> - Test motor comm (0-5)");
            debug_println("  can enable    - Enable all motors");
            debug_println("  can disable   - Disable all motors");
            debug_println("");
            debug_println("  === Gripper ===");
            debug_println("  grip open     - Open gripper");
            debug_println("  grip close    - Close gripper");
            debug_println("  grip vacuum on/off");
            debug_println("");
            debug_println("  === Diagnostics ===");
            debug_println("  rtmon         - Show real-time stats");
            debug_println("  rtmon reset   - Reset statistics");
            debug_println("  taskmon       - Show task CPU usage");
            debug_println("  taskmon reset - Reset task stats");
            debug_println("  degrade       - Show degradation status");
            debug_println("");
            debug_println("  <text>        - Send to LLM");
            debug_println("");
            debug_print("> ");
        }
    }

    char cmd_buf[256];
    uint32_t heartbeat_counter = 0;
    uint32_t collision_check_counter = 0;
    uint32_t taskmon_counter = 0;  /* 任务监控更新计数器 */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 检查用户输入 */
        if (debug_has_line()) {
            int len = debug_read_line(cmd_buf, sizeof(cmd_buf));
            if (len > 0) {
                /* 先检查是否是本地测试命令 */
                if (!handle_test_command(cmd_buf)) {
                    /* 不是测试命令，发送到LLM */
                    send_command_to_llm(cmd_buf);
                }
            }
            debug_print("> ");
        }

        /* 检查LLM响应 */
        if (g_llm_resp_ready) {
            process_llm_response(g_llm_response);
            g_llm_resp_ready = false;
            g_llm_resp_idx = 0;
            debug_print("> ");
        }

        /* 更新运动控制器 (由5ms硬件定时器触发) */
        if (g_motion_tick) {
            g_motion_tick = false;  /* 清除标志 */

            motion_state_t mstate = motion_get_state();
            if (mstate == MOTION_EXECUTING) {
                rtmon_wcet_start();
                motion_update();
                rtmon_wcet_end();
            }
            /* 运动完成，继续执行下一个动作 */
            if (g_executing_sequence && mstate == MOTION_DONE) {
                execute_next_action();
            }

            /* 碰撞检测 (10ms周期，每2个tick检测一次) */
            collision_check_counter++;
            if (collision_check_counter >= 2 && !g_collision_detected) {
                collision_check_counter = 0;

                /* CAN超时检测 (电机失联) */
                uint8_t timeout_mask = motor_ctrl_step_check_timeout();
                if (timeout_mask != 0) {
                    /* 找到第一个超时的关节 */
                    for (int i = 0; i < 6; i++) {
                        if (timeout_mask & (1U << i)) {
                            /* Use degradation strategy instead of immediate emergency stop */
                            degrade_level_t level = degradation_handle_fault(
                                FAULT_CAN_TIMEOUT, (uint8_t)i);

                            if (level == DEGRADE_EMERGENCY) {
                                g_collision_detected = true;
                                g_executing_sequence = false;
                            } else if (level == DEGRADE_SINGLE_JOINT) {
                                /* Continue with degraded operation */
                                debug_print("[DEGRADE] Continuing without J");
                                debug_print_int(i);
                                debug_println("");
                            }
                            debug_print("> ");
                            break; /* 只处理第一个超时关节 */
                        }
                    }
                }

                /* 检查所有电机的堵转状态 */
                for (int i = 0; i < 6; i++) {
                    /* Skip disabled joints */
                    if (degradation_is_joint_disabled((uint8_t)i)) {
                        continue;
                    }

                    if (ctrl_step_get_stall_status((uint8_t)i) == 1) {
                        /* 堵转计数+1 */
                        g_stall_count[i]++;

                        /* 连续检测到堵转超过阈值才触发 */
                        if (g_stall_count[i] >= COLLISION_CONFIRM_COUNT) {
                            /* Use degradation strategy */
                            degrade_level_t level = degradation_handle_fault(
                                FAULT_MOTOR_STALL, (uint8_t)i);

                            if (level >= DEGRADE_POSITION_HOLD) {
                                g_collision_detected = true;
                                g_executing_sequence = false;
                                debug_println("");
                                debug_print("[COLLISION] Motor J");
                                debug_print_int(i);
                                debug_print(" stall -> ");
                                debug_println(degradation_level_str(level));
                                debug_println("[COLLISION] Use 'clear <id>' to reset.");
                                debug_print("> ");
                            }
                            break;
                        }
                    } else {
                        /* 堵转状态消失，重置计数 */
                        g_stall_count[i] = 0;
                    }
                }

                /* Attempt auto-recovery if in degraded state */
                if (degradation_get_level() > DEGRADE_NONE &&
                    degradation_get_level() < DEGRADE_EMERGENCY) {
                    if (degradation_recover()) {
                        debug_print("[DEGRADE] Auto-recovered to ");
                        debug_println(degradation_level_str(degradation_get_level()));
                        if (degradation_get_level() == DEGRADE_NONE) {
                            g_collision_detected = false;
                        }
                        debug_print("> ");
                    }
                }
            }

            /* 心跳LED */
            heartbeat_counter++;
            if (heartbeat_counter >= 100) {  /* 5ms * 100 = 500ms */
                heartbeat_counter = 0;

                /* 喂狗 (每500ms刷新一次，超时约2.7秒) */
                watchdog_refresh();

                /* 任务监控更新 (每秒一次) */
                taskmon_counter++;
                if (taskmon_counter >= 2) {  /* 500ms * 2 = 1s */
                    taskmon_counter = 0;
                    taskmon_update("main_thread", 0);
                }

                if (g_wifi_error_code != 0) {
                    R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN, BSP_IO_LEVEL_HIGH);
                } else {
                    R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN, BSP_IO_LEVEL_HIGH);
                }
            }
            if (heartbeat_counter == 10) {  /* 50ms后关闭LED */
                R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN, BSP_IO_LEVEL_LOW);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));  /* 1ms绝对延迟，保证固定周期 */
    }
}

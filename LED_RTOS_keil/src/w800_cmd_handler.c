/**
 * @file    w800_cmd_handler.c
 * @brief   W800 TCP透传 JSON 命令处理实现
 */

#include "w800_cmd_handler.h"
#include "dummy_arm_cmd.h"
#include "gripper.h"
#include "llm_action.h"
#include "maixcam_uart.h"
#include "motion_controller.h"
#include "slot_teach.h"
#include "w800_driver.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define W800_CMD_RSP_MAX_LEN         640
#define W800_CMD_LLM_RSP_MAX_LEN     384
#define W800_SCAN_TYPE_DEFAULT       0U
#define W800_SCAN_FRAMES_DEFAULT     5U

static const char * motion_state_to_str(motion_state_t state)
{
    switch (state) {
        case MOTION_IDLE: return "IDLE";
        case MOTION_PLANNING: return "PLANNING";
        case MOTION_EXECUTING: return "EXECUTING";
        case MOTION_DONE: return "DONE";
        case MOTION_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

static const char * motion_ret_to_err(int ret)
{
    switch (ret) {
        case -1: return "Busy or invalid params";
        case -2: return "IK failed";
        case -3: return "Trajectory failed";
        case -4: return "Motor start failed";
        case -5: return "Joint limit";
        case -6: return "USB not connected";
        case -7: return "Move settle timeout";
        default: return "Motion failed";
    }
}

static void json_escape_string(const char *src, char *dst, size_t dst_size)
{
    size_t j = 0;

    if ((src == NULL) || (dst == NULL) || (dst_size == 0U)) {
        return;
    }

    for (size_t i = 0; src[i] != '\0' && j + 1U < dst_size; i++) {
        char c = src[i];

        if ((c == '\"') || (c == '\\')) {
            if (j + 2U >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = c;
        } else if (c == '\r') {
            if (j + 2U >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 'r';
        } else if (c == '\n') {
            if (j + 2U >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 'n';
        } else if (c == '\t') {
            if (j + 2U >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 't';
        } else if ((uint8_t)c >= 0x20U) {
            dst[j++] = c;
        }
    }

    dst[j] = '\0';
}

static void w800_cmd_send_json(const char *fmt, ...)
{
    char body[W800_CMD_RSP_MAX_LEN];
    char line[W800_CMD_RSP_MAX_LEN + 3];
    va_list args;
    int n;

    va_start(args, fmt);
    n = vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);
    if (n < 0) {
        return;
    }

    n = snprintf(line, sizeof(line), "%s\r\n", body);
    if (n <= 0) {
        return;
    }

    if ((size_t)n >= sizeof(line)) {
        n = (int)(sizeof(line) - 1U);
    }

    (void)W800_UART_Send((const uint8_t *)line, (uint32_t)n);
}

static void w800_cmd_send_ok(void)
{
    w800_cmd_send_json("{\"ok\":true}");
}

static void w800_cmd_send_err(const char *err)
{
    if (err == NULL) {
        err = "Unknown error";
    }
    w800_cmd_send_json("{\"ok\":false,\"err\":\"%s\"}", err);
}

static int parse_joint_array(const char *json, float joints[6])
{
    const char *p = strstr(json, "\"j\"");
    if (p == NULL) {
        return -1;
    }

    p = strchr(p, '[');
    if (p == NULL) {
        return -1;
    }

    if (sscanf(p, " [ %f , %f , %f , %f , %f , %f ] ",
               &joints[0], &joints[1], &joints[2],
               &joints[3], &joints[4], &joints[5]) != 6) {
        return -1;
    }

    return 0;
}

void w800_cmd_handle(const char *json_line)
{
    char cmd[24] = {0};

    debug_print("[CMD] w800_cmd_handle: ");
    debug_println(json_line ? json_line : "(null)");

    if (json_line == NULL) {
        w800_cmd_send_err("Empty command");
        return;
    }

    if (parse_string_after(json_line, "\"cmd\"", cmd, sizeof(cmd)) <= 0) {
        w800_cmd_send_err("Missing cmd");
        return;
    }

    if (strcmp(cmd, "status") == 0) {
        dummy_joint_pos_t jp;
        if (dummy_arm_get_joint_pos(&jp) != 0) {
            w800_cmd_send_err("Get joint failed");
            return;
        }

        w800_cmd_send_json(
            "{\"ok\":true,\"joints\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],\"state\":\"%s\"}",
            jp.j[0], jp.j[1], jp.j[2], jp.j[3], jp.j[4], jp.j[5],
            motion_state_to_str(motion_get_state()));
        return;
    }

    if (strcmp(cmd, "move_xyz") == 0) {
        /* move_xyz: 非阻塞，立即返回 state=EXECUTING，客户端用 status 轮询 */
        if ((strstr(json_line, "\"x\"") == NULL) ||
            (strstr(json_line, "\"y\"") == NULL) ||
            (strstr(json_line, "\"z\"") == NULL)) {
            w800_cmd_send_err("Missing xyz");
            return;
        }

        float x = parse_float_after(json_line, "\"x\"");
        float y = parse_float_after(json_line, "\"y\"");
        float z = parse_float_after(json_line, "\"z\"");
        int ret = motion_start_xyz(x, y, z);
        if (ret == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err(motion_ret_to_err(ret));
        }
        return;
    }

    if (strcmp(cmd, "move_joint") == 0) {
        /* move_joint: 非阻塞，立即返回 state=EXECUTING，客户端用 status 轮询 */
        float joints[6] = {0.0f};
        float spd = parse_float_after(json_line, "\"spd\"");
        (void)spd;

        if (parse_joint_array(json_line, joints) != 0) {
            w800_cmd_send_err("Invalid joints");
            return;
        }

        int ret = motion_start_joints(joints);
        if (ret == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err(motion_ret_to_err(ret));
        }
        return;
    }

    if (strcmp(cmd, "home") == 0) {
        if (dummy_arm_home() == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err("Home failed");
        }
        return;
    }

    if (strcmp(cmd, "rest") == 0) {
        if (dummy_arm_rest() == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err("Rest failed");
        }
        return;
    }

    if (strcmp(cmd, "enable") == 0) {
        int r1 = dummy_arm_set_mode(DUMMY_MODE_INTERRUPTABLE);
        int r2 = dummy_arm_enable();
        if ((r1 == 0) && (r2 == 0)) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err("Enable failed");
        }
        return;
    }

    if (strcmp(cmd, "stop") == 0) {
        if (dummy_arm_stop() == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err("Stop failed");
        }
        return;
    }

    if (strcmp(cmd, "grip") == 0) {
        char action[8] = {0};
        int ret = -1;

        if (parse_string_after(json_line, "\"action\"", action, sizeof(action)) <= 0) {
            w800_cmd_send_err("Missing action");
            return;
        }

        if (strcmp(action, "on") == 0) {
            ret = vacuum_on();
        } else if (strcmp(action, "off") == 0) {
            ret = vacuum_off();
        } else {
            w800_cmd_send_err("Invalid grip action");
            return;
        }

        if (ret == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err("Grip failed");
        }
        return;
    }

    if (strcmp(cmd, "scan") == 0) {
        if (maixcam_uart_send_req_scan(W800_SCAN_TYPE_DEFAULT, W800_SCAN_FRAMES_DEFAULT) == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err("Scan failed");
        }
        return;
    }

    if (strcmp(cmd, "speak") == 0) {
        char text[64] = {0};
        int len = parse_string_after(json_line, "\"text\"", text, sizeof(text));
        if (len < 0) {
            w800_cmd_send_err("Missing text");
            return;
        }

        if (maixcam_uart_send_speak(text, (uint8_t)strlen(text)) == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err("Speak failed");
        }
        return;
    }

    if (strcmp(cmd, "move_rel") == 0) {
        /* 相对笛卡尔移动：基于当前末端位置的增量 (mm) */
        if ((strstr(json_line, "\"dx\"") == NULL) ||
            (strstr(json_line, "\"dy\"") == NULL) ||
            (strstr(json_line, "\"dz\"") == NULL)) {
            w800_cmd_send_err("Missing dx/dy/dz");
            return;
        }

        float dx = parse_float_after(json_line, "\"dx\"");
        float dy = parse_float_after(json_line, "\"dy\"");
        float dz = parse_float_after(json_line, "\"dz\"");
        int ret = motion_start_by_xyz(dx, dy, dz);
        if (ret == 0) {
            w800_cmd_send_ok();
        } else {
            w800_cmd_send_err(motion_ret_to_err(ret));
        }
        return;
    }

    if (strcmp(cmd, "lpos") == 0) {
        /* 查询末端笛卡尔位姿 (mm / deg) */
        dummy_cart_pos_t cp;
        if (dummy_arm_get_cart_pos(&cp) != 0) {
            w800_cmd_send_err("lpos query failed");
            return;
        }
        w800_cmd_send_json(
            "{\"ok\":true,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f"
            ",\"a\":%.2f,\"b\":%.2f,\"c\":%.2f}",
            cp.x, cp.y, cp.z, cp.a, cp.b, cp.c);
        return;
    }

    /* ========== 槽位命令 ========== */

    if (strcmp(cmd, "slot_goto") == 0) {
        int id = (int)parse_float_after(json_line, "\"id\"");
        if (id < 0 || id >= SLOT_NUM) {
            w800_cmd_send_err("Slot id out of range (0-15)");
            return;
        }
        int ret = slot_goto((uint8_t)id);
        if (ret == 0) {
            w800_cmd_send_ok();
        } else if (ret == -2) {
            w800_cmd_send_err("Slot not taught");
        } else {
            w800_cmd_send_err(motion_ret_to_err(ret));
        }
        return;
    }

    if (strcmp(cmd, "slot_grab") == 0) {
        int id = (int)parse_float_after(json_line, "\"id\"");
        if (id < 0 || id >= SLOT_NUM) {
            w800_cmd_send_err("Slot id out of range (0-15)");
            return;
        }
        int ret = slot_grab((uint8_t)id);
        if (ret == 0) {
            w800_cmd_send_ok();
        } else if (ret == -2) {
            w800_cmd_send_err("Slot not taught");
        } else {
            w800_cmd_send_err(motion_ret_to_err(ret));
        }
        return;
    }

    if (strcmp(cmd, "slot_fetch") == 0) {
        int id = (int)parse_float_after(json_line, "\"id\"");
        if (id < 0 || id >= SLOT_NUM) {
            w800_cmd_send_err("Slot id out of range (0-15)");
            return;
        }
        int ret = slot_fetch((uint8_t)id);
        if (ret == 0) {
            w800_cmd_send_ok();
        } else if (ret == -2) {
            w800_cmd_send_err("Slot not taught");
        } else {
            w800_cmd_send_err(motion_ret_to_err(ret));
        }
        return;
    }

    if (strcmp(cmd, "slot_list") == 0) {
        char rsp[W800_CMD_RSP_MAX_LEN];
        int pos = snprintf(rsp, sizeof(rsp), "{\"ok\":true,\"slots\":[");
        bool first = true;

        for (int i = 0; i < SLOT_NUM; i++) {
            float j[6];
            bool taught = slot_get_joints((uint8_t)i, j);
            if (!taught) continue;

            int wrote = snprintf(rsp + pos, sizeof(rsp) - (size_t)pos,
                "%s{\"id\":%d,\"j\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}",
                first ? "" : ",",
                i, j[0], j[1], j[2], j[3], j[4], j[5]);
            if (wrote < 0 || (size_t)(pos + wrote) >= sizeof(rsp) - 4U) break;
            pos += wrote;
            first = false;
        }

        snprintf(rsp + pos, sizeof(rsp) - (size_t)pos, "]}");
        w800_cmd_send_json("%s", rsp);
        return;
    }

    if (strcmp(cmd, "llm") == 0) {
        /* LLM disabled */
        w800_cmd_send_err("LLM disabled");
        return;
    }

    w800_cmd_send_err("Unknown cmd");
}

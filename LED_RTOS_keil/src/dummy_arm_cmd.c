/**
 ******************************************************************************
 * @file    dummy_arm_cmd.c
 * @brief   Dummy机械臂USB命令实现
 * @note    ASCII协议格式 (匹配 dummy-ref-core-fw):
 *            !START\r\n          → "Started ok\r\n"
 *            >j1,j2,...,j6,spd\r\n → 立即返回FIFO大小(数字), SEQUENTIAL模式下完成后返回"ok"
 *            #GETJPOS\r\n        → "ok 0.00 0.00 ...\r\n"
 *          重要: 运动命令产生双响应，必须正确处理
 ******************************************************************************
 */

#include "dummy_arm_cmd.h"
#include "usb_host_cdc.h"
#include "watchdog.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#define TAG "DUMMY_ARM"

/* 连续命令之间的最小间隔 (ms) - 防止固件来不及处理 */
#define CMD_MIN_INTERVAL_MS   50   /* 从20ms增到50ms，给固件更多处理时间 */

/* 上次命令发送时间 */
static uint32_t g_last_cmd_tick = 0;

/* ========== 内部工具 ========== */

/**
 * @brief 判断字符串是否为纯数字 (FIFO大小响应)
 */
static bool is_numeric_resp(const char *s)
{
    if (s == NULL || *s == '\0') return false;
    while (*s) {
        if (*s < '0' || *s > '9') return false;
        s++;
    }
    return true;
}

/**
 * @brief 确保命令间隔，防止连续发送导致固件丢命令
 */
static void cmd_pace(void)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t elapsed = (now - g_last_cmd_tick) * portTICK_PERIOD_MS;
    if (elapsed < CMD_MIN_INTERVAL_MS) {
        vTaskDelay(pdMS_TO_TICKS(CMD_MIN_INTERVAL_MS - elapsed));
    }
}

/**
 * @brief 更新命令时间戳
 */
static void cmd_stamp(void)
{
    g_last_cmd_tick = xTaskGetTickCount();
}

/**
 * @brief 发送命令并检查响应
 * @return 0=ok, -1=失败/超时/非预期响应
 */
static int send_and_check(const char *cmd, char *resp_out,
                          uint16_t resp_max, uint32_t timeout_ms)
{
    cmd_pace();

    char resp[128];
    int len = usb_cdc_command(cmd, resp, sizeof(resp), timeout_ms);

    cmd_stamp();

    if (len < 0) {
        LOG_W(TAG, "Timeout: %s", cmd);
        return -1;  /* 超时视为失败 */
    }

    /* 拷贝响应给调用者 */
    if (resp_out != NULL && resp_max > 0) {
        uint16_t copy = ((uint16_t)len < resp_max - 1)
                        ? (uint16_t)len : (resp_max - 1);
        memcpy(resp_out, resp, copy);
        resp_out[copy] = '\0';
    }

    /* 已知的成功响应 */
    if (strstr(resp, "ok") != NULL ||
        strstr(resp, "Started") != NULL ||
        strstr(resp, "Stopped") != NULL ||
        strstr(resp, "Disabled") != NULL ||
        strstr(resp, "calibration") != NULL) {
        return 0;
    }

    /* 纯数字 = 运动命令的FIFO大小响应，视为成功 */
    if (is_numeric_resp(resp)) {
        return 0;
    }

    LOG_W(TAG, "Unexpected resp: %s", resp);
    return -1;
}

/**
 * @brief 发送阻塞命令并检查响应 (HOME/RESET等需要长等待的命令)
 */
static int send_and_check_blocking(const char *cmd, uint32_t timeout_ms)
{
    cmd_pace();

    char resp[128];
    int len = usb_cdc_command_blocking(cmd, resp, sizeof(resp), timeout_ms);

    cmd_stamp();

    if (len < 0) {
        LOG_W(TAG, "Blocking timeout: %s", cmd);
        return -1;
    }

    if (strstr(resp, "ok") != NULL ||
        strstr(resp, "Started") != NULL ||
        strstr(resp, "Stopped") != NULL ||
        strstr(resp, "Disabled") != NULL) {
        return 0;
    }

    LOG_W(TAG, "Unexpected resp: %s", resp);
    return -1;
}

/* ========== 公共接口 ========== */

int dummy_arm_init(void)
{
    /* 注意: usb_cdc_init() 已在 new_thread0_entry 中调用
     * 不要重复调用，否则 R_USB_Open 会失败导致USB模块异常 */
    LOG_I(TAG, "Dummy arm cmd layer init");
    return 0;
}

bool dummy_arm_wait_ready(uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount();
    while (!usb_cdc_is_ready()) {
        uint32_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
        if (elapsed >= timeout_ms) return false;
        watchdog_refresh();  /* 喂狗，防止长等待触发复位 */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return true;
}

/* ---- 控制命令 ---- */

int dummy_arm_enable(void)
{
    return send_and_check("!START", NULL, 0, DUMMY_CMD_TIMEOUT);
}

int dummy_arm_disable(void)
{
    return send_and_check("!DISABLE", NULL, 0, DUMMY_CMD_TIMEOUT);
}

int dummy_arm_stop(void)
{
    return send_and_check("!STOP", NULL, 0, DUMMY_CMD_TIMEOUT);
}

int dummy_arm_home(void)
{
    return send_and_check_blocking("!HOME", DUMMY_BLOCKING_TIMEOUT);
}

int dummy_arm_rest(void)
{
    return send_and_check_blocking("!RESET", DUMMY_BLOCKING_TIMEOUT);
}

int dummy_arm_set_mode(dummy_cmd_mode_t mode)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "#CMDMODE %d", (int)mode);
    return send_and_check(cmd, NULL, 0, DUMMY_CMD_TIMEOUT);
}

/* ---- 运动命令 ---- */

/**
 * @brief 发送运动命令 (处理双响应)
 * @note  INTERRUPTABLE模式下只返回FIFO大小(数字), 无第二条"ok"
 *        SEQUENTIAL模式下会额外返回"ok"(运动完成后)
 *        本函数在INTERRUPTABLE模式下只读FIFO响应即返回
 */
static int send_motion_cmd(const char *cmd, uint32_t timeout_ms)
{
    cmd_pace();

    char resp[128];
    int len = usb_cdc_command(cmd, resp, sizeof(resp), timeout_ms);

    cmd_stamp();

    if (len < 0) {
        LOG_W(TAG, "Motion no resp: %s", cmd);
        return -1;  /* 超时视为失败 */
    }

    /* FIFO大小 (纯数字) 或 "ok" 都算成功 */
    if (is_numeric_resp(resp) || strstr(resp, "ok") != NULL) {
        return 0;
    }

    LOG_W(TAG, "Motion unexpected: %s", resp);
    return -1;
}

int dummy_arm_move_j(const dummy_joint_pos_t *pos, float speed)
{
    if (pos == NULL) return -1;

    char cmd[128];
    if (speed > 0.0f) {
        snprintf(cmd, sizeof(cmd), ">%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                 pos->j[0], pos->j[1], pos->j[2],
                 pos->j[3], pos->j[4], pos->j[5], speed);
    } else {
        snprintf(cmd, sizeof(cmd), ">%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                 pos->j[0], pos->j[1], pos->j[2],
                 pos->j[3], pos->j[4], pos->j[5]);
    }

    return send_motion_cmd(cmd, DUMMY_MOTION_TIMEOUT);
}

int dummy_arm_move_l(const dummy_cart_pos_t *pos, float speed)
{
    if (pos == NULL) return -1;

    char cmd[128];
    if (speed > 0.0f) {
        snprintf(cmd, sizeof(cmd), "@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                 pos->x, pos->y, pos->z,
                 pos->a, pos->b, pos->c, speed);
    } else {
        snprintf(cmd, sizeof(cmd), "@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                 pos->x, pos->y, pos->z,
                 pos->a, pos->b, pos->c);
    }

    return send_motion_cmd(cmd, DUMMY_MOTION_TIMEOUT);
}

int dummy_arm_move_joints(float j1, float j2, float j3,
                          float j4, float j5, float j6, float speed)
{
    dummy_joint_pos_t pos = {{ j1, j2, j3, j4, j5, j6 }};
    return dummy_arm_move_j(&pos, speed);
}

/* ---- 查询命令 ---- */

int dummy_arm_get_joint_pos(dummy_joint_pos_t *out)
{
    if (out == NULL) return -1;

    char resp[128];
    int len = usb_cdc_command("#GETJPOS", resp, sizeof(resp), DUMMY_CMD_TIMEOUT);
    if (len < 0) return -1;

    /* 响应格式: "ok 0.00 -73.00 180.00 0.00 0.00 0.00" */
    int n = sscanf(resp, "ok %f %f %f %f %f %f",
                   &out->j[0], &out->j[1], &out->j[2],
                   &out->j[3], &out->j[4], &out->j[5]);
    if (n != 6) {
        LOG_W(TAG, "GETJPOS parse fail: %s", resp);
        return -1;
    }
    return 0;
}

int dummy_arm_get_cart_pos(dummy_cart_pos_t *out)
{
    if (out == NULL) return -1;

    char resp[128];
    int len = usb_cdc_command("#GETLPOS", resp, sizeof(resp), DUMMY_CMD_TIMEOUT);
    if (len < 0) return -1;

    int n = sscanf(resp, "ok %f %f %f %f %f %f",
                   &out->x, &out->y, &out->z,
                   &out->a, &out->b, &out->c);
    if (n != 6) {
        LOG_W(TAG, "GETLPOS parse fail: %s", resp);
        return -1;
    }
    return 0;
}

/* ---- 电机参数 ---- */

int dummy_arm_set_kp(uint8_t node, float kp)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "#SET_DCE_KP %d %.4f", (int)node, kp);
    return send_and_check(cmd, NULL, 0, DUMMY_CMD_TIMEOUT);
}

int dummy_arm_set_ki(uint8_t node, float ki)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "#SET_DCE_KI %d %.4f", (int)node, ki);
    return send_and_check(cmd, NULL, 0, DUMMY_CMD_TIMEOUT);
}

int dummy_arm_set_kd(uint8_t node, float kd)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "#SET_DCE_KD %d %.4f", (int)node, kd);
    return send_and_check(cmd, NULL, 0, DUMMY_CMD_TIMEOUT);
}

int dummy_arm_reboot_node(uint8_t node)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "#REBOOT %d", (int)node);
    return send_and_check(cmd, NULL, 0, DUMMY_CMD_TIMEOUT);
}

/* ---- 原始命令 ---- */

int dummy_arm_raw_cmd(const char *cmd, char *resp, uint16_t resp_max,
                      uint32_t timeout_ms)
{
    return usb_cdc_command(cmd, resp, resp_max, timeout_ms);
}
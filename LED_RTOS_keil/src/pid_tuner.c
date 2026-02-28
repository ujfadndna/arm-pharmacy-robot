/**
 * @file    pid_tuner.c
 * @brief   PID调参命令处理模块 - 配合MCP Server使用
 * @note    支持的命令:
 *          - PID <Kp> <Kv> <Ki>    设置PID参数
 *          - GET_PID              获取当前PID参数
 *          - STEP <angle> <time>  执行阶跃响应测试
 *          - STATE                获取当前状态
 *          - JOINT <id>           选择要调参的关节
 */

#include "pid_tuner.h"
#include "debug_uart.h"
#include "motor_can.h"
#include "hal_data.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ========== 内部状态 ========== */
static pid_tuner_state_t g_state = PID_TUNER_IDLE;
static uint8_t g_current_joint = 0;  /* 当前调参的关节 */

/* 当前PID参数 (软件侧缓存) */
static pid_params_t g_pid_params = {1000, 1000, 300};

/* 阶跃响应数据 */
static struct {
    float setpoint;             /* 目标角度 */
    float start_angle;          /* 起始角度 */
    uint32_t duration_ms;       /* 测试时长 */
    uint32_t start_tick;        /* 开始时间 */
    uint32_t sample_count;      /* 采样计数 */
    float samples[PID_TUNER_MAX_SAMPLES];  /* 采样数据 */
    uint32_t sample_times[PID_TUNER_MAX_SAMPLES];  /* 采样时间 */
} g_step_data;

/* ========== 内部函数声明 ========== */
static void cmd_set_pid(const char *args);
static void cmd_get_pid(void);
static void cmd_step_response(const char *args);
static void cmd_get_state(void);
static void cmd_set_joint(const char *args);
static void cmd_send_raw(const char *args);

/* ========== 公共接口实现 ========== */

void pid_tuner_init(void)
{
    g_state = PID_TUNER_IDLE;
    g_current_joint = 0;
    memset(&g_step_data, 0, sizeof(g_step_data));

    debug_println("[PID_TUNER] Initialized");
    debug_println("[PID_TUNER] Commands: PID, GET_PID, STEP, STATE, JOINT");
}

void pid_tuner_process_command(void)
{
    /* 检查是否有新命令 */
    if (!debug_has_line()) {
        return;
    }

    /* 读取命令 */
    char cmd_buf[256];
    int len = debug_read_line(cmd_buf, sizeof(cmd_buf));
    if (len <= 0) {
        return;
    }

    /* 解析命令 */
    if (strncmp(cmd_buf, "PID ", 4) == 0) {
        cmd_set_pid(cmd_buf + 4);
    }
    else if (strcmp(cmd_buf, "GET_PID") == 0) {
        cmd_get_pid();
    }
    else if (strncmp(cmd_buf, "STEP ", 5) == 0) {
        cmd_step_response(cmd_buf + 5);
    }
    else if (strcmp(cmd_buf, "STATE") == 0) {
        cmd_get_state();
    }
    else if (strncmp(cmd_buf, "JOINT ", 6) == 0) {
        cmd_set_joint(cmd_buf + 6);
    }
    else if (strncmp(cmd_buf, "RAW ", 4) == 0) {
        cmd_send_raw(cmd_buf + 4);
    }
    else if (strcmp(cmd_buf, "HELP") == 0) {
        debug_println("Commands:");
        debug_println("  PID <Kp> <Kv> <Ki>  - Set PID params");
        debug_println("  GET_PID             - Get current PID");
        debug_println("  STEP <angle> <sec>  - Step response test");
        debug_println("  STATE               - Get current state");
        debug_println("  JOINT <0-5>         - Select joint");
    }
    else {
        debug_print("Unknown: ");
        debug_println(cmd_buf);
    }
}

void pid_tuner_update(void)
{
    if (g_state != PID_TUNER_STEP_RUNNING) {
        return;
    }

    uint32_t now = xTaskGetTickCount();
    uint32_t elapsed = now - g_step_data.start_tick;

    /* 检查是否超时 */
    if (elapsed >= g_step_data.duration_ms) {
        g_state = PID_TUNER_STEP_DONE;
        debug_println("END");
        return;
    }

    /* 采样当前位置 */
    if (g_step_data.sample_count < PID_TUNER_MAX_SAMPLES) {
        motor_state_t motor_state;
        motor_can_get_state(g_current_joint, &motor_state);

        g_step_data.sample_times[g_step_data.sample_count] = elapsed;
        g_step_data.samples[g_step_data.sample_count] = motor_state.current_angle;
        g_step_data.sample_count++;

        /* 输出采样数据: time,value */
        char buf[32];
        snprintf(buf, sizeof(buf), "%.3f,%.3f",
                 (float)elapsed / 1000.0f,
                 motor_state.current_angle);
        debug_println(buf);
    }
}

pid_tuner_state_t pid_tuner_get_state(void)
{
    return g_state;
}

void pid_tuner_set_joint(uint8_t joint_id)
{
    if (joint_id < 6) {
        g_current_joint = joint_id;
    }
}

uint8_t pid_tuner_get_joint(void)
{
    return g_current_joint;
}

/* ========== 命令处理函数 ========== */

/**
 * @brief 设置PID参数
 * @param args 格式: "<Kp> <Kv> <Ki>"
 */
static void cmd_set_pid(const char *args)
{
    uint16_t kp, kv, ki;

    if (sscanf(args, "%hu %hu %hu", &kp, &kv, &ki) != 3) {
        debug_println("ERROR: Invalid params, use: PID <Kp> <Kv> <Ki>");
        return;
    }

    /* 发送到电机 */
    int ret = motor_can_set_pid(g_current_joint, kp, kv, ki);
    if (ret != 0) {
        debug_println("ERROR: CAN send failed");
        return;
    }

    /* 更新缓存 */
    g_pid_params.kp = kp;
    g_pid_params.kv = kv;
    g_pid_params.ki = ki;

    char buf[64];
    snprintf(buf, sizeof(buf), "OK Kp=%u Kv=%u Ki=%u J=%u",
             kp, kv, ki, g_current_joint);
    debug_println(buf);
}

/**
 * @brief 获取当前PID参数
 */
static void cmd_get_pid(void)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%u,%u,%u",
             g_pid_params.kp, g_pid_params.kv, g_pid_params.ki);
    debug_println(buf);
}

/**
 * @brief 执行阶跃响应测试
 * @param args 格式: "<target_angle> <duration_sec>"
 */
static void cmd_step_response(const char *args)
{
    float target_angle, duration_sec;

    if (sscanf(args, "%f %f", &target_angle, &duration_sec) != 2) {
        debug_println("ERROR: Invalid params, use: STEP <angle> <seconds>");
        return;
    }

    if (duration_sec <= 0 || duration_sec > 30) {
        debug_println("ERROR: Duration must be 0-30 seconds");
        return;
    }

    /* 获取当前位置作为起始点 */
    motor_state_t motor_state;
    motor_can_get_state(g_current_joint, &motor_state);

    /* 初始化阶跃测试数据 */
    g_step_data.setpoint = target_angle;
    g_step_data.start_angle = motor_state.current_angle;
    g_step_data.duration_ms = (uint32_t)(duration_sec * 1000);
    g_step_data.start_tick = xTaskGetTickCount();
    g_step_data.sample_count = 0;

    /* 发送位置指令 */
    int ret = motor_can_send_position(g_current_joint, target_angle, 100.0f, false);
    if (ret != 0) {
        debug_println("ERROR: Motor command failed");
        return;
    }

    /* 开始采样 */
    g_state = PID_TUNER_STEP_RUNNING;

    /* 注意: 数据会在 pid_tuner_update() 中持续输出 */
    /* 格式: time,value\r\n */
    /* 结束时输出: END\r\n */
}

/**
 * @brief 获取当前状态
 */
static void cmd_get_state(void)
{
    motor_state_t motor_state;
    motor_can_get_state(g_current_joint, &motor_state);

    /* 输出格式: setpoint,actual,error,output */
    char buf[64];
    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,0.0",
             motor_state.target_angle,
             motor_state.current_angle,
             motor_state.target_angle - motor_state.current_angle);
    debug_println(buf);
}

/**
 * @brief 选择要调参的关节
 * @param args 格式: "<joint_id>"
 */
static void cmd_set_joint(const char *args)
{
    int joint_id;

    if (sscanf(args, "%d", &joint_id) != 1 || joint_id < 0 || joint_id > 5) {
        debug_println("ERROR: Invalid joint, use: JOINT <0-5>");
        return;
    }

    g_current_joint = (uint8_t)joint_id;

    char buf[32];
    snprintf(buf, sizeof(buf), "OK Joint=%d", joint_id);
    debug_println(buf);
}

/**
 * @brief 发送原始命令 (调试用)
 */
static void cmd_send_raw(const char *args)
{
    debug_print("RAW: ");
    debug_println(args);
    /* TODO: 可以扩展为发送任意CAN命令 */
}

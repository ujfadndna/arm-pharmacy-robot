/**
 * @file    cabinet_main.c
 * @brief   智能药柜主流程实现
 *
 * 串联: 扫描 → LLM → 执行
 */

#include "cabinet_main.h"
#include "cabinet_state.h"
#include "cabinet_config.h"
#include "cabinet_executor.h"
#include "medicine_db.h"
#include "llm_action.h"
#include "w800_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

/***************************************************************
 * 配置
 ***************************************************************/
#define SCAN_TIMEOUT_MS         5000    /* 扫描超时 */
#define LLM_TIMEOUT_MS          30000   /* LLM超时 */
#define STATE_BUF_SIZE          512
#define RESPONSE_BUF_SIZE       512

/***************************************************************
 * 全局变量
 ***************************************************************/
static cabinet_sys_state_t g_sys_state = CABINET_SYS_IDLE;
static bool g_scan_in_progress = false;
static bool g_scan_complete = false;
static SemaphoreHandle_t g_scan_sem = NULL;

/***************************************************************
 * 内部函数
 ***************************************************************/

/**
 * @brief 默认语音播报回调（串口打印）
 */
static void default_speak_callback(const char *text)
{
    /* TODO: 替换为实际的TTS调用 */
    printf("[SPEAK] %s\n", text);
}

/**
 * @brief 触发MaixCam2扫描
 */
static int trigger_scan(void)
{
    /* 清空当前药柜状态 */
    cabinet_clear_all();
    g_scan_complete = false;
    g_scan_in_progress = true;

    /* TODO: 通过UART发送扫描命令给MaixCam2 */
    /* 格式: "SCAN\n" */
    /* 这里假设MaixCam2收到后会自动扫描并返回TAG消息 */

    return 0;
}

/**
 * @brief 等待扫描完成
 */
static int wait_scan_complete(uint32_t timeout_ms)
{
    if (g_scan_sem == NULL) {
        g_scan_sem = xSemaphoreCreateBinary();
    }

    if (xSemaphoreTake(g_scan_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return 0;
    }
    return -1;  /* 超时 */
}

/***************************************************************
 * 公共API
 ***************************************************************/

int cabinet_system_init(void)
{
    /* 初始化各模块 */
    cabinet_config_init();
    cabinet_state_init();
    medicine_db_init();
    medicine_db_load_demo();
    cabinet_executor_init();

    /* 设置语音回调 */
    cabinet_executor_set_speak_callback(default_speak_callback);

    /* 初始化W800 */
    w800_init(NULL);

    g_sys_state = CABINET_SYS_IDLE;
    printf("[Cabinet] System initialized\n");

    return 0;
}

int cabinet_process_command(const char *user_cmd)
{
    int ret;
    char state_buf[STATE_BUF_SIZE];
    char response_buf[RESPONSE_BUF_SIZE];

    if (!user_cmd || user_cmd[0] == '\0') {
        return -1;
    }

    printf("[Cabinet] Processing: %s\n", user_cmd);

    /* ========== 1. 扫描药柜 ========== */
    g_sys_state = CABINET_SYS_SCANNING;
    printf("[Cabinet] Step 1: Scanning cabinet...\n");

    ret = trigger_scan();
    if (ret < 0) {
        g_sys_state = CABINET_SYS_ERROR;
        return -1;
    }

    ret = wait_scan_complete(SCAN_TIMEOUT_MS);
    if (ret < 0) {
        printf("[Cabinet] Scan timeout, using cached state\n");
        /* 超时不算失败，使用缓存的状态继续 */
    }

    /* ========== 2. 生成状态字符串 ========== */
    ret = cabinet_serialize_state(state_buf, sizeof(state_buf));
    if (ret <= 0) {
        printf("[Cabinet] Failed to serialize state\n");
        g_sys_state = CABINET_SYS_ERROR;
        return -2;
    }
    printf("[Cabinet] State: %s\n", state_buf);

    /* ========== 3. 调用LLM ========== */
    g_sys_state = CABINET_SYS_THINKING;
    printf("[Cabinet] Step 2: Calling LLM...\n");

    ret = w800_call_llm_organize(state_buf, user_cmd,
                                  response_buf, sizeof(response_buf));
    if (ret <= 0) {
        printf("[Cabinet] LLM call failed: %d\n", ret);
        g_sys_state = CABINET_SYS_ERROR;
        return -3;
    }
    printf("[Cabinet] LLM response: %s\n", response_buf);

    /* ========== 4. 解析动作序列 ========== */
    llm_action_sequence_t seq;
    ret = llm_parse_json(response_buf, &seq);
    if (ret < 0 || seq.count == 0) {
        printf("[Cabinet] Failed to parse actions\n");
        g_sys_state = CABINET_SYS_ERROR;
        return -4;
    }
    printf("[Cabinet] Parsed %d actions\n", seq.count);

    /* ========== 5. 执行动作 ========== */
    g_sys_state = CABINET_SYS_EXECUTING;
    printf("[Cabinet] Step 3: Executing actions...\n");

    ret = cabinet_executor_run(&seq);
    if (ret < 0) {
        printf("[Cabinet] Execution failed: %d\n", ret);
        g_sys_state = CABINET_SYS_ERROR;
        return -5;
    }

    /* ========== 完成 ========== */
    g_sys_state = CABINET_SYS_IDLE;
    printf("[Cabinet] Command completed successfully\n");

    return 0;
}

int cabinet_scan_only(void)
{
    int ret;

    g_sys_state = CABINET_SYS_SCANNING;

    ret = trigger_scan();
    if (ret < 0) {
        g_sys_state = CABINET_SYS_ERROR;
        return -1;
    }

    ret = wait_scan_complete(SCAN_TIMEOUT_MS);

    g_sys_state = CABINET_SYS_IDLE;
    return ret;
}

cabinet_sys_state_t cabinet_get_system_state(void)
{
    return g_sys_state;
}

void cabinet_stop(void)
{
    cabinet_executor_stop();
    g_sys_state = CABINET_SYS_IDLE;
}

void cabinet_handle_vision_message(const char *msg)
{
    if (!msg) return;

    /* 处理扫描开始/结束标记 */
    if (strncmp(msg, "SCAN_START", 10) == 0) {
        g_scan_in_progress = true;
        cabinet_clear_all();
        return;
    }

    if (strncmp(msg, "SCAN_END", 8) == 0) {
        g_scan_in_progress = false;
        g_scan_complete = true;
        if (g_scan_sem) {
            xSemaphoreGive(g_scan_sem);
        }
        return;
    }

    /* 处理TAG消息 */
    if (strncmp(msg, "TAG:", 4) == 0) {
        cabinet_parse_tag_message(msg);
    }
}

void cabinet_task_loop(void)
{
    /* 周期任务，目前为空 */
    /* 可以在这里添加状态监控、心跳等 */
}

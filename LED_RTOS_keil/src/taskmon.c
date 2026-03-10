/**
 ******************************************************************************
 * @file    taskmon.c
 * @brief   任务监控模块（FreeRTOS任务CPU占用监控）
 ******************************************************************************
 */

#include "taskmon.h"
#include <string.h>
#include <stdio.h>

/* ========== 内部状态 ========== */
#define TASKMON_MAX_TASKS 8

typedef struct {
    const char *name;
    uint32_t total_runtime;
    uint32_t last_runtime;
} task_stats_t;

static task_stats_t g_tasks[TASKMON_MAX_TASKS];
static uint8_t g_task_count = 0;
static bool g_initialized = false;

/* ========== 公共接口实现 ========== */

void taskmon_init(void)
{
    memset(g_tasks, 0, sizeof(g_tasks));
    g_task_count = 0;
    g_initialized = true;
}

void taskmon_register(const char *task_name)
{
    if (!g_initialized || g_task_count >= TASKMON_MAX_TASKS) return;
    
    g_tasks[g_task_count].name = task_name;
    g_task_count++;
}

void taskmon_update(const char *task_name, uint32_t runtime_us)
{
    if (!g_initialized) return;
    
    for (uint8_t i = 0; i < g_task_count; i++) {
        if (strcmp(g_tasks[i].name, task_name) == 0) {
            g_tasks[i].total_runtime += runtime_us;
            g_tasks[i].last_runtime = runtime_us;
            break;
        }
    }
}

void taskmon_reset(void)
{
    for (uint8_t i = 0; i < g_task_count; i++) {
        g_tasks[i].total_runtime = 0;
        g_tasks[i].last_runtime = 0;
    }
}

void taskmon_print_report(char *buf, uint32_t buf_size)
{
    if (!g_initialized || !buf || buf_size == 0) return;
    
    uint32_t offset = 0;
    offset += snprintf(buf + offset, buf_size - offset, "=== TASKMON Report ===\n");
    
    for (uint8_t i = 0; i < g_task_count && offset < buf_size; i++) {
        offset += snprintf(buf + offset, buf_size - offset,
            "%s: %lu us (total), %lu us (last)\n",
            g_tasks[i].name,
            g_tasks[i].total_runtime,
            g_tasks[i].last_runtime
        );
    }
}

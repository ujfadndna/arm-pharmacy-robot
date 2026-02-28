/**
 * @file task_monitor.c
 * @brief FreeRTOS任务监控模块实现
 */

#include "task_monitor.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* DWT寄存器 (ARM Cortex-M) */
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define SCB_DEMCR  (*(volatile uint32_t *)0xE000EDFC)

/* CPU频率 (200MHz) */
#define CPU_FREQ_MHZ  200

/* 内部任务记录 */
typedef struct {
    void *handle;               /* FreeRTOS任务句柄 */
    const char *name;           /* 任务名称 */
    uint32_t start_cycle;       /* 本次执行开始周期 */
    uint32_t total_cycles;      /* 累计周期数 */
    uint32_t switch_count;      /* 切换次数 */
    bool active;                /* 是否正在执行 */
} task_record_t;

static task_record_t g_tasks[TASKMON_MAX_TASKS];
static int g_task_count = 0;
static uint32_t g_last_update_cycle = 0;
static uint32_t g_total_cycles_period = 0;  /* 统计周期内总周期数 */
static uint32_t g_idle_cycles_period = 0;   /* 统计周期内空闲周期数 */

/**
 * @brief 初始化任务监控
 */
void taskmon_init(void)
{
    /* 清空任务列表 */
    memset(g_tasks, 0, sizeof(g_tasks));
    g_task_count = 0;

    /* 确保DWT已启用 (realtime_monitor可能已初始化) */
    SCB_DEMCR |= (1 << 24);  /* TRCENA */
    DWT_CTRL |= 1;           /* CYCCNTENA */

    g_last_update_cycle = DWT_CYCCNT;
}

/**
 * @brief 注册任务到监控列表
 */
int taskmon_register(void *task_handle, const char *name)
{
    if (g_task_count >= TASKMON_MAX_TASKS) {
        return -1;
    }

    int id = g_task_count++;
    g_tasks[id].handle = task_handle;
    g_tasks[id].name = name;
    g_tasks[id].start_cycle = 0;
    g_tasks[id].total_cycles = 0;
    g_tasks[id].switch_count = 0;
    g_tasks[id].active = false;

    return id;
}

/**
 * @brief 任务开始执行时调用
 */
void taskmon_task_enter(int task_id)
{
    if (task_id < 0 || task_id >= g_task_count) {
        return;
    }

    g_tasks[task_id].start_cycle = DWT_CYCCNT;
    g_tasks[task_id].active = true;
    g_tasks[task_id].switch_count++;
}

/**
 * @brief 任务让出CPU前调用
 */
void taskmon_task_exit(int task_id)
{
    if (task_id < 0 || task_id >= g_task_count) {
        return;
    }

    if (g_tasks[task_id].active) {
        uint32_t now = DWT_CYCCNT;
        uint32_t elapsed = now - g_tasks[task_id].start_cycle;
        g_tasks[task_id].total_cycles += elapsed;
        g_tasks[task_id].active = false;
    }
}

/**
 * @brief 更新统计 (每秒调用一次)
 */
void taskmon_update(void)
{
    uint32_t now = DWT_CYCCNT;
    g_total_cycles_period = now - g_last_update_cycle;
    g_last_update_cycle = now;

    /* 计算空闲周期 = 总周期 - 所有任务周期 */
    uint32_t used_cycles = 0;
    for (int i = 0; i < g_task_count; i++) {
        used_cycles += g_tasks[i].total_cycles;
        /* 重置累计周期 */
        g_tasks[i].total_cycles = 0;
        g_tasks[i].switch_count = 0;
    }

    g_idle_cycles_period = (g_total_cycles_period > used_cycles) ?
                           (g_total_cycles_period - used_cycles) : 0;
}

/**
 * @brief 获取任务统计信息
 */
int taskmon_get_stats(int task_id, task_stats_t *stats)
{
    if (task_id < 0 || task_id >= g_task_count || stats == NULL) {
        return -1;
    }

    task_record_t *t = &g_tasks[task_id];
    stats->name = t->name;
    stats->run_time_us = t->total_cycles / CPU_FREQ_MHZ;
    stats->switch_count = t->switch_count;

    /* CPU占用率 */
    if (g_total_cycles_period > 0) {
        stats->cpu_percent = (uint8_t)((t->total_cycles * 100ULL) / g_total_cycles_period);
    } else {
        stats->cpu_percent = 0;
    }

    /* 栈高水位 (需要FreeRTOS支持) */
#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
    if (t->handle != NULL) {
        stats->stack_high_water = uxTaskGetStackHighWaterMark((TaskHandle_t)t->handle) * sizeof(StackType_t);
    } else {
        stats->stack_high_water = 0;
    }
#else
    stats->stack_high_water = 0;  /* 未启用 */
#endif

    return 0;
}

/**
 * @brief 获取总CPU空闲率
 */
uint8_t taskmon_get_idle_percent(void)
{
    if (g_total_cycles_period == 0) {
        return 100;
    }
    return (uint8_t)((g_idle_cycles_period * 100ULL) / g_total_cycles_period);
}

/**
 * @brief 打印所有任务统计报告
 */
void taskmon_print_report(void)
{
    debug_println("");
    debug_println("=== Task Monitor Report ===");
    debug_println("");

    /* 系统概览 */
    debug_print("CPU Idle: ");
    debug_print_int(taskmon_get_idle_percent());
    debug_println("%");
    debug_print("Monitored Tasks: ");
    debug_print_int(g_task_count);
    debug_println("");
    debug_println("");

    /* 任务详情 */
    debug_println("Task          CPU%  Switches  Stack(B)");
    debug_println("------------  ----  --------  --------");

    for (int i = 0; i < g_task_count; i++) {
        task_stats_t stats;
        if (taskmon_get_stats(i, &stats) == 0) {
            /* 任务名 (12字符) */
            char name_buf[13];
            int len = 0;
            if (stats.name) {
                while (stats.name[len] && len < 12) {
                    name_buf[len] = stats.name[len];
                    len++;
                }
            }
            while (len < 12) {
                name_buf[len++] = ' ';
            }
            name_buf[12] = '\0';
            debug_print(name_buf);
            debug_print("  ");

            /* CPU% (4字符) */
            debug_print_int(stats.cpu_percent);
            debug_print("%");
            if (stats.cpu_percent < 10) debug_print("  ");
            else if (stats.cpu_percent < 100) debug_print(" ");
            debug_print("  ");

            /* Switches (8字符) */
            debug_print_int(stats.switch_count);
            debug_print("        ");

            /* Stack (8字符) */
            if (stats.stack_high_water > 0) {
                debug_print_int(stats.stack_high_water);
            } else {
                debug_print("N/A");
            }
            debug_println("");
        }
    }

    debug_println("");
    debug_println("Note: Call 'taskmon' periodically for accurate stats");
}

/**
 * @brief 重置所有统计
 */
void taskmon_reset(void)
{
    for (int i = 0; i < g_task_count; i++) {
        g_tasks[i].total_cycles = 0;
        g_tasks[i].switch_count = 0;
    }
    g_last_update_cycle = DWT_CYCCNT;
    g_total_cycles_period = 0;
    g_idle_cycles_period = 0;
}

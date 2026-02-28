/**
 * @file task_monitor.h
 * @brief FreeRTOS任务监控模块 - 轻量级实现
 *
 * 功能:
 * - 任务CPU占用率统计
 * - 任务栈使用监控
 * - 任务切换计数
 *
 * 不依赖configUSE_TRACE_FACILITY，手动实现统计
 */

#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

/* 最大监控任务数 */
#define TASKMON_MAX_TASKS  8

/* 任务统计信息 */
typedef struct {
    const char *name;           /* 任务名称 */
    uint32_t run_time_us;       /* 累计运行时间 (微秒) */
    uint32_t switch_count;      /* 切换次数 */
    uint32_t stack_high_water;  /* 栈高水位 (字节) */
    uint8_t cpu_percent;        /* CPU占用率 (0-100) */
} task_stats_t;

/**
 * @brief 初始化任务监控
 */
void taskmon_init(void);

/**
 * @brief 注册任务到监控列表
 * @param task_handle FreeRTOS任务句柄
 * @param name 任务名称
 * @return 0=成功, -1=列表已满
 */
int taskmon_register(void *task_handle, const char *name);

/**
 * @brief 任务开始执行时调用 (在任务入口)
 * @param task_id 任务ID (注册时返回的索引)
 */
void taskmon_task_enter(int task_id);

/**
 * @brief 任务让出CPU前调用 (在vTaskDelay等之前)
 * @param task_id 任务ID
 */
void taskmon_task_exit(int task_id);

/**
 * @brief 更新统计 (每秒调用一次)
 */
void taskmon_update(void);

/**
 * @brief 获取任务统计信息
 * @param task_id 任务ID
 * @param stats 输出统计信息
 * @return 0=成功, -1=无效ID
 */
int taskmon_get_stats(int task_id, task_stats_t *stats);

/**
 * @brief 获取总CPU空闲率
 * @return 空闲率 (0-100)
 */
uint8_t taskmon_get_idle_percent(void);

/**
 * @brief 打印所有任务统计报告
 */
void taskmon_print_report(void);

/**
 * @brief 重置所有统计
 */
void taskmon_reset(void);

#endif /* TASK_MONITOR_H */

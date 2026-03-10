/**
 ******************************************************************************
 * @file    rtmon.c
 * @brief   实时性监控模块（运动控制周期监控）
 * @note    监控GPT0定时器的周期、抖动、deadline miss
 ******************************************************************************
 */

#include "rtmon.h"
#include <string.h>
#include <stdio.h>

/* ========== 内部状态 ========== */
static rtmon_stats_t g_stats;
static bool g_initialized = false;
static uint32_t g_wcet_start_tick = 0;

/* ========== 公共接口实现 ========== */

void rtmon_init(void)
{
    memset(&g_stats, 0, sizeof(g_stats));
    g_stats.min_period_us = 0xFFFFFFFF;
    g_stats.min_wcet_us = 0xFFFFFFFF;
    g_initialized = true;
}

void rtmon_tick(uint32_t period_us)
{
    if (!g_initialized) return;
    
    g_stats.tick_count++;
    
    /* 更新周期统计 */
    if (period_us > g_stats.max_period_us) {
        g_stats.max_period_us = period_us;
    }
    if (period_us < g_stats.min_period_us) {
        g_stats.min_period_us = period_us;
    }
    
    /* 计算抖动 */
    int32_t jitter = (int32_t)period_us - 5000;  /* 5ms = 5000us */
    if (jitter < 0) jitter = -jitter;
    if ((uint32_t)jitter > g_stats.max_jitter_us) {
        g_stats.max_jitter_us = (uint32_t)jitter;
    }
    
    /* Deadline miss检测（超过6ms） */
    if (period_us > 6000) {
        g_stats.deadline_miss_count++;
    }
}

void rtmon_wcet_start(void)
{
    /* TODO: 使用DWT_CYCCNT或SysTick获取精确时间戳 */
    g_wcet_start_tick = 0;
}

void rtmon_wcet_end(void)
{
    /* TODO: 计算WCET */
    uint32_t wcet_us = 0;
    
    if (wcet_us > g_stats.max_wcet_us) {
        g_stats.max_wcet_us = wcet_us;
    }
    if (wcet_us < g_stats.min_wcet_us) {
        g_stats.min_wcet_us = wcet_us;
    }
}

void rtmon_wcet_reset(void)
{
    g_stats.max_wcet_us = 0;
    g_stats.min_wcet_us = 0xFFFFFFFF;
}

void rtmon_reset(void)
{
    rtmon_init();
}

void rtmon_print_report(char *buf, uint32_t buf_size)
{
    if (!g_initialized || !buf || buf_size == 0) return;
    
    snprintf(buf, buf_size,
        "=== RTMON Report ===\n"
        "Ticks: %lu\n"
        "Period: %lu/%lu us (min/max)\n"
        "Jitter: %lu us (max)\n"
        "WCET: %lu/%lu us (min/max)\n"
        "Deadline Miss: %lu\n",
        g_stats.tick_count,
        g_stats.min_period_us, g_stats.max_period_us,
        g_stats.max_jitter_us,
        g_stats.min_wcet_us, g_stats.max_wcet_us,
        g_stats.deadline_miss_count
    );
}

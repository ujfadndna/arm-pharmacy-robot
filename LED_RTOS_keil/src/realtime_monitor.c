/**
 ******************************************************************************
 * @file    realtime_monitor.c
 * @brief   Real-time performance monitoring implementation
 * @note    Uses DWT CYCCNT for cycle-accurate timing on Cortex-M33
 *          RA6M5: 200MHz CPU, 1 cycle = 5ns
 ******************************************************************************
 */

#include "realtime_monitor.h"
#include "debug_uart.h"
#include <string.h>

/* RA6M5 CPU frequency */
#define CPU_FREQ_HZ         200000000UL
#define CPU_FREQ_MHZ        200UL
#define TARGET_PERIOD_US    5000UL      /* 5ms target period */

/* DWT register addresses (Cortex-M33) */
#define DWT_BASE            0xE0001000UL
#define DWT_CTRL            (*(volatile uint32_t *)(DWT_BASE + 0x000))
#define DWT_CYCCNT          (*(volatile uint32_t *)(DWT_BASE + 0x004))

/* CoreDebug register for trace enable */
#define COREDEBUG_BASE      0xE000EDF0UL
#define COREDEBUG_DEMCR     (*(volatile uint32_t *)(COREDEBUG_BASE + 0x00))

/* Bit definitions */
#define DEMCR_TRCENA        (1UL << 24)
#define DWT_CTRL_CYCCNTENA  (1UL << 0)

/* Internal state */
static volatile uint32_t g_last_cyccnt = 0;
static volatile uint32_t g_tick_count = 0;
static volatile uint32_t g_min_period_cycles = 0xFFFFFFFFUL;
static volatile uint32_t g_max_period_cycles = 0;
static volatile uint64_t g_total_cycles = 0;
static volatile bool g_initialized = false;

/* Deadline miss state */
static volatile uint32_t g_deadline_us = RTMON_DEFAULT_DEADLINE_US;
static volatile uint32_t g_miss_threshold = RTMON_DEFAULT_MISS_THRESHOLD;
static volatile uint32_t g_miss_count = 0;
static volatile uint32_t g_consecutive_miss = 0;
static rtmon_miss_callback_t g_miss_callback = NULL;

/* WCET measurement state */
static volatile uint32_t g_wcet_start_cyccnt = 0;
static volatile uint32_t g_wcet_max_cycles = 0;
static volatile uint32_t g_wcet_last_cycles = 0;
static volatile uint32_t g_wcet_count = 0;

/* Convert cycles to microseconds */
static inline uint32_t cycles_to_us(uint32_t cycles)
{
    return cycles / CPU_FREQ_MHZ;
}

void rtmon_init(void)
{
    /* Enable trace (required for DWT) */
    COREDEBUG_DEMCR |= DEMCR_TRCENA;

    /* Reset cycle counter */
    DWT_CYCCNT = 0;

    /* Enable cycle counter */
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;

    /* Reset statistics */
    rtmon_reset();

    g_initialized = true;
}

void rtmon_tick(void)
{
    if (!g_initialized) {
        return;
    }

    uint32_t current = DWT_CYCCNT;

    /* First tick: just record timestamp */
    if (g_tick_count == 0) {
        g_last_cyccnt = current;
        g_tick_count = 1;
        return;
    }

    /* Calculate period (handle overflow) */
    uint32_t delta = current - g_last_cyccnt;
    g_last_cyccnt = current;

    /* Update statistics */
    g_tick_count++;
    g_total_cycles += delta;

    if (delta < g_min_period_cycles) {
        g_min_period_cycles = delta;
    }
    if (delta > g_max_period_cycles) {
        g_max_period_cycles = delta;
    }

    /* Deadline miss detection */
    uint32_t period_us = cycles_to_us(delta);
    if (period_us > g_deadline_us) {
        g_miss_count++;
        g_consecutive_miss++;

        /* Trigger callback if threshold exceeded */
        if (g_consecutive_miss >= g_miss_threshold && g_miss_callback != NULL) {
            g_miss_callback(period_us, g_deadline_us);
        }
    } else {
        g_consecutive_miss = 0;
    }
}

void rtmon_get_stats(rtmon_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    stats->tick_count = g_tick_count;

    if (g_tick_count <= 1) {
        stats->min_period_us = 0;
        stats->max_period_us = 0;
        stats->avg_period_us = 0;
        stats->max_jitter_us = 0;
        return;
    }

    stats->min_period_us = cycles_to_us(g_min_period_cycles);
    stats->max_period_us = cycles_to_us(g_max_period_cycles);

    /* Average = total / (count - 1) since first tick has no delta */
    uint32_t avg_cycles = (uint32_t)(g_total_cycles / (g_tick_count - 1));
    stats->avg_period_us = cycles_to_us(avg_cycles);

    /* Jitter = max deviation from target */
    uint32_t target_cycles = TARGET_PERIOD_US * CPU_FREQ_MHZ;
    uint32_t jitter_high = 0;
    uint32_t jitter_low = 0;

    if (g_max_period_cycles > target_cycles) {
        jitter_high = g_max_period_cycles - target_cycles;
    }
    if (g_min_period_cycles < target_cycles) {
        jitter_low = target_cycles - g_min_period_cycles;
    }

    stats->max_jitter_us = cycles_to_us(jitter_high > jitter_low ? jitter_high : jitter_low);

    /* Deadline miss statistics */
    stats->deadline_us = g_deadline_us;
    stats->miss_count = g_miss_count;
    stats->consecutive_miss = g_consecutive_miss;
    stats->miss_threshold = g_miss_threshold;
}

void rtmon_reset(void)
{
    g_tick_count = 0;
    g_min_period_cycles = 0xFFFFFFFFUL;
    g_max_period_cycles = 0;
    g_total_cycles = 0;
    g_last_cyccnt = DWT_CYCCNT;
    /* Reset deadline miss counters */
    g_miss_count = 0;
    g_consecutive_miss = 0;
}

void rtmon_print_report(void)
{
    rtmon_stats_t stats;
    rtmon_get_stats(&stats);

    debug_println("");
    debug_println("[RTMON] Real-time Statistics:");
    debug_print("  Tick count: ");
    debug_print_int((int)stats.tick_count);
    debug_println("");

    debug_print("  Target period: ");
    debug_print_int((int)TARGET_PERIOD_US);
    debug_println(" us");

    debug_print("  Actual period: ");
    debug_print_int((int)stats.min_period_us);
    debug_print("-");
    debug_print_int((int)stats.max_period_us);
    debug_print(" us (avg: ");
    debug_print_int((int)stats.avg_period_us);
    debug_println(" us)");

    debug_print("  Max jitter: ");
    debug_print_int((int)stats.max_jitter_us);
    debug_println(" us");

    /* Deadline miss statistics */
    debug_print("  Deadline: ");
    debug_print_int((int)stats.deadline_us);
    debug_println(" us");

    debug_print("  Miss count: ");
    debug_print_int((int)stats.miss_count);
    debug_print(" (threshold: ");
    debug_print_int((int)stats.miss_threshold);
    debug_println(")");

    /* Status evaluation */
    debug_print("  Status: ");
    if (stats.consecutive_miss >= stats.miss_threshold) {
        debug_println("WARNING: Deadline violations detected!");
    } else if (stats.miss_count > 0) {
        debug_println("OK (sporadic misses)");
    } else if (stats.max_jitter_us < 100) {
        debug_println("OK (jitter < 100us)");
    } else if (stats.max_jitter_us < 500) {
        debug_println("WARNING (jitter 100-500us)");
    } else {
        debug_println("CRITICAL (jitter > 500us)");
    }

    /* WCET statistics */
    if (g_wcet_count > 0) {
        debug_println("");
        debug_println("[WCET] Execution Time:");
        debug_print("  motion_update: max=");
        debug_print_int((int)cycles_to_us(g_wcet_max_cycles));
        debug_print(" us, last=");
        debug_print_int((int)cycles_to_us(g_wcet_last_cycles));
        debug_print(" us (");
        debug_print_int((int)g_wcet_count);
        debug_println(" samples)");

        /* WCET vs deadline evaluation */
        uint32_t wcet_us = cycles_to_us(g_wcet_max_cycles);
        debug_print("  Budget: ");
        if (wcet_us < 1000) {
            debug_println("OK (< 1ms, plenty of margin)");
        } else if (wcet_us < 3000) {
            debug_println("OK (< 3ms, good margin)");
        } else if (wcet_us < 5000) {
            debug_println("WARNING (< 5ms, tight margin)");
        } else {
            debug_println("CRITICAL (>= 5ms, may miss deadline!)");
        }
    }

    debug_println("");
}

void rtmon_set_deadline(uint32_t deadline_us)
{
    g_deadline_us = deadline_us;
}

int rtmon_get_miss_count(void)
{
    return (int)g_miss_count;
}

bool rtmon_check_deadline_violation(void)
{
    return (g_consecutive_miss >= g_miss_threshold);
}

void rtmon_set_miss_callback(rtmon_miss_callback_t cb)
{
    g_miss_callback = cb;
}

/* ========== WCET Measurement ========== */

void rtmon_wcet_start(void)
{
    g_wcet_start_cyccnt = DWT_CYCCNT;
}

void rtmon_wcet_end(const char *tag)
{
    uint32_t end = DWT_CYCCNT;
    uint32_t delta = end - g_wcet_start_cyccnt;

    (void)tag;  /* Tag reserved for future multi-point support */

    g_wcet_last_cycles = delta;
    g_wcet_count++;

    if (delta > g_wcet_max_cycles) {
        g_wcet_max_cycles = delta;
    }
}

uint32_t rtmon_wcet_get_max(void)
{
    return cycles_to_us(g_wcet_max_cycles);
}

void rtmon_wcet_reset(void)
{
    g_wcet_max_cycles = 0;
    g_wcet_last_cycles = 0;
    g_wcet_count = 0;
}

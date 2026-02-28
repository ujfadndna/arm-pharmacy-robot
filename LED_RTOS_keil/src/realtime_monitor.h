/**
 ******************************************************************************
 * @file    realtime_monitor.h
 * @brief   Real-time performance monitoring using DWT cycle counter
 * @note    Monitors 5ms motion control timer jitter and latency
 *          RA6M5: 200MHz CPU, DWT CYCCNT for cycle-accurate timing
 ******************************************************************************
 */

#ifndef REALTIME_MONITOR_H_
#define REALTIME_MONITOR_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default configuration */
#define RTMON_DEFAULT_DEADLINE_US   6000    /* 6ms deadline (5ms period + 1ms margin) */
#define RTMON_DEFAULT_MISS_THRESHOLD  3     /* Consecutive miss threshold */

/* Statistics structure */
typedef struct {
    uint32_t tick_count;        /* Total tick count */
    uint32_t max_jitter_us;     /* Maximum jitter (microseconds) */
    uint32_t avg_period_us;     /* Average period (microseconds) */
    uint32_t min_period_us;     /* Minimum period */
    uint32_t max_period_us;     /* Maximum period */
    /* Deadline miss statistics */
    uint32_t deadline_us;       /* Current deadline setting */
    uint32_t miss_count;        /* Total deadline miss count */
    uint32_t consecutive_miss;  /* Current consecutive miss count */
    uint32_t miss_threshold;    /* Consecutive miss threshold */
} rtmon_stats_t;

/* Deadline miss callback type */
typedef void (*rtmon_miss_callback_t)(uint32_t actual_us, uint32_t deadline_us);

/**
 * @brief Initialize real-time monitor (enable DWT cycle counter)
 */
void rtmon_init(void);

/**
 * @brief Record timestamp in timer callback
 * @note  Call this at the START of motion_timer_callback()
 */
void rtmon_tick(void);

/**
 * @brief Get statistics
 * @param stats Pointer to stats structure
 */
void rtmon_get_stats(rtmon_stats_t *stats);

/**
 * @brief Reset all statistics
 */
void rtmon_reset(void);

/**
 * @brief Print statistics report to debug UART
 */
void rtmon_print_report(void);

/**
 * @brief Set deadline threshold
 * @param deadline_us Deadline in microseconds
 */
void rtmon_set_deadline(uint32_t deadline_us);

/**
 * @brief Get total deadline miss count
 * @return Number of deadline misses
 */
int rtmon_get_miss_count(void);

/**
 * @brief Check if deadline violation occurred (consecutive misses >= threshold)
 * @return true if violation detected
 */
bool rtmon_check_deadline_violation(void);

/**
 * @brief Set callback for deadline miss events
 * @param cb Callback function (NULL to disable)
 */
void rtmon_set_miss_callback(rtmon_miss_callback_t cb);

/* WCET (Worst-Case Execution Time) measurement */

/**
 * @brief Start WCET measurement
 * @note  Call before the code section to measure
 */
void rtmon_wcet_start(void);

/**
 * @brief End WCET measurement and record result
 * @param tag Identifier for this measurement point (e.g., "motion")
 * @note  Call after the code section to measure
 */
void rtmon_wcet_end(const char *tag);

/**
 * @brief Get maximum recorded execution time
 * @return Maximum execution time in microseconds
 */
uint32_t rtmon_wcet_get_max(void);

/**
 * @brief Reset WCET statistics
 */
void rtmon_wcet_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* REALTIME_MONITOR_H_ */

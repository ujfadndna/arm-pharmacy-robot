/**
 ******************************************************************************
 * @file    rtmon.h
 * @brief   实时性监控模块接口
 ******************************************************************************
 */

#ifndef RTMON_H_
#define RTMON_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 统计数据结构 ========== */
typedef struct {
    uint32_t tick_count;            /* 总tick数 */
    uint32_t max_period_us;         /* 最大周期(us) */
    uint32_t min_period_us;         /* 最小周期(us) */
    uint32_t max_jitter_us;         /* 最大抖动(us) */
    uint32_t max_wcet_us;           /* 最坏执行时间(us) */
    uint32_t min_wcet_us;           /* 最佳执行时间(us) */
    uint32_t deadline_miss_count;   /* Deadline miss次数 */
} rtmon_stats_t;

/* ========== 公共接口 ========== */

/** @brief 初始化实时性监控 */
void rtmon_init(void);

/** @brief 记录一次tick（在定时器中断中调用） */
void rtmon_tick(uint32_t period_us);

/** @brief 开始WCET测量 */
void rtmon_wcet_start(void);

/** @brief 结束WCET测量 */
void rtmon_wcet_end(void);

/** @brief 重置WCET统计 */
void rtmon_wcet_reset(void);

/** @brief 重置所有统计 */
void rtmon_reset(void);

/** @brief 打印报告 */
void rtmon_print_report(char *buf, uint32_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* RTMON_H_ */

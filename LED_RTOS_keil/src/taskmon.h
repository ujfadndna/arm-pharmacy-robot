/**
 ******************************************************************************
 * @file    taskmon.h
 * @brief   任务监控模块接口
 ******************************************************************************
 */

#ifndef TASKMON_H_
#define TASKMON_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 公共接口 ========== */

/** @brief 初始化任务监控 */
void taskmon_init(void);

/** @brief 注册任务 */
void taskmon_register(const char *task_name);

/** @brief 更新任务运行时间 */
void taskmon_update(const char *task_name, uint32_t runtime_us);

/** @brief 重置统计 */
void taskmon_reset(void);

/** @brief 打印报告 */
void taskmon_print_report(char *buf, uint32_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* TASKMON_H_ */

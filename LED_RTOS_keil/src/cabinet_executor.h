/**
 * @file    cabinet_executor.h
 * @brief   药柜动作执行器 - 把LLM动作转成机械臂运动
 */

#ifndef CABINET_EXECUTOR_H_
#define CABINET_EXECUTOR_H_

#include "llm_action.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************
 * 执行状态
 ***************************************************************/
typedef enum {
    EXEC_IDLE = 0,          /* 空闲 */
    EXEC_RUNNING,           /* 执行中 */
    EXEC_PAUSED,            /* 暂停 */
    EXEC_ERROR,             /* 出错 */
    EXEC_DONE               /* 完成 */
} exec_state_t;

/***************************************************************
 * 回调函数类型
 ***************************************************************/
typedef void (*exec_speak_callback_t)(const char *text);

/***************************************************************
 * API 函数
 ***************************************************************/

/**
 * @brief 初始化执行器
 */
void cabinet_executor_init(void);

/**
 * @brief 设置语音播报回调
 */
void cabinet_executor_set_speak_callback(exec_speak_callback_t cb);

/**
 * @brief 执行动作序列（阻塞）
 * @param seq 动作序列
 * @return 0=成功, 负数=错误
 */
int cabinet_executor_run(llm_action_sequence_t *seq);

/**
 * @brief 执行单个动作
 * @param action 动作
 * @return 0=成功, 负数=错误
 */
int cabinet_executor_step(llm_action_t *action);

/**
 * @brief 获取当前执行状态
 */
exec_state_t cabinet_executor_get_state(void);

/**
 * @brief 停止执行
 */
void cabinet_executor_stop(void);

/**
 * @brief 暂停执行
 */
void cabinet_executor_pause(void);

/**
 * @brief 恢复执行
 */
void cabinet_executor_resume(void);

#ifdef __cplusplus
}
#endif

#endif /* CABINET_EXECUTOR_H_ */

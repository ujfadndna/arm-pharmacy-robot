/**
 * @file    cabinet_main.h
 * @brief   智能药柜主流程 - 串联扫描→LLM→执行
 */

#ifndef CABINET_MAIN_H_
#define CABINET_MAIN_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************
 * 系统状态
 ***************************************************************/
typedef enum {
    CABINET_SYS_IDLE = 0,       /* 空闲，等待指令 */
    CABINET_SYS_SCANNING,       /* 扫描药柜中 */
    CABINET_SYS_THINKING,       /* LLM思考中 */
    CABINET_SYS_EXECUTING,      /* 执行动作中 */
    CABINET_SYS_ERROR           /* 出错 */
} cabinet_sys_state_t;

/***************************************************************
 * API 函数
 ***************************************************************/

/**
 * @brief 初始化智能药柜系统
 * @return 0=成功
 */
int cabinet_system_init(void);

/**
 * @brief 处理用户指令（主入口）
 * @param user_cmd 用户指令，如 "把快过期的药放前面"
 * @return 0=成功, 负数=错误
 *
 * 完整流程:
 * 1. 触发MaixCam2扫描药柜
 * 2. 接收TAG消息，更新药柜状态
 * 3. 生成状态字符串，发送给W800
 * 4. W800调用LLM，返回动作序列
 * 5. 解析动作序列
 * 6. 执行动作
 */
int cabinet_process_command(const char *user_cmd);

/**
 * @brief 仅扫描药柜（不调用LLM）
 * @return 0=成功
 */
int cabinet_scan_only(void);

/**
 * @brief 获取当前系统状态
 */
cabinet_sys_state_t cabinet_get_system_state(void);

/**
 * @brief 停止当前操作
 */
void cabinet_stop(void);

/**
 * @brief 处理MaixCam2发来的消息
 * @param msg 消息字符串
 *
 * 在UART接收回调中调用
 */
void cabinet_handle_vision_message(const char *msg);

/**
 * @brief 周期任务（在FreeRTOS任务中调用）
 */
void cabinet_task_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* CABINET_MAIN_H_ */

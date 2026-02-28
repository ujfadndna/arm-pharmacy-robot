/**
 * @file    pid_tuner.h
 * @brief   PID调参命令处理模块 - 配合MCP Server使用
 * @note    通过UART4接收AI发来的调参命令
 */

#ifndef PID_TUNER_H_
#define PID_TUNER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置 ========== */
#define PID_TUNER_MAX_SAMPLES   500     /* 阶跃响应最大采样点数 */
#define PID_TUNER_SAMPLE_PERIOD 10      /* 采样周期(ms) */

/* ========== 调参状态 ========== */
typedef enum {
    PID_TUNER_IDLE,         /* 空闲 */
    PID_TUNER_STEP_RUNNING, /* 阶跃响应测试中 */
    PID_TUNER_STEP_DONE,    /* 阶跃响应完成 */
} pid_tuner_state_t;

/* ========== 当前PID参数 ========== */
typedef struct {
    uint16_t kp;    /* 位置环 Kp */
    uint16_t kv;    /* 速度环 Kp */
    uint16_t ki;    /* 速度环 Ki */
} pid_params_t;

/* ========== 系统状态 ========== */
typedef struct {
    float setpoint;     /* 目标位置(度) */
    float actual;       /* 实际位置(度) */
    float error;        /* 位置误差(度) */
    float output;       /* 输出(预留) */
} pid_state_t;

/* ========== 公共接口 ========== */

/**
 * @brief 初始化PID调参模块
 */
void pid_tuner_init(void);

/**
 * @brief 处理串口命令 (在主循环中调用)
 * @note 检查debug_uart是否有新命令，解析并执行
 */
void pid_tuner_process_command(void);

/**
 * @brief 周期性更新 (在定时器回调中调用)
 * @note 用于阶跃响应数据采集，每10ms调用一次
 */
void pid_tuner_update(void);

/**
 * @brief 获取当前状态
 */
pid_tuner_state_t pid_tuner_get_state(void);

/**
 * @brief 设置要调参的关节ID
 * @param joint_id 关节ID (0-5)
 */
void pid_tuner_set_joint(uint8_t joint_id);

/**
 * @brief 获取当前调参的关节ID
 */
uint8_t pid_tuner_get_joint(void);

#ifdef __cplusplus
}
#endif

#endif /* PID_TUNER_H_ */

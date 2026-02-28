/**
 * @file    visual_servo.h
 * @brief   视觉伺服模块 - 简化版
 *
 * 功能：
 * 1. 请求MaixCam2进行精定位
 * 2. 获取目标相对于夹爪的偏差
 * 3. 微调机械臂位置
 */

#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include <stdint.h>
#include <stdbool.h>

/***************************************************************
 * 配置参数
 ***************************************************************/
#define VS_MAX_ITERATIONS       3       /* 最大微调次数 */
#define VS_POSITION_THRESHOLD   5.0f    /* 位置误差阈值 (mm) */
#define VS_ALIGN_TIMEOUT_MS     2000    /* 精定位超时 (ms) */
#define VS_CORRECTION_GAIN      0.6f    /* 修正增益 (0-1) */

/***************************************************************
 * 数据结构
 ***************************************************************/

/* 精定位结果 */
typedef struct {
    bool success;       /* 是否成功 */
    float dx;           /* X方向偏差 (mm) */
    float dy;           /* Y方向偏差 (mm) */
} align_result_t;

/* 视觉伺服状态 */
typedef enum {
    VS_IDLE,
    VS_WAITING_ALIGN,
    VS_DONE,
    VS_ERROR
} vs_state_t;

/***************************************************************
 * API函数
 ***************************************************************/

/**
 * @brief 初始化视觉伺服模块
 */
void visual_servo_init(void);

/**
 * @brief 请求精定位
 * @param target_id 目标AprilTag ID
 * @return 0成功，<0失败
 */
int visual_servo_request_align(int target_id);

/**
 * @brief 等待精定位结果
 * @param result 输出结果
 * @param timeout_ms 超时时间
 * @return 0成功，<0失败/超时
 */
int visual_servo_wait_align(align_result_t *result, uint32_t timeout_ms);

/**
 * @brief 执行视觉伺服微调（阻塞）
 *
 * 流程：
 * 1. 请求精定位
 * 2. 获取偏差
 * 3. 微调位置
 * 4. 重复直到偏差足够小或达到最大次数
 *
 * @param target_id 目标AprilTag ID
 * @param current_x 当前X坐标（会被更新）
 * @param current_y 当前Y坐标（会被更新）
 * @return 0成功，<0失败
 */
int visual_servo_align(int target_id, float *current_x, float *current_y);

/**
 * @brief 处理MaixCam2返回的消息
 * @param msg 消息字符串
 * @note 由UART接收任务调用
 */
void visual_servo_handle_message(const char *msg);

/**
 * @brief 获取当前状态
 */
vs_state_t visual_servo_get_state(void);

#endif /* VISUAL_SERVO_H */

/**
 ******************************************************************************
 * @file    motor_test.h
 * @brief   单电机测试工具头文件
 ******************************************************************************
 */

#ifndef MOTOR_TEST_H_
#define MOTOR_TEST_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行所有测试
 * @note 包含7个测试：使能/失能、位置控制、速度控制、位置查询、
 *       温度查询、多轴同步、参数配置
 */
void motor_test_run_all(void);

/**
 * @brief 运行单个测试
 * @param test_id 测试ID (1-7)
 *        1: 使能/失能
 *        2: 位置控制
 *        3: 速度控制
 *        4: 位置查询
 *        5: 温度查询
 *        6: 多轴同步
 *        7: 参数配置
 */
void motor_test_run_single(uint8_t test_id);

/**
 * @brief 手动位置控制测试
 * @param joint_id 关节ID (0-5)
 * @param target_angle 目标角度(度)
 */
void motor_test_move_to(uint8_t joint_id, float target_angle);

/**
 * @brief 超时检测测试
 * @note 连续检测10秒，报告超时的电机
 */
void motor_test_timeout_check(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TEST_H_ */

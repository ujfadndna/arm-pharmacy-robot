/**
 * @file    motion_controller.h
 * @brief   运动控制器 - 封装IK、轨迹规划、电机控制
 */

#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 状态定义 ========== */
typedef enum {
    MOTION_IDLE = 0,        /* 空闲 */
    MOTION_PLANNING,        /* 计算IK和轨迹 */
    MOTION_EXECUTING,       /* 执行中 */
    MOTION_DONE,            /* 完成 */
    MOTION_ERROR            /* 错误 */
} motion_state_t;

/* ========== API ========== */

/**
 * @brief 初始化运动控制器
 */
void motion_init(void);

/**
 * @brief 移动到笛卡尔坐标 (末端朝下姿态)
 * @param x,y,z 目标位置 (mm)
 * @return 0=成功开始, <0=错误
 *         -1=参数错误, -2=IK无解, -3=轨迹规划失败, -4=电机启动失败, -5=关节超限
 */
int motion_move_to_xyz(float x, float y, float z);

/**
 * @brief 移动到完整位姿
 * @param x,y,z 目标位置 (mm)
 * @param roll,pitch,yaw 目标姿态 (rad)
 * @return 0=成功开始, <0=错误
 *         -1=参数错误, -2=IK无解, -3=轨迹规划失败, -4=电机启动失败, -5=关节超限
 */
int motion_move_to_pose(float x, float y, float z,
                        float roll, float pitch, float yaw);

/**
 * @brief 直接移动到关节角度
 * @param joints 6个关节角度 (度)
 * @return 0=成功开始, <0=错误
 *         -1=参数错误, -2=关节超限, -3=轨迹规划失败, -4=电机启动失败, -5=关节超限
 */
int motion_move_to_joints(const float joints[6]);

/**
 * @brief 周期更新 (5ms调用一次，200Hz)
 * @return 当前状态
 */
motion_state_t motion_update(void);

/**
 * @brief 获取当前状态
 */
motion_state_t motion_get_state(void);

/**
 * @brief 停止运动
 */
void motion_stop(void);

/**
 * @brief 设置当前关节角度 (用于初始化或同步)
 */
void motion_set_current_joints(const float joints[6]);

/**
 * @brief 设置关节偏移量 (用于home校准)
 * @param offsets 6个关节的偏移量 (度)
 * @note 软件角度 = 电机角度 + 偏移量
 *       执行home命令后，电机清零，偏移量设为当前软件角度
 */
void motion_set_joint_offsets(const float offsets[6]);

/**
 * @brief 获取当前关节角度
 */
void motion_get_current_joints(float joints[6]);

/**
 * @brief 仅测试IK是否有解 (不执行运动)
 * @param x,y,z 目标偏移量 (mm)
 * @return 0=有解, <0=无解
 */
int motion_test_ik(float x, float y, float z);

/**
 * @brief 清除堵转保护并重置堵转计数器
 * @param joint_index 关节索引 (0-5), 或 0xFF 清除所有关节
 * @return 0=成功, <0=错误
 */
int motion_clear_stall(uint8_t joint_index);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_H_ */

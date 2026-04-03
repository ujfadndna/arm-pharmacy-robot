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
 * @brief 移动到笛卡尔绝对坐标 (基坐标系, 末端姿态保持当前)
 * @param x,y,z 目标绝对位置 (mm)
 * @return 0=成功开始, <0=错误
 *         -1=参数错误, -2=IK无解, -3=轨迹规划失败, -4=电机启动失败, -5=关节超限
 *         -6=USB未连接, -7=等待到位超时
 */
int motion_move_to_xyz(float x, float y, float z);

/**
 * @brief 笛卡尔相对位移 (基坐标系, 末端姿态保持当前)
 * @param dx,dy,dz 相对位移 (mm)
 * @return 0=成功开始, <0=错误
 *         -1=参数错误, -2=IK无解, -3=轨迹规划失败, -4=电机启动失败, -5=关节超限
 *         -6=USB未连接, -7=等待到位超时
 */
int motion_move_by_xyz(float dx, float dy, float dz);

/**
 * @brief 移动到完整位姿
 * @param x,y,z 目标位置 (mm)
 * @param roll,pitch,yaw 目标姿态 (rad)
 * @return 0=成功开始, <0=错误
 *         -1=参数错误, -2=IK无解, -3=轨迹规划失败, -4=电机启动失败, -5=关节超限
 *         -6=USB未连接, -7=等待到位超时
 */
int motion_move_to_pose(float x, float y, float z,
                        float roll, float pitch, float yaw);

/**
 * @brief 直接移动到关节角度
 * @param joints 6个关节角度 (度)
 * @return 0=成功开始, <0=错误
 *         -1=参数错误, -2=关节超限, -3=轨迹规划失败, -4=电机启动失败, -5=关节超限
 *         -6=USB未连接, -7=等待到位超时
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
 * @brief 从USB查询实际关节位置并同步到内部状态
 *        在绕过motion_controller直接操作Dummy ARM后调用
 */
void motion_sync_from_usb(void);

/**
 * @brief 清除堵转保护并重置堵转计数器
 * @param joint_index 关节索引 (0-5), 或 0xFF 清除所有关节
 * @return 0=成功, <0=错误
 */
int motion_clear_stall(uint8_t joint_index);

/**
 * @brief 非阻塞版本：发送运动指令后立即返回（state=EXECUTING）
 * @note  客户端通过 motion_get_state() 轮询确认完成（DONE/ERROR）
 *        适用于 TCP 命令场景，避免长时间阻塞导致超时
 */
int motion_start_xyz(float x, float y, float z);
int motion_start_by_xyz(float dx, float dy, float dz);
int motion_start_joints(const float joints[6]);

/**
 * @brief 兼容接口: 相对位移
 * @note 等价于 motion_move_by_xyz(dx, dy, dz)
 */
int motion_move_relative(float dx, float dy, float dz);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_H_ */

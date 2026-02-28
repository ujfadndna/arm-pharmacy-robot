/**
 ******************************************************************************
 * @file    kinematics.h
 * @brief   6-DOF机械臂逆运动学求解器 (移植自ZERO项目)
 * @note    独立模块,不依赖硬件驱动
 ******************************************************************************
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 ========== */
#define KIN_MAX_JOINT_NUM       6       /* 关节数量 */
#define KIN_SOLUTION_NUM        4       /* IK解的数量 */
#define KIN_ERROR_RANGE         1e-4f   /* 浮点误差容限 */

/* ========== 数据结构 ========== */

/**
 * @brief 3D位置坐标
 */
typedef struct {
    float x;
    float y;
    float z;
} kin_position_t;

/**
 * @brief 关节限位配置
 */
typedef struct {
    float min_angle;    /* 最小角度(度) */
    float max_angle;    /* 最大角度(度) */
} kin_joint_limit_t;

/**
 * @brief 逆运动学求解器上下文
 */
typedef struct {
    float T_target[4][4];                           /* 目标齐次变换矩阵 */
    float solutions[KIN_SOLUTION_NUM][KIN_MAX_JOINT_NUM]; /* IK解集合(度) */
    uint32_t invalid_mask;                          /* 无效解掩码(bit=1表示无效) */
    float current_angles[KIN_MAX_JOINT_NUM];        /* 当前关节角度(度),用于选最优解 */
    kin_joint_limit_t joint_limits[KIN_MAX_JOINT_NUM]; /* 关节限位 */
} kin_solver_t;

/* ========== 公共接口 ========== */

/**
 * @brief 初始化IK求解器
 * @param solver 求解器上下文
 * @param joint_limits 关节限位数组(6个元素),可传NULL使用默认值
 */
void kin_solver_init(kin_solver_t *solver, const kin_joint_limit_t *joint_limits);

/**
 * @brief 求解逆运动学
 * @param solver 求解器上下文
 * @param T_target 目标齐次变换矩阵(4×4,行优先存储)
 * @param result 输出最优解的关节角度(度),6个元素
 * @return 0-成功, -1-无有效解
 * @note 会自动选择与current_angles最接近的解
 */
int kin_inverse_kinematics(kin_solver_t *solver, const float T_target[4][4], float result[KIN_MAX_JOINT_NUM]);

/**
 * @brief 更新当前关节角度(用于最优解选择)
 * @param solver 求解器上下文
 * @param angles 当前关节角度(度),6个元素
 */
void kin_update_current_angles(kin_solver_t *solver, const float angles[KIN_MAX_JOINT_NUM]);

/**
 * @brief 更新单个关节角度
 * @param solver 求解器上下文
 * @param joint_id 关节ID(0-5)
 * @param angle 角度(度)
 */
void kin_update_joint_angle(kin_solver_t *solver, uint32_t joint_id, float angle);

/**
 * @brief 根据相对位移计算新的T矩阵
 * @param T_in 输入T矩阵
 * @param T_out 输出T矩阵
 * @param delta_pos 相对位移(仅x,y,z平移)
 */
void kin_calc_T_with_offset(const float T_in[4][4], float T_out[4][4], const kin_position_t *delta_pos);

/**
 * @brief 获取所有IK解(调试用)
 * @param solver 求解器上下文
 * @param solutions 输出缓冲区,至少4×6大小
 * @param valid_mask 有效性掩码(bit=0有效)
 * @return 有效解数量
 */
int kin_get_all_solutions(kin_solver_t *solver, float solutions[KIN_SOLUTION_NUM][KIN_MAX_JOINT_NUM], uint32_t *valid_mask);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H_ */

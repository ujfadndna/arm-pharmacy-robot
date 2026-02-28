/**
 ******************************************************************************
 * @file    kinematics.c
 * @brief   6-DOF机械臂逆运动学求解器实现
 * @note    移植自ZERO机械臂项目,使用代数法解析求解
 ******************************************************************************
 */

#include "kinematics.h"
#include "kin_math.h"      /* 快速数学库 */
#include <string.h>
#include <stdio.h>

/* 定义M_PI (某些编译器不自带) */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ========== DH参数 (ZERO机械臂) ========== */
/* DH[i][0]=a, DH[i][1]=alpha, DH[i][2]=d, DH[i][3]=theta */
static const float DH_PARAMS[6][4] = {
    {0,        0,          0,          M_PI/2},
    {0,        M_PI/2,      0,         M_PI/2},
    {200,      M_PI,        0,         -M_PI/2},
    {47.63f,   -M_PI/2,     -184.5,    0},
    {0,        M_PI/2,      0,         M_PI/2},
    {0,        M_PI/2,        0,         0}
};

/* 简化访问 */
#define a2 DH_PARAMS[2][0]
#define a3 DH_PARAMS[3][0]
#define d4 DH_PARAMS[3][2]

/* 关节选择权重(用于最优解计算) */
static const float JOINT_WEIGHTS[KIN_MAX_JOINT_NUM] = {5, 3, 3, 1, 1, 1};

/* 默认关节限位(度) */
static const kin_joint_limit_t DEFAULT_LIMITS[KIN_MAX_JOINT_NUM] = {
    {0, 360},   /* Joint 1 */
    {0, 360},   /* Joint 2 */
    {0, 360},   /* Joint 3 */
    {0, 360},   /* Joint 4 */
    {0, 360},   /* Joint 5 */
    {0, 360}    /* Joint 6 */
};

/* ========== 内部函数声明 ========== */
static void solve_theta3(kin_solver_t *solver);
static void solve_theta2(kin_solver_t *solver);
static void solve_theta1(kin_solver_t *solver);
static void solve_theta5(kin_solver_t *solver);
static void solve_theta4(kin_solver_t *solver);
static void solve_theta6(kin_solver_t *solver);
static void radians_to_degrees(kin_solver_t *solver);
static void map_joint_limits(kin_solver_t *solver);
static int get_optimal_solution(kin_solver_t *solver, float result[KIN_MAX_JOINT_NUM]);
static float rad_to_deg_0_360(float rad);

/* ========== 公共接口实现 ========== */

void kin_solver_init(kin_solver_t *solver, const kin_joint_limit_t *joint_limits)
{
    memset(solver, 0, sizeof(kin_solver_t));
    
    if (joint_limits) {
        memcpy(solver->joint_limits, joint_limits, sizeof(kin_joint_limit_t) * KIN_MAX_JOINT_NUM);
    } else {
        memcpy(solver->joint_limits, DEFAULT_LIMITS, sizeof(kin_joint_limit_t) * KIN_MAX_JOINT_NUM);
    }
}

int kin_inverse_kinematics(kin_solver_t *solver, const float T_target[4][4], float result[KIN_MAX_JOINT_NUM])
{
    /* 复制目标矩阵 */
    memcpy(solver->T_target, T_target, sizeof(float) * 16);
    
    /* 按依赖顺序求解 */
    solve_theta3(solver);
    solve_theta2(solver);
    solve_theta1(solver);
    solve_theta5(solver);
    solve_theta4(solver);
    solve_theta6(solver);
    solver->invalid_mask = 0;
    
    /* 弧度→度 */
    radians_to_degrees(solver);
    
    /* 关节限位检查 */
    map_joint_limits(solver);
    
    /* 选择最优解 */
    return get_optimal_solution(solver, result);
}

void kin_update_current_angles(kin_solver_t *solver, const float angles[KIN_MAX_JOINT_NUM])
{
    memcpy(solver->current_angles, angles, sizeof(float) * KIN_MAX_JOINT_NUM);
}

void kin_update_joint_angle(kin_solver_t *solver, uint32_t joint_id, float angle)
{
    if (joint_id < KIN_MAX_JOINT_NUM) {
        solver->current_angles[joint_id] = angle;
    }
}

void kin_calc_T_with_offset(const float T_in[4][4], float T_out[4][4], const kin_position_t *delta_pos)
{
    memcpy(T_out, T_in, sizeof(float) * 16);
    T_out[0][3] += delta_pos->x;
    T_out[1][3] += delta_pos->y;
    T_out[2][3] += delta_pos->z;
}

int kin_get_all_solutions(kin_solver_t *solver, float solutions[KIN_SOLUTION_NUM][KIN_MAX_JOINT_NUM], uint32_t *valid_mask)
{
    memcpy(solutions, solver->solutions, sizeof(float) * KIN_SOLUTION_NUM * KIN_MAX_JOINT_NUM);
    *valid_mask = solver->invalid_mask;
    
    int count = 0;
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        if (!(solver->invalid_mask & (1 << i))) {
            count++;
        }
    }
    return count;
}

/* ========== IK求解算法 ========== */

static void solve_theta3(kin_solver_t *solver)
{
    float px = solver->T_target[0][3];
    float py = solver->T_target[1][3];
    float pz = solver->T_target[2][3];

    /* 预计算DH参数的幂次 (常量，可考虑移到初始化) */
    float a2_2 = a2 * a2;
    float a3_2 = a3 * a3;
    float d4_2 = d4 * d4;
    float a2_4 = a2_2 * a2_2;
    float a3_4 = a3_2 * a3_2;
    float d4_4 = d4_2 * d4_2;

    float _2_a2_d4 = 2 * a2 * d4;
    float _2_pow_a2_2 = 2 * a2_2;
    float _2_pow_a3_2 = 2 * a3_2;
    float _2_pow_d4_2 = 2 * d4_2;
    float const_eq1 = -a2_4 + _2_pow_a2_2 * (a3_2 + d4_2)
                - a3_4 - 2*a3_2*d4_2 - d4_4;
    float const_eq2 = -a2_2 + 2*a2*a3 - a3_2 - d4_2;

    float pow_px_2 = px * px;
    float pow_py_2 = py * py;
    float pow_pz_2 = pz * pz;
    float pow_distance_2 = pow_px_2 + pow_py_2 + pow_pz_2;

    float eq1 = (const_eq1 + _2_pow_a2_2*pow_distance_2
        + _2_pow_a3_2*pow_distance_2 + _2_pow_d4_2*pow_distance_2
        - pow_px_2*pow_px_2 - pow_py_2*pow_py_2 - pow_pz_2*pow_pz_2
        - 2*pow_px_2*(pow_py_2 + pow_pz_2) - 2*pow_py_2*pow_pz_2);

    float sqrt_eq1 = kin_sqrtf(eq1);
    float u_theta3_1 = -(_2_a2_d4 + sqrt_eq1) / (const_eq2 + pow_distance_2);
    float u_theta3_2 = -(_2_a2_d4 - sqrt_eq1) / (const_eq2 + pow_distance_2);

    float theta3_1 = kin_atanf(u_theta3_1) * 2;
    float theta3_2 = kin_atanf(u_theta3_2) * 2;

    /* 前2个解用theta3_1, 后2个用theta3_2 */
    for (int i = 0; i < KIN_SOLUTION_NUM/2; i++) {
        solver->solutions[i][2] = theta3_1;
    }
    for (int i = KIN_SOLUTION_NUM/2; i < KIN_SOLUTION_NUM; i++) {
        solver->solutions[i][2] = theta3_2;
    }
}

static void solve_theta2_single(kin_solver_t *solver, float theta3, float *theta2_1, float *theta2_2)
{
    float pz = solver->T_target[2][3];

    float _pow_a2_2 = a2 * a2;      /* 优化: 用乘法代替powf */
    float _pow_a3_2 = a3 * a3;
    float _pow_d4_2 = d4 * d4;
    float _2_a2_a3 = 2 * a2 * a3;
    float _2_a2_d4 = 2 * a2 * d4;

    /* 优化: 一次计算sin和cos */
    float cos_theta3, sin_theta3;
    kin_sincosf(theta3, &sin_theta3, &cos_theta3);

    float const_eq1 = _pow_a2_2 + _pow_a3_2 + _pow_d4_2;
    float eq1 = kin_sqrtf(const_eq1 + _2_a2_a3*cos_theta3 - _2_a2_d4*sin_theta3 - pz*pz);
    float eq2 = a3*cos_theta3 - d4*sin_theta3;
    float eq3 = (d4*cos_theta3 - pz + a3*sin_theta3);

    float u_theta2_1 = -(a2 + eq1 + eq2) / eq3;
    float u_theta2_2 = -(a2 - eq1 + eq2) / eq3;

    *theta2_1 = kin_atanf(u_theta2_1) * 2;
    *theta2_2 = kin_atanf(u_theta2_2) * 2;
}

static void solve_theta2(kin_solver_t *solver)
{
    float theta3, theta2_1, theta2_2;
    
    /* 解0,1 使用同一个theta3 */
    theta3 = solver->solutions[0][2];
    solve_theta2_single(solver, theta3, &theta2_1, &theta2_2);
    solver->solutions[0][1] = theta2_1;
    solver->solutions[1][1] = theta2_2;

    /* 解2,3 使用另一个theta3 */
    theta3 = solver->solutions[2][2];
    solve_theta2_single(solver, theta3, &theta2_1, &theta2_2);
    solver->solutions[2][1] = theta2_1;
    solver->solutions[3][1] = theta2_2;
}

static float solve_theta1_single(kin_solver_t *solver, float theta2, float theta3)
{
    float px = solver->T_target[0][3];
    float py = solver->T_target[1][3];

    float diff_theta2_3 = theta2 - theta3;

    /* 优化: 使用sincosf一次计算sin和cos */
    float cos_diff, sin_diff;
    float cos_theta2, sin_theta2;
    float cos_theta3, sin_theta3;
    kin_sincosf(diff_theta2_3, &sin_diff, &cos_diff);
    kin_sincosf(theta2, &sin_theta2, &cos_theta2);
    kin_sincosf(theta3, &sin_theta3, &cos_theta3);

    float eq1 = a2*cos_theta2 + a3*cos_diff + d4*sin_diff;
    float u_theta1 = kin_sqrtf((-px + eq1)/(px + eq1));

    float u_sq_plus_1 = u_theta1 * u_theta1 + 1;
    float eq2 = (2*u_theta1*(cos_theta2*(a2 + a3*cos_theta3 - d4*sin_theta3)
                + sin_theta2*(d4*cos_theta3 + a3*sin_theta3))) / u_sq_plus_1;

    /* u_theta1需满足(eq2 == py) */
    if (kin_fabsf(py - eq2) > KIN_ERROR_RANGE) {
        u_theta1 = -u_theta1;
    }

    return kin_atanf(u_theta1) * 2;
}

static void solve_theta1(kin_solver_t *solver)
{
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        float theta2 = solver->solutions[i][1];
        float theta3 = solver->solutions[i][2];
        solver->solutions[i][0] = solve_theta1_single(solver, theta2, theta3);
    }
}

static float solve_theta5_single(kin_solver_t *solver, float theta1, float theta2, float theta3)
{
    float nx = solver->T_target[0][0];
    float ny = solver->T_target[1][0];
    float nz = solver->T_target[2][0];
    float ox = solver->T_target[0][1];
    float oy = solver->T_target[1][1];
    float oz = solver->T_target[2][1];
    float ax = solver->T_target[0][2];
    float ay = solver->T_target[1][2];
    float az = solver->T_target[2][2];

    /* 优化: 使用sincosf批量计算 */
    float cos_theta1, sin_theta1;
    float cos_theta2, sin_theta2;
    float cos_theta3, sin_theta3;
    kin_sincosf(theta1, &sin_theta1, &cos_theta1);
    kin_sincosf(theta2, &sin_theta2, &cos_theta2);
    kin_sincosf(theta3, &sin_theta3, &cos_theta3);

    float r31 = nx*cos_theta1*cos_theta3*sin_theta2 - nz*sin_theta2*sin_theta3
                - nx*cos_theta1*cos_theta2*sin_theta3 - nz*cos_theta2*cos_theta3
                - ny*cos_theta2*sin_theta1*sin_theta3 + ny*cos_theta3*sin_theta1*sin_theta2;
    float r32 = ox*cos_theta1*cos_theta3*sin_theta2 - oz*sin_theta2*sin_theta3
                - ox*cos_theta1*cos_theta2*sin_theta3 - oz*cos_theta2*cos_theta3
                - oy*cos_theta2*sin_theta1*sin_theta3 + oy*cos_theta3*sin_theta1*sin_theta2;
    float r33 = ax*cos_theta1*cos_theta3*sin_theta2 - az*sin_theta2*sin_theta3
                - ax*cos_theta1*cos_theta2*sin_theta3 - az*cos_theta2*cos_theta3
                - ay*cos_theta2*sin_theta1*sin_theta3 + ay*cos_theta3*sin_theta1*sin_theta2;

    float theta5_zyz = kin_atan2f(kin_sqrtf(r31*r31 + r32*r32), r33);
    return -theta5_zyz + M_PI;
}

static void solve_theta5(kin_solver_t *solver)
{
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        float theta1 = solver->solutions[i][0];
        float theta2 = solver->solutions[i][1];
        float theta3 = solver->solutions[i][2];
        solver->solutions[i][4] = solve_theta5_single(solver, theta1, theta2, theta3);
    }
}

static float solve_theta4_single(kin_solver_t *solver, float theta1, float theta2, float theta3, float theta5)
{
    float theta5_zyz = M_PI - theta5;
    if ((kin_fabsf(theta5_zyz) < KIN_ERROR_RANGE) || (kin_fabsf(theta5_zyz - M_PI) < KIN_ERROR_RANGE)) {
        return 0;
    }

    float ax = solver->T_target[0][2];
    float ay = solver->T_target[1][2];
    float az = solver->T_target[2][2];

    /* theta4=0时, sin_theta4=0, cos_theta4=1, 简化计算 */
    float cos_theta1, sin_theta1;
    float cos_theta2, sin_theta2;
    float cos_theta3, sin_theta3;
    kin_sincosf(theta1, &sin_theta1, &cos_theta1);
    kin_sincosf(theta2, &sin_theta2, &cos_theta2);
    kin_sincosf(theta3, &sin_theta3, &cos_theta3);

    /* 当theta4=0时: sin_theta4=0, cos_theta4=1 */
    /* r23 = ax*cos_theta4*sin_theta1 - ay*cos_theta1*cos_theta4 + ... */
    /* 含sin_theta4的项全为0 */
    float r23 = ax*sin_theta1 - ay*cos_theta1;

    float r13 = -az*cos_theta2*sin_theta3 + az*cos_theta3*sin_theta2
    + ax*cos_theta1*cos_theta2*cos_theta3 + ay*cos_theta2*cos_theta3*sin_theta1
    + ax*cos_theta1*sin_theta2*sin_theta3 + ay*sin_theta1*sin_theta2*sin_theta3;

    return kin_atan2f(r23, r13);
}

static void solve_theta4(kin_solver_t *solver)
{
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        float theta1 = solver->solutions[i][0];
        float theta2 = solver->solutions[i][1];
        float theta3 = solver->solutions[i][2];
        float theta5 = solver->solutions[i][4];
        solver->solutions[i][3] = solve_theta4_single(solver, theta1, theta2, theta3, theta5);
    }
}

static float solve_theta6_single(kin_solver_t *solver, float theta1, float theta2, float theta3,
                                  float theta4, float theta5)
{
    (void)theta4; /* 未使用 */
    float theta5_zyz = M_PI - theta5;

    float nx = solver->T_target[0][0];
    float ny = solver->T_target[1][0];
    float nz = solver->T_target[2][0];
    float ox = solver->T_target[0][1];
    float oy = solver->T_target[1][1];
    float oz = solver->T_target[2][1];

    /* 优化: 使用sincosf批量计算 */
    float cos_theta1, sin_theta1;
    float cos_theta2, sin_theta2;
    float cos_theta3, sin_theta3;
    kin_sincosf(theta1, &sin_theta1, &cos_theta1);
    kin_sincosf(theta2, &sin_theta2, &cos_theta2);
    kin_sincosf(theta3, &sin_theta3, &cos_theta3);

    float theta6_zyz;
    if ((kin_fabsf(theta5_zyz) < KIN_ERROR_RANGE) || (kin_fabsf(theta5_zyz - M_PI) < KIN_ERROR_RANGE)) {
        float r12 = -oz*cos_theta2*sin_theta3 + oz*cos_theta3*sin_theta2
                    + ox*cos_theta1*cos_theta2*cos_theta3 + oy*cos_theta2*cos_theta3*sin_theta1
                    + ox*cos_theta1*sin_theta2*sin_theta3 + oy*sin_theta1*sin_theta2*sin_theta3;
        float r11 = -nz*cos_theta2*sin_theta3 + nz*cos_theta3*sin_theta2
                    + nx*cos_theta1*cos_theta2*cos_theta3 + ny*cos_theta2*cos_theta3*sin_theta1
                    + nx*cos_theta1*sin_theta2*sin_theta3 + ny*sin_theta1*sin_theta2*sin_theta3;
        theta6_zyz = kin_atan2f(-r12, r11);
        return theta6_zyz - M_PI;
    }

    float r32 = ox*cos_theta1*cos_theta3*sin_theta2 - oz*sin_theta2*sin_theta3
                - ox*cos_theta1*cos_theta2*sin_theta3 - oz*cos_theta2*cos_theta3
                - oy*cos_theta2*sin_theta1*sin_theta3 + oy*cos_theta3*sin_theta1*sin_theta2;
    float r31 = nx*cos_theta1*cos_theta3*sin_theta2 - nz*sin_theta2*sin_theta3
                - nx*cos_theta1*cos_theta2*sin_theta3 - nz*cos_theta2*cos_theta3
                - ny*cos_theta2*sin_theta1*sin_theta3 + ny*cos_theta3*sin_theta1*sin_theta2;

    theta6_zyz = kin_atan2f(r32, -r31);
    return theta6_zyz - M_PI;
}

static void solve_theta6(kin_solver_t *solver)
{
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        float theta1 = solver->solutions[i][0];
        float theta2 = solver->solutions[i][1];
        float theta3 = solver->solutions[i][2];
        float theta4 = solver->solutions[i][3];
        float theta5 = solver->solutions[i][4];
        solver->solutions[i][5] = solve_theta6_single(solver, theta1, theta2, theta3, theta4, theta5);
    }
}

/* ========== 辅助函数 ========== */

static float rad_to_deg_0_360(float rad)
{
    float deg = rad * (180.0f / M_PI);
    deg = fmodf(deg, 360.0f);
    if (deg < 0) {
        deg += 360.0f;
    }
    if (kin_fabsf(deg - 360.0f) < KIN_ERROR_RANGE) {
        deg = 0.0f;
    }
    return deg;
}

static void radians_to_degrees(kin_solver_t *solver)
{
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        for (int j = 0; j < KIN_MAX_JOINT_NUM; j++) {
            solver->solutions[i][j] = rad_to_deg_0_360(solver->solutions[i][j]);
        }
    }
}

static void map_joint_limits(kin_solver_t *solver)
{
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        for (int j = 0; j < KIN_MAX_JOINT_NUM; j++) {
            float angle = solver->solutions[i][j];
            float min_angle = solver->joint_limits[j].min_angle;
            float max_angle = solver->joint_limits[j].max_angle;

            /* 临界值处理 */
            if (kin_fabsf(angle - min_angle) < KIN_ERROR_RANGE) {
                angle = min_angle;
            }
            if (kin_fabsf(angle - max_angle) < KIN_ERROR_RANGE) {
                angle = max_angle;
            }

            /* 尝试映射到[0,360] */
            if (angle < min_angle) {
                angle += 360;
            } else if (angle > max_angle) {
                angle -= 360;
            }
            
            /* 检查是否有效 */
            if ((angle < min_angle) || (angle > max_angle)) {
                solver->invalid_mask |= (1 << i);
            }
            solver->solutions[i][j] = angle;
        }
    }
}

static int get_optimal_solution(kin_solver_t *solver, float result[KIN_MAX_JOINT_NUM])
{
    float min_diff = 1e9f;
    int min_index = -1;
    
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        if (solver->invalid_mask & (1 << i)) {
            continue; /* 跳过无效解 */
        }
        
        float diff = 0;
        for (int j = 0; j < KIN_MAX_JOINT_NUM; j++) {
            diff += kin_fabsf(solver->solutions[i][j] - solver->current_angles[j]) * JOINT_WEIGHTS[j];
        }
        
        if (diff < min_diff) {
            min_diff = diff;
            min_index = i;
        }
    }

    if (min_index == -1) {
        return -1; /* 无有效解 */
    }

    memcpy(result, solver->solutions[min_index], sizeof(float) * KIN_MAX_JOINT_NUM);
    return 0;
}

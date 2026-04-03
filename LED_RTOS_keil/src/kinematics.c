/**
 ******************************************************************************
 * @file    kinematics.c
 * @brief   6-DOF机械臂逆运动学求解器实现
 * @note    移植自dummy-auk项目, 使用几何法(腕心分离+余弦定理)
 ******************************************************************************
 */

#include "kinematics.h"
#include "kin_math.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

/* ========== Dummy机械臂连杆参数 (米) ========== */
#define L_BASE_M   0.109f
#define D_BASE_M   0.035f
#define L_ARM_M    0.146f
#define L_FOREARM_M 0.115f
#define D_ELBOW_M  0.052f
#define L_WRIST_M  0.072f

#define RAD_TO_DEG  57.295777754771045f

/* 关节选择权重(用于最优解计算) */
static const float JOINT_WEIGHTS[KIN_MAX_JOINT_NUM] = {5, 3, 3, 1, 1, 1};

/* 默认关节限位(度) — 宽松范围 */
static const kin_joint_limit_t DEFAULT_LIMITS[KIN_MAX_JOINT_NUM] = {
    {-180, 180},  /* Joint 1 */
    {-180, 180},  /* Joint 2 */
    {-180, 180},  /* Joint 3 */
    {-180, 180},  /* Joint 4 */
    {-180, 180},  /* Joint 5 */
    {-180, 180}   /* Joint 6 */
};

/* ========== 内部函数声明 ========== */
static void mat_multiply_3x3(const float *A, const float *B, float *C);
static void mat_multiply_3x1(const float *M, const float *v, float *out);
static int  solve_ik_geometric(kin_solver_t *solver);
static void map_joint_limits(kin_solver_t *solver);
static int  get_optimal_solution(kin_solver_t *solver, float result[KIN_MAX_JOINT_NUM]);

/* ========== 3x3矩阵运算 ========== */

static void mat_multiply_3x3(const float *A, const float *B, float *C)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum += A[3 * i + k] * B[3 * k + j];
            }
            C[3 * i + j] = sum;
        }
    }
}

static void mat_multiply_3x1(const float *M, const float *v, float *out)
{
    for (int i = 0; i < 3; i++) {
        out[i] = M[3 * i + 0] * v[0] + M[3 * i + 1] * v[1] + M[3 * i + 2] * v[2];
    }
}

/* ========== 公共接口实现 ========== */

void kin_solver_init(kin_solver_t *solver, const kin_joint_limit_t *joint_limits)
{
    memset(solver, 0, sizeof(kin_solver_t));

    if (joint_limits) {
        memcpy(solver->joint_limits, joint_limits, sizeof(kin_joint_limit_t) * KIN_MAX_JOINT_NUM);
    } else {
        memcpy(solver->joint_limits, DEFAULT_LIMITS, sizeof(kin_joint_limit_t) * KIN_MAX_JOINT_NUM);
    }

    /* 初始化dummy-auk几何IK配置 (单位: 米) */
    kin_ik_config_t *cfg = &solver->ik_cfg;

    /* 连杆向量 */
    cfg->L1_base[0] = D_BASE_M;   cfg->L1_base[1] = -L_BASE_M;  cfg->L1_base[2] = 0.0f;
    cfg->L2_arm[0]  = L_ARM_M;    cfg->L2_arm[1]  = 0.0f;        cfg->L2_arm[2]  = 0.0f;
    cfg->L3_elbow[0] = -D_ELBOW_M; cfg->L3_elbow[1] = 0.0f;      cfg->L3_elbow[2] = L_FOREARM_M;
    cfg->L6_wrist[0] = 0.0f;      cfg->L6_wrist[1] = 0.0f;       cfg->L6_wrist[2] = L_WRIST_M;

    /* 预计算 */
    cfg->l_se_2 = L_ARM_M * L_ARM_M;
    cfg->l_se   = L_ARM_M;
    cfg->l_ew_2 = L_FOREARM_M * L_FOREARM_M + D_ELBOW_M * D_ELBOW_M;
    cfg->l_ew   = kin_sqrtf(cfg->l_ew_2);
    cfg->atan_e = kin_atanf(D_ELBOW_M / L_FOREARM_M);

    /* DH矩阵 [6][4]: {home_offset, d, a, alpha} (与dummy-auk构造函数一致) */
    float tmp_DH[6][4] = {
        {0.0f,       L_BASE_M,    D_BASE_M,   -M_PI_2},
        {-M_PI_2,    0.0f,        L_ARM_M,     0.0f},
        {M_PI_2,     D_ELBOW_M,   0.0f,        M_PI_2},
        {0.0f,       L_FOREARM_M, 0.0f,       -M_PI_2},
        {0.0f,       0.0f,        0.0f,        M_PI_2},
        {0.0f,       L_WRIST_M,   0.0f,        0.0f}
    };
    memcpy(cfg->DH_matrix, tmp_DH, sizeof(tmp_DH));
}

int kin_inverse_kinematics(kin_solver_t *solver, const float T_target[4][4], float result[KIN_MAX_JOINT_NUM])
{
    /* 复制目标矩阵 */
    memcpy(solver->T_target, T_target, sizeof(float) * 16);

    /* 清除无效掩码和解 */
    solver->invalid_mask = 0;
    memset(solver->solutions, 0, sizeof(solver->solutions));

    /* 几何法IK求解 */
    solve_ik_geometric(solver);

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
        if (!(solver->invalid_mask & (1u << i))) {
            count++;
        }
    }
    return count;
}

/* ========== 几何法IK求解 (移植自dummy-auk SolveIK) ========== */

/**
 * @brief 安全acosf — 钳位输入到[-1,1]
 */
static float safe_acosf(float x)
{
    if (x >= 1.0f) return 0.0f;
    if (x <= -1.0f) return (float)M_PI;
    return kin_acosf(x);
}

/**
 * @brief 将弧度归一化到 (-pi, pi]
 */
static float wrap_to_pi(float rad)
{
    while (rad > (float)M_PI)  rad -= 2.0f * (float)M_PI;
    while (rad <= -(float)M_PI) rad += 2.0f * (float)M_PI;
    return rad;
}

static int solve_ik_geometric(kin_solver_t *solver)
{
    kin_ik_config_t *cfg = &solver->ik_cfg;

    /* solFlag[8][3]: 标记每组解的3段(shoulder/arm/wrist)有效性
     *  1 = 正常解, -1 = 奇异(退化), 0 = 超限无效 */
    int solFlag[8][3];
    memset(solFlag, 0, sizeof(solFlag));

    /* --- 1. 从T_target提取位置(mm→m)和旋转矩阵 --- */
    float P06[3];
    float R06[9];

    P06[0] = solver->T_target[0][3] / 1000.0f;  /* mm → m */
    P06[1] = solver->T_target[1][3] / 1000.0f;
    P06[2] = solver->T_target[2][3] / 1000.0f;

    /* 从T_target的3x3子矩阵提取旋转矩阵(行优先→行优先) */
    R06[0] = solver->T_target[0][0];  R06[1] = solver->T_target[0][1];  R06[2] = solver->T_target[0][2];
    R06[3] = solver->T_target[1][0];  R06[4] = solver->T_target[1][1];  R06[5] = solver->T_target[1][2];
    R06[6] = solver->T_target[2][0];  R06[7] = solver->T_target[2][1];  R06[8] = solver->T_target[2][2];

    /* --- 2. 计算腕心: P0_w = P06 - R06 * L6_wrist --- */
    float L0_wt[3], P0_w[3];
    mat_multiply_3x1(R06, cfg->L6_wrist, L0_wt);
    for (int i = 0; i < 3; i++) {
        P0_w[i] = P06[i] - L0_wt[i];
    }

    /* --- 3. 求theta1 (shoulder): 2组解 --- */
    float qs[2];  /* theta1的2个解(弧度) */
    float pw_xy = kin_sqrtf(P0_w[0] * P0_w[0] + P0_w[1] * P0_w[1]);

    if (pw_xy <= 1e-6f) {
        /* 奇异: 腕心在z轴上, theta1退化 — 使用当前角度 */
        float last_q0 = solver->current_angles[0] / RAD_TO_DEG;
        qs[0] = last_q0;
        qs[1] = last_q0;
        for (int i = 0; i < 4; i++) {
            solFlag[0 + i][0] = -1;
            solFlag[4 + i][0] = -1;
        }
    } else {
        qs[0] = kin_atan2f(P0_w[1], P0_w[0]);
        qs[1] = kin_atan2f(-P0_w[1], -P0_w[0]);
        for (int i = 0; i < 4; i++) {
            solFlag[0 + i][0] = 1;
            solFlag[4 + i][0] = 1;
        }
    }

    /* --- 4. 对每个arm配置(theta1), 求theta2/theta3 --- */
    float qa[2][2];  /* qa[elbow_config][0]=theta2, [1]=theta3 */
    float qw[2][3];  /* qw[wrist_config][0]=theta4, [1]=theta5, [2]=theta6 */

    /* 初始化为当前角度(用于奇异情况fallback) */
    for (int i = 0; i < 2; i++) {
        qa[i][0] = solver->current_angles[1] / RAD_TO_DEG;
        qa[i][1] = solver->current_angles[2] / RAD_TO_DEG;
        qw[i][0] = solver->current_angles[3] / RAD_TO_DEG;
        qw[i][1] = solver->current_angles[4] / RAD_TO_DEG;
        qw[i][2] = solver->current_angles[5] / RAD_TO_DEG;
    }

    for (int ind_arm = 0; ind_arm < 2; ind_arm++) {
        /* R10 = R01^T (frame 1 相对于 frame 0 的逆旋转) */
        float cosqs = kin_cosf(qs[ind_arm] + cfg->DH_matrix[0][0]);
        float sinqs = kin_sinf(qs[ind_arm] + cfg->DH_matrix[0][0]);

        float R10[9];
        R10[0] =  cosqs;   R10[1] = sinqs;   R10[2] = 0.0f;
        R10[3] =  0.0f;    R10[4] = 0.0f;    R10[5] = -1.0f;
        R10[6] = -sinqs;   R10[7] = cosqs;   R10[8] = 0.0f;

        /* P1_w = R10 * P0_w, 然后减去基座偏移 */
        float P1_w[3], L1_sw[3];
        mat_multiply_3x1(R10, P0_w, P1_w);
        for (int i = 0; i < 3; i++) {
            L1_sw[i] = P1_w[i] - cfg->L1_base[i];
        }

        float l_sw_2 = L1_sw[0] * L1_sw[0] + L1_sw[1] * L1_sw[1];
        float l_sw = kin_sqrtf(l_sw_2);

        /* 余弦定理求theta2, theta3 */
        if (kin_fabsf(cfg->l_se + cfg->l_ew - l_sw) <= 1e-6f) {
            /* 完全伸展 */
            qa[0][0] = kin_atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            qa[0][1] = 0.0f;
            qa[1][1] = 0.0f;
            if (l_sw > cfg->l_se + cfg->l_ew) {
                for (int i = 0; i < 2; i++) {
                    solFlag[4 * ind_arm + 0 + i][1] = 0;
                    solFlag[4 * ind_arm + 2 + i][1] = 0;
                }
            } else {
                for (int i = 0; i < 2; i++) {
                    solFlag[4 * ind_arm + 0 + i][1] = 1;
                    solFlag[4 * ind_arm + 2 + i][1] = 1;
                }
            }
        } else if (kin_fabsf(l_sw - kin_fabsf(cfg->l_se - cfg->l_ew)) <= 1e-6f) {
            /* 完全折叠 */
            qa[0][0] = kin_atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            if (ind_arm == 0) {
                qa[0][1] = (float)M_PI;
                qa[1][1] = -(float)M_PI;
            } else {
                qa[0][1] = -(float)M_PI;
                qa[1][1] = (float)M_PI;
            }
            if (l_sw < kin_fabsf(cfg->l_se - cfg->l_ew)) {
                for (int i = 0; i < 2; i++) {
                    solFlag[4 * ind_arm + 0 + i][1] = 0;
                    solFlag[4 * ind_arm + 2 + i][1] = 0;
                }
            } else {
                for (int i = 0; i < 2; i++) {
                    solFlag[4 * ind_arm + 0 + i][1] = 1;
                    solFlag[4 * ind_arm + 2 + i][1] = 1;
                }
            }
        } else {
            /* 正常情况: 余弦定理 */
            float atan_a = kin_atan2f(L1_sw[1], L1_sw[0]);
            float cos_acos_a = 0.5f * (cfg->l_se_2 + l_sw_2 - cfg->l_ew_2) / (cfg->l_se * l_sw);
            float acos_a = safe_acosf(cos_acos_a);
            float cos_acos_e = 0.5f * (cfg->l_se_2 + cfg->l_ew_2 - l_sw_2) / (cfg->l_se * cfg->l_ew);
            float acos_e = safe_acosf(cos_acos_e);

            if (ind_arm == 0) {
                qa[0][0] = atan_a - acos_a + M_PI_2;
                qa[0][1] = cfg->atan_e - acos_e + (float)M_PI;
                qa[1][0] = atan_a + acos_a + M_PI_2;
                qa[1][1] = cfg->atan_e + acos_e - (float)M_PI;
            } else {
                qa[0][0] = atan_a + acos_a + M_PI_2;
                qa[0][1] = cfg->atan_e + acos_e - (float)M_PI;
                qa[1][0] = atan_a - acos_a + M_PI_2;
                qa[1][1] = cfg->atan_e - acos_e + (float)M_PI;
            }
            for (int i = 0; i < 2; i++) {
                solFlag[4 * ind_arm + 0 + i][1] = 1;
                solFlag[4 * ind_arm + 2 + i][1] = 1;
            }
        }

        /* --- 5. 对每个elbow配置, 求theta4/theta5/theta6 --- */
        for (int ind_elbow = 0; ind_elbow < 2; ind_elbow++) {
            /* R31 = R32 * R21 (从frame3到frame1的旋转) */
            float cosqa0 = kin_cosf(qa[ind_elbow][0] + cfg->DH_matrix[1][0]);
            float sinqa0 = kin_sinf(qa[ind_elbow][0] + cfg->DH_matrix[1][0]);
            float cosqa1 = kin_cosf(qa[ind_elbow][1] + cfg->DH_matrix[2][0]);
            float sinqa1 = kin_sinf(qa[ind_elbow][1] + cfg->DH_matrix[2][0]);

            float R31[9];
            R31[0] = cosqa0 * cosqa1 - sinqa0 * sinqa1;
            R31[1] = cosqa0 * sinqa1 + sinqa0 * cosqa1;
            R31[2] = 0.0f;
            R31[3] = 0.0f;
            R31[4] = 0.0f;
            R31[5] = 1.0f;
            R31[6] = cosqa0 * sinqa1 + sinqa0 * cosqa1;
            R31[7] = -cosqa0 * cosqa1 + sinqa0 * sinqa1;
            R31[8] = 0.0f;

            /* R36 = R31 * R10 * R06 = R30 * R06 */
            float R30[9], R36[9];
            mat_multiply_3x3(R31, R10, R30);
            mat_multiply_3x3(R30, R06, R36);

            /* 从R36分解theta5 (腕关节)
             * 奇异性检测: theta5≈0或≈π时, theta4/theta6不可分解
             * 阈值收紧，靠近dummy-auk默认策略，避免过早进入奇异分支 */
            #define WRIST_SINGULAR_THRESH  1e-4f
            float cosqw;
            if (R36[8] >= 1.0f - WRIST_SINGULAR_THRESH) {
                cosqw = 1.0f;
                qw[0][1] = 0.0f;
                qw[1][1] = 0.0f;
            } else if (R36[8] <= -1.0f + WRIST_SINGULAR_THRESH) {
                cosqw = -1.0f;
                if (ind_arm == 0) {
                    qw[0][1] = (float)M_PI;
                    qw[1][1] = -(float)M_PI;
                } else {
                    qw[0][1] = -(float)M_PI;
                    qw[1][1] = (float)M_PI;
                }
            } else {
                cosqw = R36[8];
                if (ind_arm == 0) {
                    qw[0][1] = kin_acosf(cosqw);
                    qw[1][1] = -kin_acosf(cosqw);
                } else {
                    qw[0][1] = -kin_acosf(cosqw);
                    qw[1][1] = kin_acosf(cosqw);
                }
            }

            /* 从R36分解theta4, theta6 */
            if (cosqw == 1.0f || cosqw == -1.0f) {
                /* 腕关节奇异: theta4 + theta6 耦合 */
                float last_q3 = solver->current_angles[3] / RAD_TO_DEG;
                float last_q5 = solver->current_angles[5] / RAD_TO_DEG;
                float cqw_tmp, sqw_tmp;
                if (ind_arm == 0) {
                    qw[0][0] = last_q3;
                    cqw_tmp = kin_cosf(last_q3 + cfg->DH_matrix[3][0]);
                    sqw_tmp = kin_sinf(last_q3 + cfg->DH_matrix[3][0]);
                    qw[0][2] = kin_atan2f(cqw_tmp * R36[3] - sqw_tmp * R36[0],
                                          cqw_tmp * R36[0] + sqw_tmp * R36[3]);
                    qw[1][2] = last_q5;
                    cqw_tmp = kin_cosf(last_q5 + cfg->DH_matrix[5][0]);
                    sqw_tmp = kin_sinf(last_q5 + cfg->DH_matrix[5][0]);
                    qw[1][0] = kin_atan2f(cqw_tmp * R36[3] - sqw_tmp * R36[0],
                                          cqw_tmp * R36[0] + sqw_tmp * R36[3]);
                } else {
                    qw[0][2] = last_q5;
                    cqw_tmp = kin_cosf(last_q5 + cfg->DH_matrix[5][0]);
                    sqw_tmp = kin_sinf(last_q5 + cfg->DH_matrix[5][0]);
                    qw[0][0] = kin_atan2f(cqw_tmp * R36[3] - sqw_tmp * R36[0],
                                          cqw_tmp * R36[0] + sqw_tmp * R36[3]);
                    qw[1][0] = last_q3;
                    cqw_tmp = kin_cosf(last_q3 + cfg->DH_matrix[3][0]);
                    sqw_tmp = kin_sinf(last_q3 + cfg->DH_matrix[3][0]);
                    qw[1][2] = kin_atan2f(cqw_tmp * R36[3] - sqw_tmp * R36[0],
                                          cqw_tmp * R36[0] + sqw_tmp * R36[3]);
                }
                solFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = -1;
                solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;
            } else {
                /* 正常: 直接从R36提取theta4, theta6 */
                if (ind_arm == 0) {
                    qw[0][0] = kin_atan2f(R36[5], R36[2]);
                    qw[1][0] = kin_atan2f(-R36[5], -R36[2]);
                    qw[0][2] = kin_atan2f(R36[7], -R36[6]);
                    qw[1][2] = kin_atan2f(-R36[7], R36[6]);
                } else {
                    qw[0][0] = kin_atan2f(-R36[5], -R36[2]);
                    qw[1][0] = kin_atan2f(R36[5], R36[2]);
                    qw[0][2] = kin_atan2f(-R36[7], R36[6]);
                    qw[1][2] = kin_atan2f(R36[7], -R36[6]);
                }
                solFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = 1;
                solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;
            }

            /* --- 6. 组装8组解 ---
             * qa 的计算式 (atan ± acos ± π/2 等) 可能使角度超出 (-π, π]
             * 与参考实现 (6dof_kinematic.cpp 456-489) 保持一致:
             * 对 qa 用 ±π 步长归一化 (不是 ±2π), 确保物理关节角在可达范围内 */
            for (int ind_wrist = 0; ind_wrist < 2; ind_wrist++) {
                int sol_idx = 4 * ind_arm + 2 * ind_elbow + ind_wrist;
                float *sol = solver->solutions[sol_idx];

                /* theta1: qs 来自 atan2，已在 (-π, π]，无需归一化 */
                sol[0] = qs[ind_arm] * RAD_TO_DEG;

                /* theta2, theta3: ±π 步长归一化（与参考实现一致）*/
                for (int k = 0; k < 2; k++) {
                    float angle = qa[ind_elbow][k];
                    if      (angle >  (float)M_PI) angle -= (float)M_PI;
                    else if (angle < -(float)M_PI) angle += (float)M_PI;
                    sol[1 + k] = angle * RAD_TO_DEG;
                }

                /* theta4, theta5, theta6: qw 来自 atan2/acos，已在 (-π, π] */
                sol[3] = qw[ind_wrist][0] * RAD_TO_DEG;
                sol[4] = qw[ind_wrist][1] * RAD_TO_DEG;
                sol[5] = qw[ind_wrist][2] * RAD_TO_DEG;

                /* 标记无效解: solFlag中任一段为0则无效 */
                if (solFlag[sol_idx][0] == 0 ||
                    solFlag[sol_idx][1] == 0) {
                    solver->invalid_mask |= (1u << sol_idx);
                }
            }
        }
    }

    return 0;
}

/* ========== 辅助函数 ========== */

/* 关节限位边界容差(度) — 覆盖IK浮点累积误差(~0.2°) */
#define KIN_LIMIT_TOLERANCE  0.5f

static void map_joint_limits(kin_solver_t *solver)
{
    for (int i = 0; i < KIN_SOLUTION_NUM; i++) {
        if (solver->invalid_mask & (1u << i)) {
            continue;  /* 已标记无效, 跳过 */
        }
        for (int j = 0; j < KIN_MAX_JOINT_NUM; j++) {
            float angle = solver->solutions[i][j];
            float min_a = solver->joint_limits[j].min_angle;
            float max_a = solver->joint_limits[j].max_angle;

            /* 边界容差: 略微超限时吸附到边界 */
            if (angle < min_a && (min_a - angle) < KIN_LIMIT_TOLERANCE)
                angle = min_a;
            if (angle > max_a && (angle - max_a) < KIN_LIMIT_TOLERANCE)
                angle = max_a;

            /* ±360°归一化: 找到落在[min,max]内的偏移 */
            if (angle < min_a || angle > max_a) {
                float a360  = angle + 360.0f;
                float a_360 = angle - 360.0f;
                if (a360 >= min_a - KIN_LIMIT_TOLERANCE &&
                    a360 <= max_a + KIN_LIMIT_TOLERANCE) {
                    angle = (a360 > max_a) ? max_a :
                            (a360 < min_a) ? min_a : a360;
                } else if (a_360 >= min_a - KIN_LIMIT_TOLERANCE &&
                           a_360 <= max_a + KIN_LIMIT_TOLERANCE) {
                    angle = (a_360 > max_a) ? max_a :
                            (a_360 < min_a) ? min_a : a_360;
                } else {
                    solver->invalid_mask |= (1u << i);
                }
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
        if (solver->invalid_mask & (1u << i)) {
            continue;
        }

        float diff = 0;
        for (int j = 0; j < KIN_MAX_JOINT_NUM; j++) {
            float d = solver->solutions[i][j] - solver->current_angles[j];
            /* 考虑角度环绕 */
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            diff += kin_fabsf(d) * JOINT_WEIGHTS[j];
        }

        if (diff < min_diff) {
            min_diff = diff;
            min_index = i;
        }
    }

    if (min_index == -1) {
        return -1;  /* 无有效解 */
    }

    memcpy(result, solver->solutions[min_index], sizeof(float) * KIN_MAX_JOINT_NUM);
    return 0;
}

/* ========== 正运动学 (移植自dummy-auk SolveFK) ========== */

void kin_forward_kinematics(const kin_solver_t *solver,
                            const float joint_angles[KIN_MAX_JOINT_NUM],
                            float T_out[4][4])
{
    const kin_ik_config_t *cfg = &solver->ik_cfg;

    /* 1. 角度→弧度 + DH home offset */
    float q[6];
    for (int i = 0; i < 6; i++) {
        q[i] = joint_angles[i] / RAD_TO_DEG + cfg->DH_matrix[i][0];
    }

    /* 2. 逐关节DH旋转矩阵 Ri (3×3) */
    float R[6][9];
    for (int i = 0; i < 6; i++) {
        float cq = kin_cosf(q[i]);
        float sq = kin_sinf(q[i]);
        float ca = kin_cosf(cfg->DH_matrix[i][3]);  /* alpha */
        float sa = kin_sinf(cfg->DH_matrix[i][3]);

        R[i][0] =  cq;    R[i][1] = -ca * sq;  R[i][2] =  sa * sq;
        R[i][3] =  sq;    R[i][4] =  ca * cq;  R[i][5] = -sa * cq;
        R[i][6] = 0.0f;   R[i][7] =  sa;       R[i][8] =  ca;
    }

    /* 3. 链式乘法: R02, R03, R04, R05, R06 */
    float R02[9], R03[9], R04[9], R05[9], R06[9];
    mat_multiply_3x3(R[0], R[1], R02);
    mat_multiply_3x3(R02,  R[2], R03);
    mat_multiply_3x3(R03,  R[3], R04);
    mat_multiply_3x3(R04,  R[4], R05);
    mat_multiply_3x3(R05,  R[5], R06);

    /* 4. 位置: P06 = R0*L1 + R02*L2 + R03*L3 + R06*L6 */
    float L0_bs[3], L0_se[3], L0_ew[3], L0_wt[3];
    mat_multiply_3x1(R[0], cfg->L1_base,  L0_bs);
    mat_multiply_3x1(R02,  cfg->L2_arm,   L0_se);
    mat_multiply_3x1(R03,  cfg->L3_elbow, L0_ew);
    mat_multiply_3x1(R06,  cfg->L6_wrist, L0_wt);

    float pos[3];
    for (int i = 0; i < 3; i++) {
        pos[i] = L0_bs[i] + L0_se[i] + L0_ew[i] + L0_wt[i];
    }

    /* 5. 组装4×4 T矩阵, 位置 m→mm */
    T_out[0][0] = R06[0];  T_out[0][1] = R06[1];  T_out[0][2] = R06[2];  T_out[0][3] = pos[0] * 1000.0f;
    T_out[1][0] = R06[3];  T_out[1][1] = R06[4];  T_out[1][2] = R06[5];  T_out[1][3] = pos[1] * 1000.0f;
    T_out[2][0] = R06[6];  T_out[2][1] = R06[7];  T_out[2][2] = R06[8];  T_out[2][3] = pos[2] * 1000.0f;
    T_out[3][0] = 0.0f;    T_out[3][1] = 0.0f;    T_out[3][2] = 0.0f;    T_out[3][3] = 1.0f;
}

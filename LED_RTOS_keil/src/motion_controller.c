/**
 * @file    motion_controller.c
 * @brief   运动控制器实现 - USB后端 (Dummy ARM)
 * @note    通过USB CDC发送ASCII命令控制Dummy机械臂
 *          Dummy ARM内部处理IK、轨迹规划、电机控制
 *          RA6M5端保留IK求解用于笛卡尔→关节角转换
 */

#include "motion_controller.h"
#include "kinematics.h"
#include "dummy_arm_cmd.h"
#include "usb_host_cdc.h"
#include "degradation.h"
#include "watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* CMSIS-DSP 矩阵运算加速 */
#include "arm_math_types.h"
#include "dsp/matrix_functions.h"

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* debug_printf stub - 使用空实现 */
#define debug_printf(...) ((void)0)

/* ========== 关节配置 (从motor_ctrl_step迁移) ========== */
#define NUM_JOINTS  6U

typedef struct {
    bool    inverse;      /* 逻辑角度是否反转 */
    float   reduction;    /* 减速比 */
    float   angle_min;    /* 电机角度下限 (度) */
    float   angle_max;    /* 电机角度上限 (度) */
} joint_limit_config_t;

static const joint_limit_config_t g_joint_limits[NUM_JOINTS] = {
    /* inverse, reduction, angle_min, angle_max  (dummy-auk 原生角度, 度) */
    { true,  30.0f, -170.0f, 170.0f},   /* J0 */
    { false, 30.0f,  -73.0f,  90.0f},   /* J1 */
    { true,  30.0f,   35.0f, 180.0f},   /* J2 */
    { false, 24.0f, -180.0f, 180.0f},   /* J3 */
    { true,  30.0f, -120.0f, 120.0f},   /* J4 */
    { true,  50.0f, -720.0f, 720.0f},   /* J5 */
};

/* ========== 内部状态 ========== */
static kin_solver_t g_ik_solver;
static motion_state_t g_state = MOTION_IDLE;
static float g_current_joints[6] = {0};
static const float g_joint_weights[NUM_JOINTS] = {5.0f, 3.0f, 3.0f, 1.0f, 1.0f, 1.0f};

typedef enum {
    IK_ACCEPT_HARD = 0,
    IK_ACCEPT_SOFT,
    IK_ACCEPT_DEGRADE,
    IK_ACCEPT_REJECT
} ik_accept_level_t;

typedef struct {
    float joints[NUM_JOINTS];
    float pos_err_mm;
    float rot_err;
    float joint_cost;
    bool  valid;
} ik_candidate_eval_t;


/* ========== 默认配置 ========== */
#define MOTION_DEFAULT_SPEED_DEG_S 30.0f   /* 对齐dummy-auk默认关节速度 */
#define MOTION_SETTLE_TIMEOUT_MS   15000U  /* 最长等待到位时间 */
#define MOTION_SETTLE_POLL_MS      300U    /* 查询关节状态周期 */
#define MOTION_SETTLE_TOL_DEG      0.3f    /* 关节稳定阈值 */
#define MOTION_SETTLE_STABLE_NEED  3U      /* 连续稳定次数 */
#define MOTION_MAX_USB_QUERY_FAIL  3U      /* 连续查询失败阈值 */
#define MOTION_FK_POS_HARD_MM      5.0f    /* IK硬通过位置误差上限 */
#define MOTION_FK_POS_SOFT_MM      10.0f   /* IK软通过位置误差上限 */
#define MOTION_FK_POS_DEGRADE_MM   20.0f   /* IK降级通过位置误差上限 */
#define MOTION_FK_ROT_TOL          0.05f   /* IK结果回代旋转误差上限(Frobenius) */

/* ========== ZERO坐标系 ========== */

/**
 * T_0_6_reset - 机械臂初始姿态矩阵（末端执行器零点位置）
 *
 * 对应dummy-auk的REST_POSE = {0, -73, 180, 0, 0, 0}（度）
 * 使用dummy-auk FK手算：
 *   DH: {home, d, a, alpha}, 连杆向量法
 *   q = {0, -1.2741, 3.1416, 0, 0, 0} + home_offset
 *   P06 = R0*L1 + R02*L2 + R03*L3 + R06*L6
 *   位置: (89.4, 0.0, 146.7) mm
 *   旋转: R06 = {{-0.2924, 0, 0.9563}, {0, 1, 0}, {-0.9563, 0, -0.2924}}
 */
static const float T_0_6_reset[4][4] = {
    { -0.2924f,   0.0000f,   0.9563f,    89.4f},
    {  0.0000f,   1.0000f,   0.0000f,     0.0f},
    { -0.9563f,   0.0000f,  -0.2924f,   146.7f},
    {  0.0000f,   0.0000f,   0.0000f,     1.0f}
};

/* ========== 内部函数 ========== */

/**
 * @brief 4x4矩阵乘法 C = A * B (CMSIS-DSP 加速版本)
 * 使用 arm_mat_mult_f32() 替代手写循环，利用 SIMD 指令加速
 */
static void matrix_multiply_4x4(const float A[4][4], const float B[4][4], float C[4][4])
{
    arm_matrix_instance_f32 A_mat, B_mat, C_mat;

    arm_mat_init_f32(&A_mat, 4, 4, (float32_t *)A);
    arm_mat_init_f32(&B_mat, 4, 4, (float32_t *)B);
    arm_mat_init_f32(&C_mat, 4, 4, (float32_t *)C);

    arm_mat_mult_f32(&A_mat, &B_mat, &C_mat);
}

/**
 * @brief 构建平移矩阵 (纯位移，无旋转)
 */
static void build_translation_matrix(float T[4][4], float x, float y, float z)
{
    /* 单位矩阵 + 平移 */
    T[0][0] = 1.0f; T[0][1] = 0.0f; T[0][2] = 0.0f; T[0][3] = x;
    T[1][0] = 0.0f; T[1][1] = 1.0f; T[1][2] = 0.0f; T[1][3] = y;
    T[2][0] = 0.0f; T[2][1] = 0.0f; T[2][2] = 1.0f; T[2][3] = z;
    T[3][0] = 0.0f; T[3][1] = 0.0f; T[3][2] = 0.0f; T[3][3] = 1.0f;
}

/**
 * @brief 构建4x4齐次变换矩阵
 * 使用ZYX欧拉角约定
 */
static void build_transform_matrix(float T[4][4],
                                   float x, float y, float z,
                                   float roll, float pitch, float yaw)
{
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    /* 旋转矩阵 (ZYX欧拉角) */
    T[0][0] = cy * cp;
    T[0][1] = cy * sp * sr - sy * cr;
    T[0][2] = cy * sp * cr + sy * sr;
    T[0][3] = x;

    T[1][0] = sy * cp;
    T[1][1] = sy * sp * sr + cy * cr;
    T[1][2] = sy * sp * cr - cy * sr;
    T[1][3] = y;

    T[2][0] = -sp;
    T[2][1] = cp * sr;
    T[2][2] = cp * cr;
    T[2][3] = z;

    T[3][0] = 0.0f;
    T[3][1] = 0.0f;
    T[3][2] = 0.0f;
    T[3][3] = 1.0f;
}

static const char * motion_state_str(motion_state_t state)
{
    switch (state) {
        case MOTION_IDLE: return "IDLE";
        case MOTION_PLANNING: return "PLANNING";
        case MOTION_EXECUTING: return "EXECUTING";
        case MOTION_DONE: return "DONE";
        case MOTION_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

static void motion_set_state(motion_state_t next_state, const char * reason)
{
    if (g_state == next_state) {
        return;
    }

    debug_print("[MOTION] ");
    debug_print(motion_state_str(g_state));
    debug_print(" -> ");
    debug_print(motion_state_str(next_state));
    if (reason != NULL) {
        debug_print(" : ");
        debug_print(reason);
    }
    debug_println("");

    g_state = next_state;
}

static void motion_refresh_ik_limits(void)
{
    kin_joint_limit_t limits[NUM_JOINTS];

    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        limits[i].min_angle = g_joint_limits[i].angle_min;
        limits[i].max_angle = g_joint_limits[i].angle_max;
    }

    kin_solver_init(&g_ik_solver, limits);
    kin_update_current_angles(&g_ik_solver, g_current_joints);
}

static int motion_wait_settled(uint32_t timeout_ms, float tol_deg, uint8_t stable_needed)
{
    if (!usb_cdc_is_ready()) {
        motion_set_state(MOTION_ERROR, "USB not connected");
        return -6;
    }

    dummy_joint_pos_t prev = {{0.0f}};
    dummy_joint_pos_t cur = {{0.0f}};
    bool has_prev = false;
    uint8_t stable_count = 0;
    uint8_t fail_count = 0;
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(MOTION_SETTLE_POLL_MS));
        elapsed_ms += MOTION_SETTLE_POLL_MS;
        watchdog_refresh();

        if (!usb_cdc_is_ready()) {
            motion_set_state(MOTION_ERROR, "USB disconnected");
            return -6;
        }

        if (dummy_arm_get_joint_pos(&cur) != 0) {
            fail_count++;
            if (fail_count >= MOTION_MAX_USB_QUERY_FAIL) {
                motion_set_state(MOTION_ERROR, "GETJPOS failed");
                return -7;
            }
            continue;
        }

        fail_count = 0;

        if (!has_prev) {
            prev = cur;
            has_prev = true;
            continue;
        }

        float max_diff = 0.0f;
        for (uint8_t i = 0; i < NUM_JOINTS; i++) {
            float diff = fabsf(cur.j[i] - prev.j[i]);
            if (diff > max_diff) {
                max_diff = diff;
            }
            g_current_joints[i] = cur.j[i];
        }

        if (max_diff < tol_deg) {
            stable_count++;
            if (stable_count >= stable_needed) {
                return 0;
            }
        } else {
            stable_count = 0;
        }

        prev = cur;
    }

    motion_set_state(MOTION_ERROR, "settle timeout");
    return -7;
}

static float motion_wrap_delta_deg(float delta)
{
    while (delta > 180.0f) {
        delta -= 360.0f;
    }
    while (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}

static float motion_calc_joint_cost(const float cur_joints[NUM_JOINTS], const float cand_joints[NUM_JOINTS])
{
    /* minimax 准则（与 dummy-auk MoveL 一致）：最小化最大单关节变化量
     * 避免加权和偏爱腕关节大转动的问题 */
    float max_delta = 0.0f;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        float delta = fabsf(motion_wrap_delta_deg(cand_joints[i] - cur_joints[i]));
        if (delta > max_delta) max_delta = delta;
    }
    return max_delta;
}

static void motion_calc_fk_error(const float T_target[4][4], const float target_joints[6],
                                 float *pos_err_mm, float *rot_err)
{
    float T_verify[4][4];
    kin_forward_kinematics(&g_ik_solver, target_joints, T_verify);

    float dx = T_verify[0][3] - T_target[0][3];
    float dy = T_verify[1][3] - T_target[1][3];
    float dz = T_verify[2][3] - T_target[2][3];

    float rot_sq = 0.0f;
    for (uint8_t r = 0; r < 3; r++) {
        for (uint8_t c = 0; c < 3; c++) {
            float d = T_verify[r][c] - T_target[r][c];
            rot_sq += d * d;
        }
    }

    if (pos_err_mm != NULL) {
        *pos_err_mm = sqrtf(dx * dx + dy * dy + dz * dz);
    }
    if (rot_err != NULL) {
        *rot_err = sqrtf(rot_sq);
    }
}

static bool motion_is_better_soft(const ik_candidate_eval_t *lhs, const ik_candidate_eval_t *rhs)
{
    const float eps = 1e-4f;
    if (lhs->pos_err_mm < rhs->pos_err_mm - eps) {
        return true;
    }
    if (fabsf(lhs->pos_err_mm - rhs->pos_err_mm) <= eps && lhs->joint_cost < rhs->joint_cost) {
        return true;
    }
    return false;
}

static int motion_select_ik_candidate(const float T_target[4][4], const float cur_joints[NUM_JOINTS],
                                      float selected_joints[NUM_JOINTS], ik_accept_level_t *selected_level,
                                      float *selected_pos_err, float *selected_rot_err, float *selected_joint_cost)
{
    float all_solutions[KIN_SOLUTION_NUM][KIN_MAX_JOINT_NUM] = {{0.0f}};
    uint32_t valid_mask = 0;
    int valid_cnt = kin_get_all_solutions(&g_ik_solver, all_solutions, &valid_mask);

    ik_candidate_eval_t best_hard = {0};
    ik_candidate_eval_t best_soft = {0};
    ik_candidate_eval_t best_reject = {0};
    uint8_t hard_count = 0;
    uint8_t soft_count = 0;
    uint8_t reject_count = 0;

    if (valid_cnt <= 0) {
        return -2;
    }

    for (uint8_t i = 0; i < KIN_SOLUTION_NUM; i++) {
        if (valid_mask & (1u << i)) {
            continue;
        }

        ik_candidate_eval_t eval = {0};
        memcpy(eval.joints, all_solutions[i], sizeof(eval.joints));
        motion_calc_fk_error(T_target, eval.joints, &eval.pos_err_mm, &eval.rot_err);
        eval.joint_cost = motion_calc_joint_cost(cur_joints, eval.joints);
        eval.valid = true;

        if (eval.rot_err <= MOTION_FK_ROT_TOL && eval.pos_err_mm <= MOTION_FK_POS_HARD_MM) {
            hard_count++;
            if (!best_hard.valid || eval.joint_cost < best_hard.joint_cost) {
                best_hard = eval;
            }
            continue;
        }

        if (eval.rot_err <= MOTION_FK_ROT_TOL && eval.pos_err_mm <= MOTION_FK_POS_SOFT_MM) {
            soft_count++;
            if (!best_soft.valid || motion_is_better_soft(&eval, &best_soft)) {
                best_soft = eval;
            }
            continue;
        }

        reject_count++;
        if (!best_reject.valid || motion_is_better_soft(&eval, &best_reject)) {
            best_reject = eval;
        }
    }

    {
        char summary[128];
        snprintf(summary, sizeof(summary),
                 "[MOTION] IK candidates valid=%d hard=%u soft=%u reject=%u",
                 valid_cnt, hard_count, soft_count, reject_count);
        debug_println(summary);
    }

    if (best_hard.valid) {
        memcpy(selected_joints, best_hard.joints, sizeof(best_hard.joints));
        if (selected_level != NULL) {
            *selected_level = IK_ACCEPT_HARD;
        }
        if (selected_pos_err != NULL) {
            *selected_pos_err = best_hard.pos_err_mm;
        }
        if (selected_rot_err != NULL) {
            *selected_rot_err = best_hard.rot_err;
        }
        if (selected_joint_cost != NULL) {
            *selected_joint_cost = best_hard.joint_cost;
        }
        return 0;
    }

    if (best_soft.valid) {
        memcpy(selected_joints, best_soft.joints, sizeof(best_soft.joints));
        if (selected_level != NULL) {
            *selected_level = IK_ACCEPT_SOFT;
        }
        if (selected_pos_err != NULL) {
            *selected_pos_err = best_soft.pos_err_mm;
        }
        if (selected_rot_err != NULL) {
            *selected_rot_err = best_soft.rot_err;
        }
        if (selected_joint_cost != NULL) {
            *selected_joint_cost = best_soft.joint_cost;
        }
        return 0;
    }

    /* 没有硬/软通过解时，如果仍有候选解且位置误差在可接受范围内，则降级通过 */
    if (best_reject.valid && best_reject.pos_err_mm <= MOTION_FK_POS_DEGRADE_MM) {
        memcpy(selected_joints, best_reject.joints, sizeof(best_reject.joints));
        if (selected_level != NULL) {
            *selected_level = IK_ACCEPT_DEGRADE;
        }
        if (selected_pos_err != NULL) {
            *selected_pos_err = best_reject.pos_err_mm;
        }
        if (selected_rot_err != NULL) {
            *selected_rot_err = best_reject.rot_err;
        }
        if (selected_joint_cost != NULL) {
            *selected_joint_cost = best_reject.joint_cost;
        }
        return 0;
    }

    if (selected_level != NULL) {
        *selected_level = IK_ACCEPT_REJECT;
    }
    if (best_reject.valid) {
        if (selected_pos_err != NULL) {
            *selected_pos_err = best_reject.pos_err_mm;
        }
        if (selected_rot_err != NULL) {
            *selected_rot_err = best_reject.rot_err;
        }
        if (selected_joint_cost != NULL) {
            *selected_joint_cost = best_reject.joint_cost;
        }
    }
    return -2;
}

/**
 * @brief 通过USB查询Dummy ARM关节位置并同步到g_current_joints
 *        在绕过motion_controller直接操作Dummy ARM后调用
 */
void motion_sync_from_usb(void)
{
    if (!usb_cdc_is_ready()) {
        return;
    }

    dummy_joint_pos_t jpos;
    if (dummy_arm_get_joint_pos(&jpos) == 0) {
        for (uint8_t i = 0; i < 6; i++) {
            g_current_joints[i] = jpos.j[i];
        }
    }
}

/**
 * @brief 检查关节限位 (dummy-auk 原生角度)
 * @return 0=OK, -5=超限
 */
static int motion_check_joint_limits(const float target_joints[6])
{
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        if ((target_joints[i] < g_joint_limits[i].angle_min) ||
            (target_joints[i] > g_joint_limits[i].angle_max)) {
            debug_printf("[MOTION] ERROR: Joint %d out of limit: %.2f deg, limit=[%.2f, %.2f]\r\n",
                         i, target_joints[i], g_joint_limits[i].angle_min, g_joint_limits[i].angle_max);
            return -5;
        }
    }
    return 0;
}

/**
 * @brief 通过USB发送关节运动命令到Dummy ARM
 * @param target_joints 目标关节角度 (dummy-auk 原生角度, 度)
 * @param tag 日志标签
 * @return 0=成功, <0=失败
 */
static int motion_send_usb_move(const float target_joints[6], const char * tag)
{
    /* 关节限位检查 */
    int ret = motion_check_joint_limits(target_joints);
    if (ret != 0) {
        motion_set_state(MOTION_ERROR, "joint out of limits");
        return ret;
    }

    /* 检查USB连接 */
    if (!usb_cdc_is_ready()) {
        debug_println("[MOTION] USB not connected");
        motion_set_state(MOTION_ERROR, "USB not connected");
        return -6;
    }

    /* 构建Dummy ARM关节目标 (dummy-auk 原生角度直接发送) */
    dummy_joint_pos_t pos;
    for (uint8_t i = 0; i < 6; i++) {
        pos.j[i] = target_joints[i];
    }

    /* 发送USB运动命令 */
    ret = dummy_arm_move_j(&pos, MOTION_DEFAULT_SPEED_DEG_S);
    if (ret != 0) {
        debug_print("[MOTION] USB move_j failed: ");
        debug_println(tag);
        motion_set_state(MOTION_ERROR, "USB move_j failed");
        return -3;
    }

    /* 强制等待到位，避免上层在运动未稳定时继续发新指令 */
    motion_set_state(MOTION_EXECUTING, tag);
    ret = motion_wait_settled(MOTION_SETTLE_TIMEOUT_MS, MOTION_SETTLE_TOL_DEG, MOTION_SETTLE_STABLE_NEED);
    if (ret != 0) {
        return ret;
    }

    kin_update_current_angles(&g_ik_solver, g_current_joints);
    motion_set_state(MOTION_DONE, tag);
    return 0;
}

/**
 * @brief 发送USB运动命令（非阻塞版）—— 不等待到位，立即返回
 */
static int motion_start_usb_move(const float target_joints[6], const char *tag)
{
    int ret = motion_check_joint_limits(target_joints);
    if (ret != 0) {
        motion_set_state(MOTION_ERROR, "joint out of limits");
        return ret;
    }

    if (!usb_cdc_is_ready()) {
        motion_set_state(MOTION_ERROR, "USB not connected");
        return -6;
    }

    dummy_joint_pos_t pos;
    for (uint8_t i = 0; i < 6U; i++) {
        pos.j[i] = target_joints[i];
    }

    ret = dummy_arm_move_j(&pos, MOTION_DEFAULT_SPEED_DEG_S);
    if (ret != 0) {
        motion_set_state(MOTION_ERROR, "USB move_j failed");
        return -4;
    }

    motion_set_state(MOTION_EXECUTING, tag);
    return 0; /* 立即返回，运动在Dummy ARM后台执行 */
}

static int motion_move_cartesian_internal(float x, float y, float z, bool is_relative)
{
    if (g_state == MOTION_EXECUTING) {
        debug_println("[MOTION] Busy!");
        return -1;
    }

    motion_set_state(MOTION_PLANNING, is_relative ? "move_by_xyz" : "move_to_xyz");

    /* 1. 查询当前关节角度 */
    dummy_joint_pos_t jpos;
    if (!usb_cdc_is_ready() || dummy_arm_get_joint_pos(&jpos) != 0) {
        debug_println("[MOTION] GETJPOS failed!");
        motion_set_state(MOTION_ERROR, "GETJPOS failed");
        return -6;
    }
    float cur_joints[6] = {jpos.j[0], jpos.j[1], jpos.j[2],
                           jpos.j[3], jpos.j[4], jpos.j[5]};
    memcpy(g_current_joints, cur_joints, sizeof(cur_joints));

    /* 2. FK: 当前关节 -> 当前T矩阵 (旋转矩阵精确,无Euler角损失) */
    float T_cur[4][4];
    kin_forward_kinematics(&g_ik_solver, cur_joints, T_cur);

    /* 3. 目标T: 保持旋转，仅修改位置 */
    float T_target[4][4];
    memcpy(T_target, T_cur, sizeof(T_target));
    if (is_relative) {
        T_target[0][3] += x;  /* mm */
        T_target[1][3] += y;
        T_target[2][3] += z;
        debug_print("[MOTION] REL dXYZ -> ");
    } else {
        T_target[0][3] = x;   /* mm */
        T_target[1][3] = y;
        T_target[2][3] = z;
        debug_print("[MOTION] ABS XYZ -> ");
    }

    debug_print_int((int)x);
    debug_print(", ");
    debug_print_int((int)y);
    debug_print(", ");
    debug_print_int((int)z);
    debug_println("");

    debug_print("[MOTION] MoveJ target: ");
    debug_print_int((int)T_target[0][3]);
    debug_print(", ");
    debug_print_int((int)T_target[1][3]);
    debug_print(", ");
    debug_print_int((int)T_target[2][3]);
    debug_println("");

    /* 4. 本地IK求解 + 多解筛选 */
    kin_update_current_angles(&g_ik_solver, cur_joints);
    float target_joints[6];
    int ret = kin_inverse_kinematics(&g_ik_solver, T_target, target_joints);
    if (ret != 0) {
        debug_println("[MOTION] IK failed - no solution!");
        motion_set_state(MOTION_ERROR, "IK failed");
        return -2;
    }

    ik_accept_level_t level = IK_ACCEPT_REJECT;
    float pos_err = 0.0f;
    float rot_err = 0.0f;
    float joint_cost = 0.0f;
    ret = motion_select_ik_candidate(T_target, cur_joints, target_joints,
                                     &level, &pos_err, &rot_err, &joint_cost);
    if (ret != 0 || level == IK_ACCEPT_REJECT) {
        char errbuf[128];
        snprintf(errbuf, sizeof(errbuf),
                 "[MOTION][ERROR] IK reject: pos=%.2fmm rot=%.3f cost=%.1f",
                 pos_err, rot_err, joint_cost);
        debug_println(errbuf);
        motion_set_state(MOTION_ERROR, "IK quality rejected");
        return -2;
    }

    if (level == IK_ACCEPT_SOFT) {
        char warnbuf[128];
        snprintf(warnbuf, sizeof(warnbuf),
                 "[MOTION][WARN] IK soft-pass: pos=%.2fmm rot=%.3f cost=%.1f",
                 pos_err, rot_err, joint_cost);
        debug_println(warnbuf);
    } else if (level == IK_ACCEPT_DEGRADE) {
        char warnbuf[128];
        snprintf(warnbuf, sizeof(warnbuf),
                 "[MOTION][WARN] IK degraded-pass: pos=%.2fmm rot=%.3f cost=%.1f",
                 pos_err, rot_err, joint_cost);
        debug_println(warnbuf);
    } else {
        char okbuf[128];
        snprintf(okbuf, sizeof(okbuf),
                 "[MOTION] IK hard-pass: pos=%.2fmm rot=%.3f cost=%.1f",
                 pos_err, rot_err, joint_cost);
        debug_println(okbuf);
    }

    /* 5. 发送关节目标 (绕过固件IK) */
    return motion_send_usb_move(target_joints, is_relative ? "dxyz" : "xyz");
}

/* ========== 公共API ========== */

void motion_init(void)
{
    debug_println("[MOTION] init (USB backend)...");

    /* 初始角度 = dummy-auk REST_POSE */
    static const float REST_POSE[6] = {0.0f, -73.0f, 180.0f, 0.0f, 0.0f, 0.0f};
    memcpy(g_current_joints, REST_POSE, sizeof(g_current_joints));

    /* 初始化IK求解器（限位来自关节配置） */
    motion_refresh_ik_limits();

    /* 初始化USB Dummy ARM命令层 */
    dummy_arm_init();

    /* 等待USB连接 (wait_ready内部会喂狗，不怕看门狗超时) */
    debug_println("[MOTION] Waiting for USB connection (5s)...");
    if (dummy_arm_wait_ready(5000)) {
        debug_println("[MOTION] USB connected");

        /* 设置INTERRUPTABLE模式 + 使能，带重试
         * SEQUENTIAL模式下运动命令阻塞到完成才返回"ok",
         * 超时后的延迟"ok"会污染后续命令的响应.
         * INTERRUPTABLE模式下只返回FIFO大小, 新命令可打断旧运动.
         * USB时序问题可能导致首次失败，重试3次. */
        {
            int init_ok = 0;
            for (int attempt = 0; attempt < 3; attempt++) {
                int r1 = dummy_arm_set_mode(DUMMY_MODE_INTERRUPTABLE);
                int r2 = dummy_arm_enable();

                debug_print("[MOTION] Init attempt ");
                debug_print_int(attempt);
                debug_print(": mode=");
                debug_print_int(r1);
                debug_print(" enable=");
                debug_print_int(r2);
                debug_println("");

                if (r1 == 0 && r2 == 0) {
                    init_ok = 1;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            if (!init_ok) {
                debug_println("[MOTION] WARNING: set_mode/enable failed after 3 attempts!");
            }
        }

        /* 同步关节位置 */
        motion_sync_from_usb();
        kin_update_current_angles(&g_ik_solver, g_current_joints);

        motion_set_state(MOTION_IDLE, "init done (USB)");
        debug_println("[MOTION] Initialized (USB Dummy ARM, INTERRUPTABLE mode)");
    } else {
        debug_println("[MOTION] USB not connected, continuing without arm");
        motion_set_state(MOTION_IDLE, "init done (no USB)");
    }
}

int motion_move_to_xyz(float x, float y, float z)
{
    return motion_move_cartesian_internal(x, y, z, false);
}

int motion_move_by_xyz(float dx, float dy, float dz)
{
    return motion_move_cartesian_internal(dx, dy, dz, true);
}

int motion_start_xyz(float x, float y, float z)
{
    /* 与 motion_move_cartesian_internal 相同的规划流程，
     * 但发送USB指令后立即返回（不等待到位）。*/
    if (g_state == MOTION_EXECUTING) {
        return -1;
    }

    motion_set_state(MOTION_PLANNING, "start_xyz");

    dummy_joint_pos_t jpos;
    if (!usb_cdc_is_ready() || dummy_arm_get_joint_pos(&jpos) != 0) {
        motion_set_state(MOTION_ERROR, "GETJPOS failed");
        return -6;
    }
    float cur_joints[6] = {jpos.j[0], jpos.j[1], jpos.j[2],
                           jpos.j[3], jpos.j[4], jpos.j[5]};
    memcpy(g_current_joints, cur_joints, sizeof(cur_joints));

    float T_cur[4][4];
    kin_forward_kinematics(&g_ik_solver, cur_joints, T_cur);

    float T_target[4][4];
    memcpy(T_target, T_cur, sizeof(T_target));
    T_target[0][3] = x;
    T_target[1][3] = y;
    T_target[2][3] = z;

    kin_update_current_angles(&g_ik_solver, cur_joints);
    float target_joints[6];
    int ret = kin_inverse_kinematics(&g_ik_solver, T_target, target_joints);
    if (ret != 0) {
        motion_set_state(MOTION_ERROR, "IK failed");
        return -2;
    }

    ik_accept_level_t level = IK_ACCEPT_REJECT;
    float pos_err = 0.0f, rot_err = 0.0f, joint_cost = 0.0f;
    ret = motion_select_ik_candidate(T_target, cur_joints, target_joints,
                                     &level, &pos_err, &rot_err, &joint_cost);
    if (ret != 0 || level == IK_ACCEPT_REJECT) {
        motion_set_state(MOTION_ERROR, "IK quality rejected");
        return -2;
    }

    return motion_start_usb_move(target_joints, "xyz");
}

int motion_start_joints(const float joints[6])
{
    if (g_state == MOTION_EXECUTING) {
        return -1;
    }
    motion_set_state(MOTION_PLANNING, "start_joints");
    return motion_start_usb_move(joints, "joints");
}

int motion_start_by_xyz(float dx, float dy, float dz)
{
    if (g_state == MOTION_EXECUTING) {
        return -1;
    }

    motion_set_state(MOTION_PLANNING, "start_by_xyz");

    dummy_joint_pos_t jpos;
    if (!usb_cdc_is_ready() || dummy_arm_get_joint_pos(&jpos) != 0) {
        motion_set_state(MOTION_ERROR, "GETJPOS failed");
        return -6;
    }
    float cur_joints[6] = {jpos.j[0], jpos.j[1], jpos.j[2],
                           jpos.j[3], jpos.j[4], jpos.j[5]};
    memcpy(g_current_joints, cur_joints, sizeof(cur_joints));

    float T_cur[4][4];
    kin_forward_kinematics(&g_ik_solver, cur_joints, T_cur);

    float T_target[4][4];
    memcpy(T_target, T_cur, sizeof(T_target));
    T_target[0][3] += dx;
    T_target[1][3] += dy;
    T_target[2][3] += dz;

    kin_update_current_angles(&g_ik_solver, cur_joints);
    float target_joints[6];
    int ret = kin_inverse_kinematics(&g_ik_solver, T_target, target_joints);
    if (ret != 0) {
        motion_set_state(MOTION_ERROR, "IK failed");
        return -2;
    }

    ik_accept_level_t level = IK_ACCEPT_REJECT;
    float pos_err = 0.0f, rot_err = 0.0f, joint_cost = 0.0f;
    ret = motion_select_ik_candidate(T_target, cur_joints, target_joints,
                                     &level, &pos_err, &rot_err, &joint_cost);
    if (ret != 0 || level == IK_ACCEPT_REJECT) {
        motion_set_state(MOTION_ERROR, "IK quality rejected");
        return -2;
    }

    return motion_start_usb_move(target_joints, "rel_xyz");
}

int motion_move_to_pose(float x, float y, float z,
                        float roll, float pitch, float yaw)
{
    /*
     * 完整位姿运动：
     * 直接组装笛卡尔坐标 → 通过 @ 命令发送给固件
     * roll/pitch/yaw (rad) 转换为 a/b/c (度)
     */
    if (g_state == MOTION_EXECUTING) {
        debug_println("[MOTION] Busy!");
        return -1;
    }

    motion_set_state(MOTION_PLANNING, "move_to_pose");

    /* 1. 组装笛卡尔目标 (位置mm, 姿态rad→度) */
    dummy_cart_pos_t target;
    target.x = x;
    target.y = y;
    target.z = z;
    target.a = roll  * (180.0f / M_PI);
    target.b = pitch * (180.0f / M_PI);
    target.c = yaw   * (180.0f / M_PI);

    /* 2. 打印目标 */
    debug_print("[MOTION] MoveL pose: ");
    debug_print_int((int)target.x);
    debug_print(", ");
    debug_print_int((int)target.y);
    debug_print(", ");
    debug_print_int((int)target.z);
    debug_println("");

    /* 3. 通过 @ 命令发送，固件内部做IK */
    int ret = dummy_arm_move_l(&target, MOTION_DEFAULT_SPEED_DEG_S);
    if (ret != 0) {
        debug_println("[MOTION] MoveL failed!");
        motion_set_state(MOTION_ERROR, "MoveL failed");
        return -3;
    }

    motion_set_state(MOTION_EXECUTING, "pose");
    ret = motion_wait_settled(MOTION_SETTLE_TIMEOUT_MS, MOTION_SETTLE_TOL_DEG, MOTION_SETTLE_STABLE_NEED);
    if (ret != 0) {
        return ret;
    }

    kin_update_current_angles(&g_ik_solver, g_current_joints);
    motion_set_state(MOTION_DONE, "pose");
    return 0;

#if 0  /* RA6M5端IK求解 — 备用 */
    float T_target[4][4];
    build_transform_matrix(T_target, x, y, z, roll, pitch, yaw);
    kin_update_current_angles(&g_ik_solver, g_current_joints);

    float target_joints[6];
    ret = kin_inverse_kinematics(&g_ik_solver, T_target, target_joints);
    if (ret != 0) {
        motion_set_state(MOTION_ERROR, "IK failed");
        return -2;
    }
    return motion_send_usb_move(target_joints, "pose");
#endif
}

int motion_move_to_joints(const float joints[6])
{
    if (g_state == MOTION_EXECUTING) {
        debug_println("[MOTION] Busy!");
        return -1;
    }

    motion_set_state(MOTION_PLANNING, "move_to_joints");

    /* 通过USB发送运动命令 (内含限位检查) */
    return motion_send_usb_move(joints, "joints");
}

motion_state_t motion_update(void)
{
    /* USB模式下不需要5ms轨迹插补
     * Dummy ARM自己处理轨迹规划和电机控制
     * 这里仅返回当前状态 */
    return g_state;
}

motion_state_t motion_get_state(void)
{
    return g_state;
}

void motion_stop(void)
{
    if (usb_cdc_is_ready()) {
        dummy_arm_stop();
        /* 急停后位置未知，从USB查询实际位置 */
        motion_sync_from_usb();
    }
    motion_set_state(MOTION_IDLE, "stopped");
    debug_println("[MOTION] Stopped");
}

void motion_set_current_joints(const float joints[6])
{
    memcpy(g_current_joints, joints, sizeof(g_current_joints));
    kin_update_current_angles(&g_ik_solver, joints);
}


void motion_get_current_joints(float joints[6])
{
    memcpy(joints, g_current_joints, sizeof(g_current_joints));
}

int motion_test_ik(float x, float y, float z)
{
    /* 1. 复制REST矩阵, 在世界坐标系下加偏移 */
    float T_target[4][4];
    memcpy(T_target, T_0_6_reset, sizeof(T_target));
    T_target[0][3] += x;
    T_target[1][3] += y;
    T_target[2][3] += z;

    /* 3. 更新IK求解器的当前角度 */
    kin_update_current_angles(&g_ik_solver, g_current_joints);

    /* 4. 求解逆运动学 (静默模式，不打印) */
    float target_joints[6];
    int ret = kin_inverse_kinematics(&g_ik_solver, T_target, target_joints);

    return ret;  /* 0=有解, 非0=无解 */
}

int motion_clear_stall(uint8_t joint_index)
{
    /* USB模式下没有CAN堵转概念
     * 但保留接口用于清除软件层错误状态 */
    (void)joint_index;

    if (g_state == MOTION_ERROR) {
        motion_set_state(MOTION_IDLE, "error cleared");
    }

    debug_println("[MOTION] Error state cleared");
    return 0;
}

/* ========== 兼容桩函数 ========== */

/** @brief 相对运动兼容接口 (visual_servo.c使用) */
int motion_move_relative(float dx, float dy, float dz)
{
    return motion_move_by_xyz(dx, dy, dz);
}

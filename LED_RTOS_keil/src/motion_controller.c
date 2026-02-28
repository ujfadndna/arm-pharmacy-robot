/**
 * @file    motion_controller.c
 * @brief   运动控制器实现 - 封装IK、轨迹规划、电机控制
 */

#include "motion_controller.h"
#include "kinematics.h"
#include "trajectory.h"
#include "motor_can.h"
#include "debug_uart.h"
#include <string.h>
#include <math.h>

/* CMSIS-DSP 矩阵运算加速 */
#include "arm_math_types.h"
#include "dsp/matrix_functions.h"

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* ========== 内部状态 ========== */
static kin_solver_t g_ik_solver;
static traj_planner_t g_planner;
static motion_state_t g_state = MOTION_IDLE;
static float g_current_joints[6] = {0};

/* 软件到电机的角度偏移量
 * 软件角度 = 电机角度 + 偏移量
 * 电机角度 = 软件角度 - 偏移量
 * 初始值对应ZERO的初始姿态 */
static float g_joint_offset[6] = {90.0f, 90.0f, -90.0f, 0.0f, 90.0f, 0.0f};

/* ========== 默认配置 ========== */

/* 关节限位 (度) - 来自ZERO项目的真实限位 */
static const kin_joint_limit_t DEFAULT_LIMITS[6] = {
    {0.0f,   360.0f},   /* J1 base:    0° ~ 360° */
    {90.0f,  180.0f},   /* J2 shoulder: 90° ~ 180° */
    {-90.0f, 90.0f},    /* J3 elbow:   -90° ~ 90° */
    {-90.0f, 90.0f},    /* J4 wrist1:  -90° ~ 90° */
    {0.0f,   90.0f},    /* J5 wrist2:   0° ~ 90° */
    {0.0f,   360.0f}    /* J6 wrist3:   0° ~ 360° */
};

/* 默认运动时间 (ms) */
#define DEFAULT_DURATION_MS  2000

/* ========== ZERO坐标系 ========== */

/**
 * T_0_6_reset - ZERO机械臂的初始姿态矩阵
 * 这是末端执行器的"零点"位置，用户输入的(x,y,z)是相对于这个位置的偏移
 * 来自 ZERO项目 robot.c
 */
static const float T_0_6_reset[4][4] = {
    {0.0f, -1.0f,  0.0f,   0.0f},
    {0.0f,  0.0f, -1.0f, -47.63f},
    {1.0f,  0.0f,  0.0f,  15.5f},
    {0.0f,  0.0f,  0.0f,   1.0f}
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

/* ========== 公共API ========== */

void motion_init(void)
{
    debug_println("[MOTION] init step 1...");
    /* 初始化IK求解器 */
    kin_solver_init(&g_ik_solver, DEFAULT_LIMITS);

    debug_println("[MOTION] init step 2...");
    /* 初始化轨迹规划器 (5ms周期，与GPT0定时器一致) */
    traj_init(&g_planner, 5);

    debug_println("[MOTION] init step 3...");
    /* 设置速度/加速度限制 */
    traj_limits_t limits = {
        .max_vel = {90.0f, 90.0f, 90.0f, 120.0f, 120.0f, 120.0f},
        .max_acc = {180.0f, 180.0f, 180.0f, 240.0f, 240.0f, 240.0f}
    };
    traj_set_limits(&g_planner, &limits);

    debug_println("[MOTION] init step 4...");
    /* 初始化CAN电机驱动 */
    motor_can_init(NULL);

    debug_println("[MOTION] init step 5...");
    /* 初始化状态 */
    g_state = MOTION_IDLE;

    /* 设置初始关节角度 - 对应ZERO的复位姿态T_0_6_reset
     * 来自ZERO robot.c: {90, 90, -90, 0, 90, 0} */
    g_current_joints[0] = 90.0f;   /* J1 */
    g_current_joints[1] = 90.0f;   /* J2 */
    g_current_joints[2] = -90.0f;  /* J3 */
    g_current_joints[3] = 0.0f;    /* J4 */
    g_current_joints[4] = 90.0f;   /* J5 */
    g_current_joints[5] = 0.0f;    /* J6 */

    debug_println("[MOTION] init step 6...");
    kin_update_current_angles(&g_ik_solver, g_current_joints);

    debug_println("[MOTION] Initialized (ZERO coords)");
}

int motion_move_to_xyz(float x, float y, float z)
{
    /*
     * 使用ZERO坐标系：
     * 用户输入的(x,y,z)是相对于T_0_6_reset的偏移量
     * T_target = T_0_6_reset * T_offset
     */
    if (g_state == MOTION_EXECUTING) {
        debug_println("[MOTION] Busy!");
        return -1;
    }

    g_state = MOTION_PLANNING;

    /* 1. 构建偏移矩阵 */
    float T_offset[4][4];
    build_translation_matrix(T_offset, x, y, z);

    /* 2. 计算目标矩阵: T_target = T_0_6_reset * T_offset */
    float T_target[4][4];
    matrix_multiply_4x4(T_0_6_reset, T_offset, T_target);

    /* 3. 打印目标位置 (调试) */
    debug_print("[MOTION] Target xyz: ");
    debug_print_int((int)T_target[0][3]);
    debug_print(", ");
    debug_print_int((int)T_target[1][3]);
    debug_print(", ");
    debug_print_int((int)T_target[2][3]);
    debug_println("");

    /* 4. 更新IK求解器的当前角度 */
    kin_update_current_angles(&g_ik_solver, g_current_joints);

    /* 5. 求解逆运动学 */
    float target_joints[6];
    int ret = kin_inverse_kinematics(&g_ik_solver, T_target, target_joints);
    if (ret != 0) {
        debug_println("[MOTION] IK failed - no solution!");
        g_state = MOTION_ERROR;
        return -2;
    }

    /* 6. 打印IK结果 */
    debug_print("[MOTION] IK result: ");
    for (int i = 0; i < 6; i++) {
        debug_print_int((int)target_joints[i]);
        debug_print(" ");
    }
    debug_println("deg");

    /* 7. 清空并添加轨迹点 */
    traj_clear(&g_planner);
    ret = traj_add_point(&g_planner, target_joints, DEFAULT_DURATION_MS,
                         TRAJ_INTERP_MINIMUM_JERK);
    if (ret != 0) {
        debug_println("[MOTION] Trajectory add failed!");
        g_state = MOTION_ERROR;
        return -3;
    }

    /* 8. 开始轨迹执行 */
    ret = traj_start(&g_planner, g_current_joints);
    if (ret != 0) {
        debug_println("[MOTION] Trajectory start failed!");
        g_state = MOTION_ERROR;
        return -4;
    }

    g_state = MOTION_EXECUTING;
    debug_println("[MOTION] Executing...");
    return 0;
}

int motion_move_to_pose(float x, float y, float z,
                        float roll, float pitch, float yaw)
{
    if (g_state == MOTION_EXECUTING) {
        debug_println("[MOTION] Busy!");
        return -1;
    }

    g_state = MOTION_PLANNING;

    /* 1. 构建目标变换矩阵 */
    float T_target[4][4];
    build_transform_matrix(T_target, x, y, z, roll, pitch, yaw);

    /* 2. 更新IK求解器的当前角度 */
    kin_update_current_angles(&g_ik_solver, g_current_joints);

    /* 3. 求解逆运动学 */
    float target_joints[6];
    int ret = kin_inverse_kinematics(&g_ik_solver, T_target, target_joints);
    if (ret != 0) {
        debug_println("[MOTION] IK failed - no solution!");
        g_state = MOTION_ERROR;
        return -2;
    }

    /* 4. 打印IK结果 */
    debug_print("[MOTION] IK result: ");
    for (int i = 0; i < 6; i++) {
        debug_print_int((int)target_joints[i]);
        debug_print(" ");
    }
    debug_println("deg");

    /* 5. 清空并添加轨迹点 */
    traj_clear(&g_planner);
    ret = traj_add_point(&g_planner, target_joints, DEFAULT_DURATION_MS,
                         TRAJ_INTERP_MINIMUM_JERK);
    if (ret != 0) {
        debug_println("[MOTION] Trajectory add failed!");
        g_state = MOTION_ERROR;
        return -3;
    }

    /* 6. 开始轨迹执行 */
    ret = traj_start(&g_planner, g_current_joints);
    if (ret != 0) {
        debug_println("[MOTION] Trajectory start failed!");
        g_state = MOTION_ERROR;
        return -4;
    }

    g_state = MOTION_EXECUTING;
    debug_println("[MOTION] Executing...");
    return 0;
}

int motion_move_to_joints(const float joints[6])
{
    if (g_state == MOTION_EXECUTING) {
        debug_println("[MOTION] Busy!");
        return -1;
    }

    g_state = MOTION_PLANNING;

    /* 直接添加关节目标 */
    traj_clear(&g_planner);
    int ret = traj_add_point(&g_planner, joints, DEFAULT_DURATION_MS,
                             TRAJ_INTERP_MINIMUM_JERK);
    if (ret != 0) {
        g_state = MOTION_ERROR;
        return -1;
    }

    ret = traj_start(&g_planner, g_current_joints);
    if (ret != 0) {
        g_state = MOTION_ERROR;
        return -2;
    }

    g_state = MOTION_EXECUTING;
    debug_println("[MOTION] Executing joints...");
    return 0;
}

motion_state_t motion_update(void)
{
    if (g_state != MOTION_EXECUTING) {
        return g_state;
    }

    /* 轨迹插补一步 */
    float q_interp[6];
    traj_state_t traj_state = traj_step(&g_planner, q_interp);

    if (traj_state == TRAJ_STATE_RUNNING) {
        /* 计算电机角度 = 软件角度 - 偏移量 */
        float motor_angles[6];
        for (int i = 0; i < 6; i++) {
            motor_angles[i] = q_interp[i] - g_joint_offset[i];
        }

        /* 发送到电机 (同步模式，内部已包含广播触发) */
        motor_can_send_sync_position(motor_angles, 100.0f);  /* 100 RPM，提高速度 */

        /* 更新当前角度 (软件侧) */
        memcpy(g_current_joints, q_interp, sizeof(g_current_joints));
    }
    else if (traj_state == TRAJ_STATE_DONE) {
        g_state = MOTION_DONE;
        debug_println("[MOTION] Done");
    }
    else if (traj_state == TRAJ_STATE_ERROR) {
        g_state = MOTION_ERROR;
        debug_println("[MOTION] Error");
    }

    return g_state;
}

motion_state_t motion_get_state(void)
{
    return g_state;
}

void motion_stop(void)
{
    traj_stop(&g_planner);
    motor_can_emergency_stop();
    g_state = MOTION_IDLE;
    debug_println("[MOTION] Stopped");
}

void motion_set_current_joints(const float joints[6])
{
    memcpy(g_current_joints, joints, sizeof(g_current_joints));
    kin_update_current_angles(&g_ik_solver, joints);
}

void motion_set_joint_offsets(const float offsets[6])
{
    memcpy(g_joint_offset, offsets, sizeof(g_joint_offset));
}

void motion_get_current_joints(float joints[6])
{
    memcpy(joints, g_current_joints, sizeof(g_current_joints));
}

int motion_test_ik(float x, float y, float z)
{
    /* 1. 构建偏移矩阵 */
    float T_offset[4][4];
    build_translation_matrix(T_offset, x, y, z);

    /* 2. 计算目标矩阵: T_target = T_0_6_reset * T_offset */
    float T_target[4][4];
    matrix_multiply_4x4(T_0_6_reset, T_offset, T_target);

    /* 3. 更新IK求解器的当前角度 */
    kin_update_current_angles(&g_ik_solver, g_current_joints);

    /* 4. 求解逆运动学 (静默模式，不打印) */
    float target_joints[6];
    int ret = kin_inverse_kinematics(&g_ik_solver, T_target, target_joints);

    return ret;  /* 0=有解, 非0=无解 */
}

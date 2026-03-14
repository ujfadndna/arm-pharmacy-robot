/**
 * @file    motion_controller.c
 * @brief   运动控制器实现 - 封装IK、轨迹规划、电机控制
 */

#include "motion_controller.h"
#include "kinematics.h"
#include "trajectory.h"
#include "motor_ctrl_step.h"
#include "degradation.h"
/* #include "debug_uart.h" */  /* 暂时注释掉，避免debug_printf冲突 */
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

/* ========== 内部状态 ========== */
static kin_solver_t g_ik_solver;
static traj_planner_t g_planner;
static motion_state_t g_state = MOTION_IDLE;
static float g_current_joints[6] = {0};
static float g_cmd_speed_deg_s[6] = {30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f};
static uint8_t g_feedback_query_joint = 0U;
static uint8_t g_feedback_query_divider = 0U;
static uint32_t g_timeout_check_counter = 0U;
static uint8_t g_stall_count[MOTOR_CTRL_STEP_NUM_JOINTS] = {0};  /* 堵转连续检测计数器 */

/* 软件到电机的角度偏移量
 * 软件角度 = 电机角度 + 偏移量
 * 电机角度 = 软件角度 - 偏移量
 * 对应dummy-auk的REST_POSE = {0, -73, 180, 0, 0, 0}
 * 使得软件零点对应机械臂初始姿态 */
static float g_joint_offset[6] = {0.0f, -73.0f, 180.0f, 0.0f, 0.0f, 0.0f};

/* ========== 默认配置 ========== */

/* 默认运动时间 (ms) */
#define DEFAULT_DURATION_MS  2000
#define MOTION_CTRL_PERIOD_MS 5U                        /* 200Hz */
#define MOTION_QUERY_PERIOD_TICKS 2U                    /* 10ms轮询一轴反馈 (2 * 5ms = 10ms, 单关节反馈周期60ms) */
#define MOTION_DEFAULT_SPEED_DEG_S 30.0f                /* 对齐dummy-auk默认关节速度 */
#define MOTION_STALL_CONFIRM_COUNT 2U                   /* 堵转连续检测阈值 (2次 x 5ms = 10ms) */
#define MOTION_STATE_BYTE_STALL_BIT 0x10U               /* state_byte堵转标志位 (bit4) */

/* ========== ZERO坐标系 ========== */

/**
 * T_0_6_reset - 机械臂初始姿态矩阵（末端执行器零点位置）
 *
 * 对应dummy-auk的REST_POSE = {0, -73, 180, 0, 0, 0}（度）
 * FK计算结果：末端位置 (109.98mm, 68.62mm, 379.00mm)
 * 计算脚本：tools/calc_fk_reset.py
 */
static const float T_0_6_reset[4][4] = {
    {  0.9563f,  -0.0000f,   0.2924f,   0.1100f},
    {  0.2924f,   0.0000f,  -0.9563f,   0.0686f},
    {  0.0000f,   1.0000f,   0.0000f,   0.3790f},
    {  0.0000f,   0.0000f,   0.0000f,   1.0000f}
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
    kin_joint_limit_t limits[MOTOR_CTRL_STEP_NUM_JOINTS];

    for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
        /* IK在“软件角度域”规划，驱动在“电机角度域”执行，二者通过offset映射 */
        limits[i].min_angle = joint_config[i].angle_min + g_joint_offset[i];
        limits[i].max_angle = joint_config[i].angle_max + g_joint_offset[i];
    }

    kin_solver_init(&g_ik_solver, limits);
    kin_update_current_angles(&g_ik_solver, g_current_joints);
}

static float motion_software_to_motor_angle(uint8_t joint_index, float software_angle, float * out_motor_rev)
{
    float motor_angle = software_angle - g_joint_offset[joint_index];
    float mapped = joint_config[joint_index].inverse ? -motor_angle : motor_angle;

    if (out_motor_rev != NULL) {
        *out_motor_rev = mapped / 360.0f * joint_config[joint_index].reduction;
    }

    return motor_angle;
}

static float motion_feedback_to_software_angle(uint8_t joint_index, float feedback_motor_rev)
{
    float motor_angle = feedback_motor_rev / joint_config[joint_index].reduction * 360.0f;
    if (joint_config[joint_index].inverse) {
        motor_angle = -motor_angle;
    }
    return motor_angle + g_joint_offset[joint_index];
}

static void motion_sync_from_feedback(void)
{
    for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
        motor_ctrl_step_feedback_t feedback = {0};
        if (!motor_ctrl_step_get_feedback(i, &feedback) || !feedback.online) {
            continue;
        }
        g_current_joints[i] = motion_feedback_to_software_angle(i, feedback.feedback_motor_rev);
    }
}

static void motion_feedback_tick(void)
{
    (void) motor_ctrl_step_poll_rx();

    g_feedback_query_divider++;
    if (g_feedback_query_divider >= MOTION_QUERY_PERIOD_TICKS) {
        g_feedback_query_divider = 0U;
        (void) motor_ctrl_step_query_position(g_feedback_query_joint);
        g_feedback_query_joint++;
        if (g_feedback_query_joint >= MOTOR_CTRL_STEP_NUM_JOINTS) {
            g_feedback_query_joint = 0U;
        }
    }
}

/**
 * @brief 检查关节堵转状态
 * @param joint_index 关节索引 (0-5)
 * @return 1=堵转, 0=正常, -1=参数错误
 */
static int motion_check_stall_status(uint8_t joint_index)
{
    if (joint_index >= MOTOR_CTRL_STEP_NUM_JOINTS) {
        return -1;
    }

    motor_ctrl_step_feedback_t feedback = {0};
    if (!motor_ctrl_step_get_feedback(joint_index, &feedback) || !feedback.online) {
        return 0;
    }

    /* 检查state_byte的bit4 (0x10) */
    return ((feedback.state_byte & MOTION_STATE_BYTE_STALL_BIT) != 0U) ? 1 : 0;
}

static int motion_plan_and_start(const float target_joints[6], const char * tag)
{
    /* 统一关节限位检查 - 覆盖所有运动路径（关节空间和笛卡尔空间）
     * 防止IK解超限导致碰撞 */
    for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
        /* 将软件角度转换为电机角度进行限位检查 */
        float motor_angle = target_joints[i] - g_joint_offset[i];

        if ((motor_angle < joint_config[i].angle_min) || (motor_angle > joint_config[i].angle_max)) {
            debug_printf("[MOTION] ERROR: Joint %d out of limit: target=%.2f deg, limit=[%.2f, %.2f]\r\n",
                         i, motor_angle, joint_config[i].angle_min, joint_config[i].angle_max);
            g_state = MOTION_ERROR;
            return -5;  /* 新增错误码：关节超限 */
        }
    }

    float max_weighted_delta = 0.0f;
    for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
        float delta = fabsf(target_joints[i] - g_current_joints[i]);
        float weighted = delta * joint_config[i].reduction;
        if (weighted > max_weighted_delta) {
            max_weighted_delta = weighted;
        }
    }

    if (max_weighted_delta < 1e-3f) {
        for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
            g_cmd_speed_deg_s[i] = MOTION_DEFAULT_SPEED_DEG_S;
        }
    } else {
        /* 对齐dummy-auk: time = max(|dq| * reduction) / jointSpeed */
        float motion_time_s = max_weighted_delta / MOTION_DEFAULT_SPEED_DEG_S;
        if (motion_time_s < 0.05f) {
            motion_time_s = 0.05f;
        }

        for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
            float delta = target_joints[i] - g_current_joints[i];
            float motor_rev_s = fabsf(delta * joint_config[i].reduction / motion_time_s * 0.1f);
            float speed_deg_s = motor_rev_s * 360.0f / joint_config[i].reduction;

            if (speed_deg_s < 1.0f) {
                speed_deg_s = 1.0f;
            } else if (speed_deg_s > 180.0f) {
                speed_deg_s = 180.0f;
            }

            g_cmd_speed_deg_s[i] = speed_deg_s;
        }
    }

    traj_clear(&g_planner);

    if (motor_ctrl_step_set_enable(MOTOR_CTRL_STEP_ALL_JOINTS, true) != 0) {
        debug_println("[MOTION] WARN: enable all joints failed before execution");
    }

    int ret = traj_add_point(&g_planner, target_joints, DEFAULT_DURATION_MS, TRAJ_INTERP_MINIMUM_JERK);
    if (ret != 0) {
        debug_print("[MOTION] Trajectory add failed: ");
        debug_println(tag);
        motion_set_state(MOTION_ERROR, "traj_add_point failed");
        return -3;
    }

    ret = traj_start(&g_planner, g_current_joints);
    if (ret != 0) {
        debug_print("[MOTION] Trajectory start failed: ");
        debug_println(tag);
        motion_set_state(MOTION_ERROR, "traj_start failed");
        return -4;
    }

    motion_set_state(MOTION_EXECUTING, tag);
    return 0;
}

/* ========== 公共API ========== */

void motion_init(void)
{
    debug_println("[MOTION] init step 1...");
    /* 初始化状态，软件角度初值与偏移量一致（电机角度=0） */
    for (uint8_t i = 0; i < 6U; i++) {
        g_current_joints[i] = g_joint_offset[i];
    }

    /* 初始化IK求解器（限位来自dummy-auk joint_config） */
    motion_refresh_ik_limits();

    debug_println("[MOTION] init step 2...");
    /* 初始化轨迹规划器 (5ms周期，与GPT0定时器一致) */
    traj_init(&g_planner, MOTION_CTRL_PERIOD_MS);

    debug_println("[MOTION] init step 3...");
    /* 设置速度/加速度限制 */
    traj_limits_t limits = {
        .max_vel = {90.0f, 90.0f, 90.0f, 120.0f, 120.0f, 120.0f},
        .max_acc = {180.0f, 180.0f, 180.0f, 240.0f, 240.0f, 240.0f}
    };
    traj_set_limits(&g_planner, &limits);

    debug_println("[MOTION] init step 4...");
    /* 初始化CtrlStep电机驱动 */
    if (motor_ctrl_step_init() != 0) {
        motion_set_state(MOTION_ERROR, "motor_ctrl_step_init failed");
        return;
    }
    if (motor_ctrl_step_set_enable(MOTOR_CTRL_STEP_ALL_JOINTS, true) != 0) {
        debug_println("[MOTION] WARN: enable all joints failed");
    }
    g_feedback_query_joint = 0U;
    g_feedback_query_divider = 0U;

    debug_println("[MOTION] init step 5: wait CAN bus stable...");
    /* 等待CAN总线稳定 */
    vTaskDelay(pdMS_TO_TICKS(100));

    debug_println("[MOTION] init step 6: query all joint positions...");
    /* 轮询查询所有6个关节位置 */
    for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
        motor_ctrl_step_query_position(i);
        vTaskDelay(pdMS_TO_TICKS(20));  /* 等待响应 */
        motor_ctrl_step_poll_rx();      /* 接收响应 */
    }

    debug_println("[MOTION] init step 7: verify joint online status...");
    /* 验证在线状态 */
    uint8_t online_count = 0;
    motor_ctrl_step_feedback_t feedback;
    for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
        if (motor_ctrl_step_get_feedback(i, &feedback)) {
            if (feedback.online) {
                online_count++;
                debug_printf("[MOTION] Joint %d: ONLINE (angle=%.2f deg)\r\n",
                            i, feedback.feedback_angle_deg);
            } else {
                debug_printf("[MOTION] Joint %d: OFFLINE\r\n", i);
            }
        } else {
            debug_printf("[MOTION] Joint %d: NO FEEDBACK\r\n", i);
        }
    }

    debug_printf("[MOTION] Online joints: %d/%d\r\n", online_count, MOTOR_CTRL_STEP_NUM_JOINTS);

    /* 至少4个关节在线才继续 */
    if (online_count < 4) {
        debug_printf("[MOTION] ERROR: Insufficient online joints (%d < 4)\r\n", online_count);
        degradation_handle_fault(FAULT_CAN_TIMEOUT, 0xFF);
        motion_set_state(MOTION_ERROR, "insufficient online joints");
        return;
    }

    debug_println("[MOTION] init step 8...");
    motion_set_state(MOTION_IDLE, "init done");

    debug_println("[MOTION] init step 9...");
    motion_feedback_tick();
    motion_sync_from_feedback();
    kin_update_current_angles(&g_ik_solver, g_current_joints);

    debug_println("[MOTION] Initialized (dummy-auk CtrlStep)");
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

    motion_set_state(MOTION_PLANNING, "move_to_xyz");

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
        motion_set_state(MOTION_ERROR, "IK failed");
        return -2;
    }

    /* 6. 打印IK结果 */
    debug_print("[MOTION] IK result: ");
    for (int i = 0; i < 6; i++) {
        debug_print_int((int)target_joints[i]);
        debug_print(" ");
    }
    debug_println("deg");

    /* 7. 轨迹规划并进入执行态 */
    return motion_plan_and_start(target_joints, "xyz");
}

int motion_move_to_pose(float x, float y, float z,
                        float roll, float pitch, float yaw)
{
    if (g_state == MOTION_EXECUTING) {
        debug_println("[MOTION] Busy!");
        return -1;
    }

    motion_set_state(MOTION_PLANNING, "move_to_pose");

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
        motion_set_state(MOTION_ERROR, "IK failed");
        return -2;
    }

    /* 4. 打印IK结果 */
    debug_print("[MOTION] IK result: ");
    for (int i = 0; i < 6; i++) {
        debug_print_int((int)target_joints[i]);
        debug_print(" ");
    }
    debug_println("deg");

    /* 5. 轨迹规划并进入执行态 */
    return motion_plan_and_start(target_joints, "pose");
}

int motion_move_to_joints(const float joints[6])
{
    if (g_state == MOTION_EXECUTING) {
        debug_println("[MOTION] Busy!");
        return -1;
    }

    motion_set_state(MOTION_PLANNING, "move_to_joints");

    /* 先进行限位检查（软件角度域） */
    for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
        float motor_angle = joints[i] - g_joint_offset[i];
        if ((motor_angle < joint_config[i].angle_min) || (motor_angle > joint_config[i].angle_max)) {
            debug_print("[MOTION] Joint limit: J");
            debug_print_int((int) i);
            debug_println(" out of range");
            motion_set_state(MOTION_ERROR, "joint target out of limits");
            return -2;
        }
    }

    int ret = motion_plan_and_start(joints, "joints");
    if (ret == -3) {
        return -1;
    }
    if (ret == -4) {
        return -2;
    }
    return ret;
}

motion_state_t motion_update(void)
{
    if (g_state != MOTION_EXECUTING) {
        return g_state;
    }

    /* 每5ms轮询一次RX，且每20ms轮询一个关节位置 */
    motion_feedback_tick();

    /* CAN超时检测 - 每100ms检查一次 (20个tick @ 5ms) */
    g_timeout_check_counter++;
    if (g_timeout_check_counter >= 20U) {
        g_timeout_check_counter = 0U;

        uint8_t timeout_mask = motor_ctrl_step_check_timeout();
        if (timeout_mask != 0U) {
            /* 有关节超时，触发降级 */
            for (uint8_t i = 0U; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
                if (timeout_mask & (1U << i)) {
                    debug_print("[MOTION] CAN timeout on J");
                    debug_print_int((int)i);
                    debug_println("");

                    /* 调用降级模块处理超时故障 */
                    degradation_handle_fault(FAULT_CAN_TIMEOUT, i);
                }
            }
        }
    }

    /* 堵转检测 - 每个tick检查所有关节 */
    for (uint8_t i = 0U; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
        /* 跳过已禁用的关节 */
        if (degradation_is_joint_disabled(i)) {
            continue;
        }

        int stall_status = motion_check_stall_status(i);
        if (stall_status == 1) {
            /* 堵转计数+1 */
            g_stall_count[i]++;

            /* 连续检测到堵转超过阈值才触发 */
            if (g_stall_count[i] >= MOTION_STALL_CONFIRM_COUNT) {
                /* 获取state_byte用于日志 */
                motor_ctrl_step_feedback_t feedback = {0};
                motor_ctrl_step_get_feedback(i, &feedback);

                debug_print("[MOTION] Joint J");
                debug_print_int((int)i);
                debug_print(" stalled! state_byte=0x");
                debug_print_hex(feedback.state_byte);
                debug_println("");

                /* 停止所有电机 */
                motor_ctrl_step_set_enable(MOTOR_CTRL_STEP_ALL_JOINTS, false);

                /* 调用降级模块处理堵转故障 */
                degrade_level_t level = degradation_handle_fault(FAULT_MOTOR_STALL, i);

                /* 设置错误状态 */
                debug_print("[MOTION] Degradation level: ");
                debug_println(degradation_level_str(level));
                motion_set_state(MOTION_ERROR, "motor stall detected");

                /* 停止轨迹规划 */
                traj_stop(&g_planner);

                return g_state;
            }
        } else {
            /* 堵转状态消失，重置计数 */
            g_stall_count[i] = 0U;
        }
    }

    /* 轨迹插补一步 */
    float q_interp[6];
    traj_state_t traj_state = traj_step(&g_planner, q_interp);

    if (traj_state == TRAJ_STATE_RUNNING) {
        /* 发送dummy-auk CtrlStep位置+速度命令 */
        for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
            float motor_rev = 0.0f;
            float motor_angle = motion_software_to_motor_angle(i, q_interp[i], &motor_rev);
            (void) motor_rev; /* 便于调试时快速观察转换链路 */

            int ret = motor_ctrl_step_set_position_with_speed(i, motor_angle, g_cmd_speed_deg_s[i]);
            if (ret != 0) {
                debug_print("[MOTION] CtrlStep cmd failed, J");
                debug_print_int((int) i);
                debug_print(" ret=");
                debug_print_int(ret);
                debug_println("");
                motion_set_state(MOTION_ERROR, "set_position_with_speed failed");
                return g_state;
            }
        }

        /* 更新当前角度 (软件侧) */
        memcpy(g_current_joints, q_interp, sizeof(g_current_joints));
    }
    else if (traj_state == TRAJ_STATE_DONE) {
        motion_sync_from_feedback();
        motion_set_state(MOTION_DONE, "trajectory done");
    }
    else if (traj_state == TRAJ_STATE_ERROR) {
        motion_set_state(MOTION_ERROR, "trajectory error");
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
    (void) motor_ctrl_step_set_enable(MOTOR_CTRL_STEP_ALL_JOINTS, false);
    motion_set_state(MOTION_IDLE, "stopped");
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
    motion_refresh_ik_limits();
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

int motion_clear_stall(uint8_t joint_index)
{
    /* 清除电机堵转保护 */
    int ret = motor_ctrl_step_clear_clog(joint_index);
    if (ret != 0) {
        debug_print("[MOTION] Clear stall failed, ret=");
        debug_print_int(ret);
        debug_println("");
        return ret;
    }

    /* 重置堵转计数器 */
    if (joint_index == MOTOR_CTRL_STEP_ALL_JOINTS) {
        /* 清除所有关节 */
        for (uint8_t i = 0U; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++) {
            g_stall_count[i] = 0U;
        }
        debug_println("[MOTION] Cleared stall protection for all joints");
    } else if (joint_index < MOTOR_CTRL_STEP_NUM_JOINTS) {
        /* 清除单个关节 */
        g_stall_count[joint_index] = 0U;
        debug_print("[MOTION] Cleared stall protection for J");
        debug_print_int((int)joint_index);
        debug_println("");
    } else {
        return -1;  /* 参数错误 */
    }

    return 0;
}

/* ========== 兼容桩函数 ========== */

/** @brief 相对运动 (visual_servo.c使用) - 待实现 */
int motion_move_relative(float dx, float dy, float dz)
{
    (void)dx; (void)dy; (void)dz;
    return -1;  /* TODO: 实现相对运动 */
}

/**
 ******************************************************************************
 * @file    dummy_arm_cmd.h
 * @brief   Dummy机械臂USB命令接口
 * @note    通过USB CDC发送ASCII协议命令控制机械臂
 *          协议格式:
 *            控制命令: !START, !STOP, !HOME, !RESET, !DISABLE
 *            关节运动: >j1,j2,j3,j4,j5,j6[,speed]
 *            直线运动: @x,y,z,a,b,c[,speed]
 *            查询命令: #GETJPOS, #GETLPOS, #CMDMODE <mode>
 ******************************************************************************
 */

#ifndef DUMMY_ARM_CMD_H_
#define DUMMY_ARM_CMD_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 命令模式 ========== */
typedef enum {
    DUMMY_MODE_SEQUENTIAL     = 1,  /* 等运动完成再响应 */
    DUMMY_MODE_INTERRUPTABLE  = 2,  /* 立即响应，可打断 */
    DUMMY_MODE_TRAJECTORY     = 3,  /* 连续轨迹模式 */
    DUMMY_MODE_TUNING         = 4   /* 电机调参模式 */
} dummy_cmd_mode_t;

/* ========== 关节位置 ========== */
typedef struct {
    float j[6];     /* 6轴关节角度 (度) */
} dummy_joint_pos_t;

/* ========== 笛卡尔位姿 ========== */
typedef struct {
    float x, y, z;  /* 位置 (mm) */
    float a, b, c;  /* 姿态 (度) */
} dummy_cart_pos_t;

/* ========== 默认超时 ========== */
#define DUMMY_CMD_TIMEOUT       1000    /* 普通命令超时 (ms) */
#define DUMMY_MOTION_TIMEOUT    2000    /* 运动命令超时 (ms) - 必须小于看门狗2.7s */
#define DUMMY_BLOCKING_TIMEOUT  30000   /* HOME/RESET超时 (ms) - 使用分段等待+喂狗，支持长运动 */

/* ========== 公共接口 ========== */

/**
 * @brief 初始化命令层 (内部调用usb_cdc_init)
 * @return 0=成功, -1=失败
 */
int dummy_arm_init(void);

/**
 * @brief 等待USB连接就绪
 * @param timeout_ms 超时时间
 * @return true=已连接, false=超时
 */
bool dummy_arm_wait_ready(uint32_t timeout_ms);

/* ---- 控制命令 ---- */

/** @brief 使能电机 (!START) */
int dummy_arm_enable(void);

/** @brief 失能电机 (!DISABLE) */
int dummy_arm_disable(void);

/** @brief 急停 (!STOP) */
int dummy_arm_stop(void);

/** @brief 回零 (!HOME → 0,0,90,0,0,0) */
int dummy_arm_home(void);

/** @brief 休息位 (!RESET → 0,-73,180,0,0,0) */
int dummy_arm_rest(void);

/** @brief 设置命令模式 (#CMDMODE) */
int dummy_arm_set_mode(dummy_cmd_mode_t mode);

/* ---- 运动命令 ---- */

/**
 * @brief 关节运动
 * @param pos 6轴目标角度 (度)
 * @param speed 速度 (度/秒), 0=使用默认速度
 * @return 0=成功, -1=失败
 */
int dummy_arm_move_j(const dummy_joint_pos_t *pos, float speed);

/**
 * @brief 直线运动 (笛卡尔空间)
 * @param pos 目标位姿
 * @param speed 速度, 0=使用默认速度
 * @return 0=成功, -1=失败
 */
int dummy_arm_move_l(const dummy_cart_pos_t *pos, float speed);

/**
 * @brief 6轴关节运动 (便捷版)
 */
int dummy_arm_move_joints(float j1, float j2, float j3, float j4, float j5, float j6, float speed);

/* ---- 查询命令 ---- */

/**
 * @brief 查询当前关节位置
 * @param out 输出关节角度
 * @return 0=成功, -1=失败
 */
int dummy_arm_get_joint_pos(dummy_joint_pos_t *out);

/**
 * @brief 查询当前笛卡尔位姿
 * @param out 输出位姿
 * @return 0=成功, -1=失败
 */
int dummy_arm_get_cart_pos(dummy_cart_pos_t *out);

/* ---- 电机参数 ---- */

/**
 * @brief 设置单轴DCE增益
 * @param node 电机节点 (1-6)
 * @param kp Kp增益
 */
int dummy_arm_set_kp(uint8_t node, float kp);
int dummy_arm_set_ki(uint8_t node, float ki);
int dummy_arm_set_kd(uint8_t node, float kd);

/** @brief 重启电机节点 */
int dummy_arm_reboot_node(uint8_t node);

/**
 * @brief 发送原始ASCII命令
 * @param cmd 命令字符串 (不含\r\n)
 * @param resp 响应缓冲区 (可为NULL)
 * @param resp_max 缓冲区大小
 * @param timeout_ms 超时
 * @return 响应长度, -1=失败
 */
int dummy_arm_raw_cmd(const char *cmd, char *resp, uint16_t resp_max, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* DUMMY_ARM_CMD_H_ */

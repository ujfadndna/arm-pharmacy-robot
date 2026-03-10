/**
 ******************************************************************************
 * @file    motor_sca.h
 * @brief   INNFOS SCA智能执行器CAN驱动接口
 * @note    适配SCA 1.x协议，标准11-bit CAN帧，1Mbps
 *          移植自 dummy-auk/firmware/dummy-ref-core-fw/Robot/actuators/mintasca/
 ******************************************************************************
 */

#ifndef MOTOR_SCA_H_
#define MOTOR_SCA_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 ========== */
#define MOTOR_SCA_NUM_JOINTS    6       /* 关节数量 */
#define MOTOR_SCA_BASE_ID       1       /* 执行器起始ID (1~6) */

/* 减速比（输出轴转数 = 关节角度/360 * 减速比）
 * 默认1.0 = 直驱，按实际SCA型号修改 */
#define MOTOR_SCA_REDUCTION_RATIO   1.0f

/* 到位判断阈值（圈数），默认2度 */
#define MOTOR_SCA_REACHED_THRESHOLD (2.0f / 360.0f)

/* ========== 电机状态（与motor_can.h兼容） ========== */
#ifndef MOTOR_STATE_T_DEFINED
#define MOTOR_STATE_T_DEFINED
typedef struct {
    float current_angle;    /* 当前角度(度) */
    float target_angle;     /* 目标角度(度) */
    uint8_t status;         /* 状态字节 */
    bool reached;           /* 是否到位 */
    bool error;             /* 是否故障 */
} motor_state_t;
#endif

/* ========== 公共接口 ========== */

/** @brief 初始化SCA驱动（打开CAN外设，创建信号量） */
void motor_sca_init(void);

/** @brief 使能所有执行器（切换到位置模式 + 上电） */
int  motor_sca_enable_all(void);

/** @brief 失能所有执行器 */
int  motor_sca_disable_all(void);

/**
 * @brief 发送6轴同步位置指令
 * @param joint_angles 6个关节目标角度(度)
 * @param max_speed    最大速度(RPM)，当前版本暂不使用
 * @return 0=成功, -1=发送失败
 */
int  motor_sca_send_sync_position(const float joint_angles[MOTOR_SCA_NUM_JOINTS],
                                   float max_speed);

/** @brief 急停（立即失能所有执行器） */
void motor_sca_emergency_stop(void);

/** @brief 检查所有执行器是否到位 */
bool motor_sca_all_reached(void);

/**
 * @brief 获取执行器状态
 * @param id    执行器ID (0-5，对应关节1-6)
 * @param state 输出状态
 */
void motor_sca_get_state(uint8_t id, motor_state_t *state);

/**
 * @brief Ping执行器（发送心跳，等待响应）
 * @param id 执行器ID (0-5)
 * @return 0=有响应, -1=超时
 */
int  motor_sca_ping(uint8_t id);

/**
 * @brief CAN接收回调（在canfd_callback中调用）
 * @param can_id CAN帧ID（= 执行器ID，1-6）
 * @param data   数据
 * @param len    长度
 */
void motor_sca_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len);

/** @brief TX完成通知（在canfd_callback的TX_COMPLETE分支中调用） */
void motor_sca_tx_complete_notify(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SCA_H_ */

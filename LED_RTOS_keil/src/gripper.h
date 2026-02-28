/**
 ******************************************************************************
 * @file    gripper.h
 * @brief   夹爪CAN控制接口
 * @note    协议来自jiazhao工程，CAN ID 0x700
 ******************************************************************************
 */

#ifndef GRIPPER_H_
#define GRIPPER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 夹爪配置 ========== */
#define GRIPPER_CAN_ID          0x700   /* 夹爪CAN ID (扩展帧) */
#define GRIPPER_CMD_SERVO       0xAE    /* 舵机控制命令 */
#define GRIPPER_CMD_VACUUM      0xAD    /* 吸盘控制命令 */

#define GRIPPER_ANGLE_OPEN      90      /* 张开角度 */
#define GRIPPER_ANGLE_CLOSE     0       /* 闭合角度 */

/* ========== 公共接口 ========== */

/**
 * @brief 初始化夹爪控制
 * @note  需要先调用motor_can_init()初始化CAN外设
 */
void gripper_init(void);

/**
 * @brief 张开夹爪
 * @return 0=成功, -1=发送失败
 */
int gripper_open(void);

/**
 * @brief 闭合夹爪
 * @return 0=成功, -1=发送失败
 */
int gripper_close(void);

/**
 * @brief 设置夹爪角度
 * @param angle 舵机角度 (0-180)
 * @return 0=成功, -1=发送失败
 */
int gripper_set_angle(uint8_t angle);

/**
 * @brief 开启吸盘
 * @return 0=成功, -1=发送失败
 */
int vacuum_on(void);

/**
 * @brief 关闭吸盘
 * @return 0=成功, -1=发送失败
 */
int vacuum_off(void);

/**
 * @brief CAN接收回调 (处理夹爪返回数据)
 * @param can_id CAN帧ID
 * @param data 数据
 * @param len 长度
 * @note  由canfd_callback调用
 */
void gripper_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_H_ */

/**
 ******************************************************************************
 * @file    motor_can_dummy.h
 * @brief   Dummy机械臂CAN驱动接口 (STM32F103电机控制器)
 * @note    适配dummy-auk机械臂的CAN协议
 *          标准11位帧: CAN_ID = (motor_id << 7) | cmd_id
 ******************************************************************************
 */

#ifndef MOTOR_CAN_DUMMY_H_
#define MOTOR_CAN_DUMMY_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 ========== */
#define MOTOR_CAN_NUM_JOINTS    6       /* 关节数量 */
#define MOTOR_CAN_BAUD_RATE     1000000 /* 1Mbps */

/* ========== CAN协议定义 ========== */

/* 实时控制命令 (0x01-0x0F) */
#define CMD_ENABLE              0x01    /* 使能/失能电机 */
#define CMD_TRIGGER_CALIB       0x02    /* 触发编码器校准 */
#define CMD_SET_CURRENT         0x03    /* 设置电流(力矩模式) */
#define CMD_SET_VELOCITY        0x04    /* 设置速度 */
#define CMD_SET_POSITION        0x05    /* 设置位置 */
#define CMD_SET_POS_TIME        0x06    /* 设置位置+时间 */
#define CMD_SET_POS_VEL         0x07    /* 设置位置+速度限制 */

/* 配置命令 (0x10-0x1F) */
#define CMD_SET_NODE_ID         0x11    /* 设置节点ID */
#define CMD_SET_CURRENT_LIMIT   0x12    /* 设置电流限制 */
#define CMD_SET_VELOCITY_LIMIT  0x13    /* 设置速度限制 */
#define CMD_SET_ACCELERATION    0x14    /* 设置加速度 */
#define CMD_SET_HOME_OFFSET     0x15    /* 设置当前位置为零点 */
#define CMD_SET_AUTO_ENABLE     0x16    /* 设置上电自动使能 */
#define CMD_SET_DCE_KP          0x17    /* 设置DCE Kp增益 */
#define CMD_SET_DCE_KV          0x18    /* 设置DCE Kv增益 */
#define CMD_SET_DCE_KI          0x19    /* 设置DCE Ki增益 */
#define CMD_SET_DCE_KD          0x1A    /* 设置DCE Kd增益 */
#define CMD_SET_STALL_PROTECT   0x1B    /* 设置堵转保护 */

/* 查询命令 (0x20-0x2F) */
#define CMD_GET_CURRENT         0x21    /* 获取电流 */
#define CMD_GET_VELOCITY        0x22    /* 获取速度 */
#define CMD_GET_POSITION        0x23    /* 获取位置 */
#define CMD_GET_HOME_OFFSET     0x24    /* 获取零点偏移 */
#define CMD_GET_TEMPERATURE     0x25    /* 获取温度 */

/* 系统命令 (0x7D-0x7F) */
#define CMD_ENABLE_TEMP_MON     0x7D    /* 使能温度监控 */
#define CMD_ERASE_CONFIG        0x7E    /* 擦除配置 */
#define CMD_REBOOT              0x7F    /* 重启 */

/* ========== 电机状态 ========== */
typedef struct {
    float current_angle;    /* 当前角度(度) */
    float target_angle;     /* 目标角度(度) */
    float velocity;         /* 当前速度(度/秒) */
    float current;          /* 当前电流(A) */
    float temperature;      /* 温度(°C) */
    uint8_t finish_flag;    /* 运动完成标志 (1=完成, 0=运动中) */
    bool error;             /* 故障标志 */
} motor_state_t;

/* ========== 驱动器配置 ========== */
typedef struct {
    uint8_t base_addr;          /* 起始地址 (1-15) */
    float max_speed_rps;        /* 最大速度(转/秒) */
    float max_acceleration;     /* 最大加速度(转/秒²) */
    float current_limit;        /* 电流限制(A) */
    bool use_standard_frame;    /* 使用标准帧(11位) */
} motor_can_config_t;

/* ========== 控制模式 ========== */
typedef enum {
    MODE_STOP = 0,              /* 停止 */
    MODE_COMMAND_POSITION = 1,  /* 位置控制 */
    MODE_COMMAND_VELOCITY = 2,  /* 速度控制 */
    MODE_COMMAND_CURRENT = 3,   /* 电流控制 */
    MODE_COMMAND_Trajectory = 4,/* 轨迹模式 */
    MODE_PWM_POSITION = 5,      /* PWM位置控制 */
    MODE_PWM_VELOCITY = 6,      /* PWM速度控制 */
    MODE_PWM_CURRENT = 7,       /* PWM电流控制 */
    MODE_STEP_DIR = 8           /* 步进/方向接口 */
} motor_mode_t;

/* ========== 公共接口 ========== */

/**
 * @brief 初始化电机CAN驱动
 * @param config 配置参数，NULL使用默认值
 */
void motor_can_init(const motor_can_config_t *config);

/**
 * @brief 使能/失能电机
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param enable true=使能, false=失能
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_enable(uint8_t joint_id, bool enable);

/**
 * @brief 设置位置 (位置控制模式)
 * @param joint_id 关节ID (0-5)
 * @param position 目标位置(度)
 * @param ack_request 是否请求反馈
 * @return 0=成功, -1=发送失败
 * @note 内部转换: 度 → 转数 (360度 = 1转)
 */
int motor_can_set_position(uint8_t joint_id, float position, bool ack_request);

/**
 * @brief 设置位置+速度限制
 * @param joint_id 关节ID (0-5)
 * @param position 目标位置(度)
 * @param max_velocity 最大速度(度/秒)
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_position_velocity(uint8_t joint_id, float position, float max_velocity);

/**
 * @brief 设置速度 (速度控制模式)
 * @param joint_id 关节ID (0-5)
 * @param velocity 目标速度(度/秒)
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_velocity(uint8_t joint_id, float velocity);

/**
 * @brief 设置电流 (力矩控制模式)
 * @param joint_id 关节ID (0-5)
 * @param current 目标电流(A)
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_current(uint8_t joint_id, float current);

/**
 * @brief 查询位置
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @return 0=成功, -1=发送失败
 * @note 位置会通过CAN回调更新到 motor_state_t
 */
int motor_can_get_position(uint8_t joint_id);

/**
 * @brief 查询温度
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @return 0=成功, -1=发送失败
 */
int motor_can_get_temperature(uint8_t joint_id);

/**
 * @brief 设置当前位置为零点
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @return 0=成功, -1=发送失败
 * @note 此命令会将电机编码器当前位置清零，掉电不丢失
 */
int motor_can_set_home_offset(uint8_t joint_id);

/**
 * @brief 触发编码器校准
 * @param joint_id 关节ID (0-5)
 * @return 0=成功, -1=发送失败
 * @note 电机会自动旋转一圈进行校准
 */
int motor_can_trigger_calibration(uint8_t joint_id);

/**
 * @brief 设置电流限制
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param current_limit 电流限制(A)
 * @param save_to_eeprom 是否保存到EEPROM
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_current_limit(uint8_t joint_id, float current_limit, bool save_to_eeprom);

/**
 * @brief 设置速度限制
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param velocity_limit 速度限制(度/秒)
 * @param save_to_eeprom 是否保存到EEPROM
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_velocity_limit(uint8_t joint_id, float velocity_limit, bool save_to_eeprom);

/**
 * @brief 设置加速度
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param acceleration 加速度(度/秒²)
 * @param save_to_eeprom 是否保存到EEPROM
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_acceleration(uint8_t joint_id, float acceleration, bool save_to_eeprom);

/**
 * @brief 设置DCE控制器参数
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param kp, kv, ki, kd DCE增益参数
 * @param save_to_eeprom 是否保存到EEPROM
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_dce_gains(uint8_t joint_id, int32_t kp, int32_t kv, int32_t ki, int32_t kd, bool save_to_eeprom);

/**
 * @brief 使能堵转保护
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param enable true=使能, false=失能
 * @param save_to_eeprom 是否保存到EEPROM
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_stall_protect(uint8_t joint_id, bool enable, bool save_to_eeprom);

/**
 * @brief 使能温度监控
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @return 0=成功, -1=发送失败
 * @note 使能后电机会以1Hz频率自动上报温度
 */
int motor_can_enable_temp_monitor(uint8_t joint_id);

/**
 * @brief 获取电机状态
 * @param joint_id 关节ID (0-5)
 * @param state 输出状态
 */
void motor_can_get_state(uint8_t joint_id, motor_state_t *state);

/**
 * @brief 检查所有电机是否到位
 * @return true=所有电机到位, false=有电机在运动
 */
bool motor_can_all_reached(void);

/**
 * @brief CAN接收回调 (在CAN中断中调用)
 * @param can_id CAN帧ID
 * @param data 数据
 * @param len 长度
 */
void motor_can_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len);

/**
 * @brief 电机通信超时检测
 * @return 超时的电机ID (0-5), -1=无超时
 * @note 在主循环中定期调用 (建议每100ms)
 *       超时阈值: 500ms无响应视为失联
 */
int motor_can_check_timeout(void);

/**
 * @brief 获取电机最后通信时间
 * @param joint_id 关节ID (0-5)
 * @return 最后收到响应的时间戳 (ms)
 */
uint32_t motor_can_get_last_rx_time(uint8_t joint_id);

/**
 * @brief 重置电机通信超时计时器
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 */
void motor_can_reset_timeout(uint8_t joint_id);

/**
 * @brief 获取CAN调试信息
 * @param tx_count 输出TX完成计数
 * @param error_flags 输出错误标志位
 */
void motor_can_get_debug_info(uint32_t *tx_count, uint32_t *error_flags);

/**
 * @brief 清除堵转保护 (兼容接口)
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @return 0=成功
 */
int motor_can_clear_stall(uint8_t joint_id);

/**
 * @brief 急停 (兼容接口)
 */
void motor_can_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CAN_DUMMY_H_ */

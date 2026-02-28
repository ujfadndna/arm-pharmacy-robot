/**
 ******************************************************************************
 * @file    motor_can.h
 * @brief   ZDT闭环步进电机CAN驱动接口
 * @note    适配Emm_V5固件协议(0xFD位置模式)，配合Minimum Jerk轨迹规划
 ******************************************************************************
 */

#ifndef MOTOR_CAN_H_
#define MOTOR_CAN_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 ========== */
#define MOTOR_CAN_NUM_JOINTS    6       /* 关节数量 */
#define MOTOR_CAN_CHECKSUM      0x6B    /* 默认校验字节 */
#define MOTOR_CAN_SPEED_SCALE   1.0f    /* 速度放大倍数 (Emm: RPM直接传) */

/* Emm固件位置参数 (默认16细分) */
#define MOTOR_CAN_PULSES_PER_REV  3200.0f   /* 每圈脉冲数 */
#define MOTOR_CAN_DEG_TO_PULSES   (MOTOR_CAN_PULSES_PER_REV / 360.0f)  /* 角度→脉冲系数 ≈ 8.889 */
#define MOTOR_CAN_DEFAULT_ACC     10    /* 默认加速度 (0-255) */

/* ========== 驱动参数表偏移量 (说明书第61-64页) ========== */
#define PARAM_BLOCK_SIZE        32      /* 参数表总长度 */
#define OFFSET_MPLYER           7       /* 细分插补开关 (0=关, 1=开) */
#define OFFSET_LPFILTER         9       /* 低通滤波等级 (0-7) */
#define OFFSET_S_POSTDP         22      /* 高精度模式 (0=0.1°, 1=0.01°) */

/* ========== 电机状态 ========== */
typedef struct {
    float current_angle;    /* 当前角度(度) */
    float target_angle;     /* 目标角度(度) */
    uint8_t status;         /* 状态字节 */
    bool reached;           /* 是否到位 */
    bool error;             /* 是否故障 */
} motor_state_t;

/* ========== 驱动器配置 ========== */
typedef struct {
    uint8_t base_addr;          /* 起始地址 (默认1) */
    float max_speed_rpm;        /* 最大速度(RPM) */
    bool use_high_precision;    /* 使用高精度模式(0.01度) */
    bool use_extended_frame;    /* 使用扩展帧 */
} motor_can_config_t;

/* ========== 公共接口 ========== */

/**
 * @brief 初始化电机CAN驱动
 * @param config 配置参数，NULL使用默认值
 */
void motor_can_init(const motor_can_config_t *config);

/**
 * @brief 发送6轴同步位置指令 (直通限速模式)
 * @param joint_angles 6个关节目标角度(度)
 * @param max_speed 最大速度(RPM)，0=使用默认值
 * @return 0=成功, -1=发送失败
 * @note 配合Minimum Jerk轨迹，每10ms调用一次
 */
int motor_can_send_sync_position(const float joint_angles[MOTOR_CAN_NUM_JOINTS],
                                  float max_speed);

/**
 * @brief 发送单轴位置指令
 * @param joint_id 关节ID (0-5)
 * @param angle 目标角度(度)
 * @param speed 速度(RPM)
 * @param sync 是否等待同步触发
 */
int motor_can_send_position(uint8_t joint_id, float angle, float speed, bool sync);

/**
 * @brief 发送同步触发广播
 * @note 触发所有等待同步的电机同时运动
 */
void motor_can_trigger_sync(void);

/**
 * @brief 发送急停指令
 */
void motor_can_emergency_stop(void);

/**
 * @brief 使能/失能所有电机
 */
void motor_can_set_enable(bool enable);

/**
 * @brief 获取电机状态
 * @param joint_id 关节ID (0-5)
 * @param state 输出状态
 */
void motor_can_get_state(uint8_t joint_id, motor_state_t *state);

/**
 * @brief 检查所有电机是否到位
 */
bool motor_can_all_reached(void);

/**
 * @brief 设置电机加速度
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param acc 加速度 (单位: 步/秒²), 推荐范围 500-5000
 * @note 较高加速度 = 更快响应, 但可能导致振动
 */
int motor_can_set_acceleration(uint8_t joint_id, uint16_t acc);

/**
 * @brief 设置电机 PID 参数
 * @param joint_id 关节ID (0-5)
 * @param kp 位置环 Kp (0-65535)
 * @param kv 速度环 Kp (0-65535)
 * @param ki 速度环 Ki (0-65535)
 * @note 调参建议: 先调 Kv 到不振动, 再调 Kp 减少稳态误差
 */
int motor_can_set_pid(uint8_t joint_id, uint16_t kp, uint16_t kv, uint16_t ki);

/**
 * @brief 主动查询电机状态
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @return 0=成功, -1=发送失败
 * @note 状态会通过CAN回调更新到 motor_state_t
 */
int motor_can_read_status(uint8_t joint_id);

/**
 * @brief 读取电机堵转保护状态
 * @param joint_id 关节ID (0-5)
 * @return 0=正常, 1=堵转保护触发
 */
int motor_can_get_stall_status(uint8_t joint_id);

/**
 * @brief 清除电机堵转保护
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 */
int motor_can_clear_stall(uint8_t joint_id);

/**
 * @brief 读取位置误差 (Minimum Jerk 调试用)
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @return 0=成功, -1=发送失败
 * @note 误差值会通过CAN回调更新，用于监控轨迹跟踪精度
 */
int motor_can_read_position_error(uint8_t joint_id);

/**
 * @brief 设置电流限制
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param ma_limit 电流限制值 (mA)
 * @return 0=成功, -1=发送失败
 */
int motor_can_set_current_limit(uint8_t joint_id, uint16_t ma_limit);

/**
 * @brief 将电机当前位置设为零点
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @note 此命令会将电机编码器当前位置清零，掉电不丢失
 *       用于手动校准：将机械臂摆到零位后执行此命令
 */
int motor_can_reset_position_to_zero(uint8_t joint_id);

/**
 * @brief 电机回零 (寻找零点)
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param speed 回零速度 (RPM * 10)
 * @param direction 回零方向 (0=正向, 1=反向)
 * @return 0=成功, -1=发送失败
 * @note 电机会以指定速度向指定方向运动，直到触发零点开关或堵转
 */
int motor_can_go_home(uint8_t joint_id, uint16_t speed, uint8_t direction);

/**
 * @brief 配置电机为高速模式
 * @note 适合快速响应场景，可能牺牲一些平滑度
 *       加速度=3000, Kp=2000, Kv=1500, Ki=500
 */
void motor_can_config_fast_mode(void);

/**
 * @brief 配置电机为平稳模式
 * @note 适合需要平滑运动的场景，响应稍慢
 *       加速度=1500, Kp=1000, Kv=1000, Ki=300
 */
void motor_can_config_smooth_mode(void);

/* ========== 状态读取函数 ========== */

/**
 * @brief 读取电机实时位置
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @return 0=成功, -1=发送失败
 * @note 位置值会通过CAN回调更新到 motor_state_t.current_angle
 *       使用 0x36 指令，比 0x43 批量读取更简单可靠
 */
int motor_can_read_position(uint8_t joint_id);

/* ========== 驱动参数配置 (Read-Modify-Write) ========== */

/**
 * @brief 设置细分插补开关
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param enable 0=关闭, 1=开启
 * @return 0=成功, -1=发送失败, -2=读取超时
 * @note 使用 Read-Modify-Write 方式，通过 0x42/0x48 指令
 *       开启插补可让电机在低速段更平滑
 */
int motor_can_set_mplyer(uint8_t joint_id, uint8_t enable);

/**
 * @brief 设置低通滤波等级
 * @param joint_id 关节ID (0-5), 0xFF=所有电机
 * @param level 滤波等级 (0=关闭/最低, 1-7=低到高)
 * @return 0=成功, -1=发送失败, -2=读取超时
 * @note 使用 Read-Modify-Write 方式，通过 0x42/0x48 指令
 *       Minimum Jerk 建议设为 0 或 1，减少相位滞后
 */
int motor_can_set_lpfilter(uint8_t joint_id, uint8_t level);

/**
 * @brief 配置 Minimum Jerk 优化模式
 * @return 0=成功, 负数=失败
 * @note 设置: MPlyer=1 (开启插补) + LPFilter=0 (最低滤波) + S_PosTDP=1 (高精度)
 *       适合 Minimum Jerk 轨迹规划，减少相位滞后和低速抖动
 *       参数不保存到EEPROM，掉电后恢复出厂设置
 */
int motor_can_config_minimum_jerk_mode(void);

/**
 * @brief 配置 Minimum Jerk 优化模式 (扩展版)
 * @param save_to_eeprom true=保存到EEPROM(永久), false=不保存(掉电丢失)
 * @return 0=成功, 负数=失败
 * @note 调试阶段建议 save_to_eeprom=false，最终部署时设为 true
 *       EEPROM 有写入寿命限制（约10万次），避免频繁调用
 */
int motor_can_config_minimum_jerk_mode_ex(bool save_to_eeprom);

/**
 * @brief CAN接收回调 (在CAN中断中调用)
 * @param can_id CAN帧ID
 * @param data 数据
 * @param len 长度
 */
void motor_can_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len);

/**
 * @brief CAN通信测试 (ping电机)
 * @param joint_id 关节ID (0-5)
 * @return 0=成功(收到响应), -1=超时无响应
 * @note 发送0x3A读取状态命令，等待电机返回
 */
int motor_can_ping(uint8_t joint_id);

/**
 * @brief 获取CAN调试信息
 * @param tx_count 输出TX完成计数
 * @param error_flags 输出错误标志位
 *        bit0: ERR_WARNING
 *        bit1: ERR_PASSIVE
 *        bit2: ERR_BUS_OFF
 *        bit3: ERR_BUS_LOCK
 *        bit4: ERR_CHANNEL
 *        bit5: ERR_GLOBAL
 */
void motor_can_get_debug_info(uint32_t *tx_count, uint32_t *error_flags);

/**
 * @brief 清除CAN错误标志
 */
void motor_can_clear_error_flags(void);

/* ========== 超时检测 ========== */

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
 * @note 在电机使能或系统初始化后调用
 */
void motor_can_reset_timeout(uint8_t joint_id);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CAN_H_ */

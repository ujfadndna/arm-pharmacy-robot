/**
 ******************************************************************************
 * @file    motor_can_dummy.c
 * @brief   Dummy机械臂CAN驱动实现
 * @note    适配dummy-auk机械臂的CAN协议
 *          保留原有的发送流控和超时检测机制
 ******************************************************************************
 */

#include "motor_can_dummy.h"
#include "hal_data.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <math.h>

/* ========== 内部配置 ========== */
#define CAN_TX_MAILBOX_COUNT    4       /* TXMB 0-3 */
#define CAN_TX_RETRY_MAX        100     /* 最大重试次数 */
#define CAN_TX_RETRY_DELAY_US   50      /* 重试间隔 (微秒) */
#define MOTOR_TIMEOUT_MS        500     /* 电机通信超时阈值 (ms) */

/* 单位转换常数 */
#define STEPS_PER_REV           51200.0f    /* 每转步数 (200*256) */
#define DEG_TO_REV              (1.0f/360.0f)  /* 度转转数 */
#define REV_TO_DEG              360.0f         /* 转数转度 */

/* ========== 内部状态 ========== */
static motor_can_config_t g_config = {
    .base_addr = 1,
    .max_speed_rps = 30.0f,
    .max_acceleration = 100.0f,
    .current_limit = 1.0f,
    .use_standard_frame = true
};

static motor_state_t g_motor_states[MOTOR_CAN_NUM_JOINTS] = {0};
static bool g_initialized = false;
static uint8_t g_tx_mailbox_idx = 0;

/* 发送完成信号量 */
static SemaphoreHandle_t g_tx_sem = NULL;
static StaticSemaphore_t g_tx_sem_buffer;
static volatile uint8_t g_tx_pending = 0;
static volatile uint32_t g_tx_complete_count = 0;
static volatile uint32_t g_can_error_flags = 0;

/* 超时检测 */
static volatile uint32_t g_last_rx_time[MOTOR_CAN_NUM_JOINTS] = {0};
static volatile bool g_timeout_enabled = false;

/* ========== 内部函数声明 ========== */
static int CAN_Transmit(uint32_t can_id, const uint8_t *data, uint8_t len);
static void delay_us(uint32_t us);
static uint32_t get_tick_ms(void);

/* ========== 微秒延时 ========== */
static void delay_us(uint32_t us)
{
    /* 基于200MHz主频的粗略延时 */
    volatile uint32_t count = us * 40;
    while (count--) {
        __NOP();
    }
}

/* ========== 获取系统时间戳 (ms) ========== */
static uint32_t get_tick_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* ========== 底层CAN发送接口 (带流控) ========== */

/**
 * @brief CAN发送函数 (优化版：信号量等待 + Mailbox轮询)
 * @param can_id  CAN ID (标准帧11-bit)
 * @param data    数据指针
 * @param len     数据长度 (1-8)
 * @return 0=成功, -1=失败
 */
static int CAN_Transmit(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    can_frame_t frame = {
        .id               = can_id,
        .id_mode          = g_config.use_standard_frame ? CAN_ID_MODE_STANDARD : CAN_ID_MODE_EXTENDED,
        .type             = CAN_FRAME_TYPE_DATA,
        .data_length_code = len,
        .options          = 0
    };
    memcpy(frame.data, data, len);

    /* 尝试多个TX Mailbox */
    for (uint32_t retry = 0; retry < CAN_TX_RETRY_MAX; retry++) {
        /* 轮询尝试每个Mailbox */
        for (uint8_t mb = 0; mb < CAN_TX_MAILBOX_COUNT; mb++) {
            uint8_t mailbox = (g_tx_mailbox_idx + mb) % CAN_TX_MAILBOX_COUNT;

            fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, mailbox, &frame);

            if (FSP_SUCCESS == err) {
                /* 成功，更新下次起始Mailbox */
                g_tx_mailbox_idx = (mailbox + 1) % CAN_TX_MAILBOX_COUNT;
                g_tx_pending++;
                return 0;
            }
        }

        /* 所有Mailbox都忙，等待发送完成信号量 (最多1ms) */
        if (g_tx_sem != NULL) {
            xSemaphoreTake(g_tx_sem, pdMS_TO_TICKS(1));
        } else {
            delay_us(CAN_TX_RETRY_DELAY_US);
        }
    }

    /* 超时失败 */
    return -1;
}

/* ========== 公共接口实现 ========== */

/**
 * @brief 初始化电机CAN驱动
 */
void motor_can_init(const motor_can_config_t *config)
{
    if (config != NULL) {
        memcpy(&g_config, config, sizeof(motor_can_config_t));
    }

    /* 创建发送完成信号量 */
    if (g_tx_sem == NULL) {
        g_tx_sem = xSemaphoreCreateBinaryStatic(&g_tx_sem_buffer);
    }

    /* 初始化电机状态 */
    memset((void*)g_motor_states, 0, sizeof(g_motor_states));
    memset((void*)g_last_rx_time, 0, sizeof(g_last_rx_time));

    g_initialized = true;
}

/**
 * @brief 使能/失能电机
 */
int motor_can_set_enable(uint8_t joint_id, bool enable)
{
    uint8_t data[8] = {0};
    uint32_t enable_val = enable ? 1 : 0;
    memcpy(data, &enable_val, sizeof(uint32_t));

    if (joint_id == 0xFF) {
        /* 广播到所有电机 */
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_ENABLE;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_ENABLE;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 设置位置 (位置控制模式)
 */
int motor_can_set_position(uint8_t joint_id, float position, bool ack_request)
{
    uint8_t data[8] = {0};

    /* 度 → 转数 */
    float position_rev = position * DEG_TO_REV;
    memcpy(data, &position_rev, sizeof(float));
    data[4] = ack_request ? 1 : 0;

    uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_POSITION;
    return CAN_Transmit(can_id, data, 8);
}

/**
 * @brief 设置位置+速度限制
 */
int motor_can_set_position_velocity(uint8_t joint_id, float position, float max_velocity)
{
    uint8_t data[8] = {0};

    /* 度 → 转数 */
    float position_rev = position * DEG_TO_REV;
    float velocity_rps = max_velocity * DEG_TO_REV;

    memcpy(data, &position_rev, sizeof(float));
    memcpy(data + 4, &velocity_rps, sizeof(float));

    uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_POS_VEL;
    return CAN_Transmit(can_id, data, 8);
}

/**
 * @brief 设置速度 (速度控制模式)
 */
int motor_can_set_velocity(uint8_t joint_id, float velocity)
{
    uint8_t data[8] = {0};

    /* 度/秒 → 转/秒 */
    float velocity_rps = velocity * DEG_TO_REV;
    memcpy(data, &velocity_rps, sizeof(float));

    uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_VELOCITY;
    return CAN_Transmit(can_id, data, 8);
}

/**
 * @brief 设置电流 (力矩控制模式)
 */
int motor_can_set_current(uint8_t joint_id, float current)
{
    uint8_t data[8] = {0};
    memcpy(data, &current, sizeof(float));

    uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_CURRENT;
    return CAN_Transmit(can_id, data, 8);
}

/**
 * @brief 查询位置
 */
int motor_can_get_position(uint8_t joint_id)
{
    uint8_t data[8] = {0};

    if (joint_id == 0xFF) {
        /* 查询所有电机 */
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_GET_POSITION;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_GET_POSITION;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 查询温度
 */
int motor_can_get_temperature(uint8_t joint_id)
{
    uint8_t data[8] = {0};

    if (joint_id == 0xFF) {
        /* 查询所有电机 */
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_GET_TEMPERATURE;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_GET_TEMPERATURE;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 设置当前位置为零点
 */
int motor_can_set_home_offset(uint8_t joint_id)
{
    uint8_t data[8] = {0};

    if (joint_id == 0xFF) {
        /* 所有电机 */
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_SET_HOME_OFFSET;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_HOME_OFFSET;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 触发编码器校准
 */
int motor_can_trigger_calibration(uint8_t joint_id)
{
    uint8_t data[8] = {0};
    uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_TRIGGER_CALIB;
    return CAN_Transmit(can_id, data, 8);
}

/**
 * @brief 设置电流限制
 */
int motor_can_set_current_limit(uint8_t joint_id, float current_limit, bool save_to_eeprom)
{
    uint8_t data[8] = {0};
    memcpy(data, &current_limit, sizeof(float));
    data[4] = save_to_eeprom ? 1 : 0;

    if (joint_id == 0xFF) {
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_SET_CURRENT_LIMIT;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_CURRENT_LIMIT;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 设置速度限制
 */
int motor_can_set_velocity_limit(uint8_t joint_id, float velocity_limit, bool save_to_eeprom)
{
    uint8_t data[8] = {0};

    /* 度/秒 → 转/秒 */
    float velocity_rps = velocity_limit * DEG_TO_REV;
    memcpy(data, &velocity_rps, sizeof(float));
    data[4] = save_to_eeprom ? 1 : 0;

    if (joint_id == 0xFF) {
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_SET_VELOCITY_LIMIT;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_VELOCITY_LIMIT;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 设置加速度
 */
int motor_can_set_acceleration(uint8_t joint_id, float acceleration, bool save_to_eeprom)
{
    uint8_t data[8] = {0};

    /* 度/秒² → 转/秒² */
    float acceleration_rps2 = acceleration * DEG_TO_REV;
    memcpy(data, &acceleration_rps2, sizeof(float));
    data[4] = save_to_eeprom ? 1 : 0;

    if (joint_id == 0xFF) {
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_SET_ACCELERATION;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_ACCELERATION;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 设置DCE控制器参数
 */
int motor_can_set_dce_gains(uint8_t joint_id, int32_t kp, int32_t kv, int32_t ki, int32_t kd, bool save_to_eeprom)
{
    uint8_t data[8] = {0};
    data[4] = save_to_eeprom ? 1 : 0;

    /* 分别发送4个增益参数 */
    uint16_t can_id_base = (g_config.base_addr + joint_id) << 7;

    /* Kp */
    memcpy(data, &kp, sizeof(int32_t));
    if (CAN_Transmit(can_id_base | CMD_SET_DCE_KP, data, 8) != 0) return -1;

    /* Kv */
    memcpy(data, &kv, sizeof(int32_t));
    if (CAN_Transmit(can_id_base | CMD_SET_DCE_KV, data, 8) != 0) return -1;

    /* Ki */
    memcpy(data, &ki, sizeof(int32_t));
    if (CAN_Transmit(can_id_base | CMD_SET_DCE_KI, data, 8) != 0) return -1;

    /* Kd */
    memcpy(data, &kd, sizeof(int32_t));
    if (CAN_Transmit(can_id_base | CMD_SET_DCE_KD, data, 8) != 0) return -1;

    return 0;
}

/**
 * @brief 使能堵转保护
 */
int motor_can_set_stall_protect(uint8_t joint_id, bool enable, bool save_to_eeprom)
{
    uint8_t data[8] = {0};
    uint32_t enable_val = enable ? 1 : 0;
    memcpy(data, &enable_val, sizeof(uint32_t));
    data[4] = save_to_eeprom ? 1 : 0;

    if (joint_id == 0xFF) {
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_SET_STALL_PROTECT;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_SET_STALL_PROTECT;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 使能温度监控
 */
int motor_can_enable_temp_monitor(uint8_t joint_id)
{
    uint8_t data[8] = {0};

    if (joint_id == 0xFF) {
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            uint16_t can_id = ((g_config.base_addr + i) << 7) | CMD_ENABLE_TEMP_MON;
            if (CAN_Transmit(can_id, data, 8) != 0) {
                return -1;
            }
        }
        return 0;
    } else {
        uint16_t can_id = ((g_config.base_addr + joint_id) << 7) | CMD_ENABLE_TEMP_MON;
        return CAN_Transmit(can_id, data, 8);
    }
}

/**
 * @brief 获取电机状态
 */
void motor_can_get_state(uint8_t joint_id, motor_state_t *state)
{
    if (joint_id < MOTOR_CAN_NUM_JOINTS && state != NULL) {
        memcpy(state, (void*)&g_motor_states[joint_id], sizeof(motor_state_t));
    }
}

/**
 * @brief 检查所有电机是否到位
 */
bool motor_can_all_reached(void)
{
    for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        if (g_motor_states[i].finish_flag == 0) {
            return false;
        }
    }
    return true;
}

/**
 * @brief CAN接收回调
 */
void motor_can_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    /* 解析CAN ID */
    uint8_t motor_id = (can_id >> 7) & 0x0F;
    uint8_t cmd_id = can_id & 0x7F;

    /* 转换为关节索引 */
    if (motor_id < g_config.base_addr || motor_id >= g_config.base_addr + MOTOR_CAN_NUM_JOINTS) {
        return;  /* 不是我们的电机 */
    }
    uint8_t joint_id = motor_id - g_config.base_addr;

    /* 更新最后接收时间 */
    g_last_rx_time[joint_id] = get_tick_ms();

    /* 解析命令 */
    switch (cmd_id) {
        case CMD_GET_POSITION:  /* 0x23: 位置反馈 */
            if (len >= 5) {
                float position_rev;
                memcpy(&position_rev, data, sizeof(float));
                g_motor_states[joint_id].current_angle = position_rev * REV_TO_DEG;
                g_motor_states[joint_id].finish_flag = data[4];
            }
            break;

        case CMD_GET_VELOCITY:  /* 0x22: 速度反馈 */
            if (len >= 5) {
                float velocity_rps;
                memcpy(&velocity_rps, data, sizeof(float));
                g_motor_states[joint_id].velocity = velocity_rps * REV_TO_DEG;
                g_motor_states[joint_id].finish_flag = data[4];
            }
            break;

        case CMD_GET_CURRENT:  /* 0x21: 电流反馈 */
            if (len >= 5) {
                memcpy(&g_motor_states[joint_id].current, data, sizeof(float));
                g_motor_states[joint_id].finish_flag = data[4];
            }
            break;

        case CMD_GET_TEMPERATURE:  /* 0x25: 温度反馈 */
            if (len >= 4) {
                memcpy(&g_motor_states[joint_id].temperature, data, sizeof(float));
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 电机通信超时检测
 */
int motor_can_check_timeout(void)
{
    if (!g_timeout_enabled) {
        return -1;
    }

    uint32_t now = get_tick_ms();
    for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        if (g_last_rx_time[i] > 0 && (now - g_last_rx_time[i]) > MOTOR_TIMEOUT_MS) {
            return i;  /* 返回超时的电机ID */
        }
    }
    return -1;  /* 无超时 */
}

/**
 * @brief 获取电机最后通信时间
 */
uint32_t motor_can_get_last_rx_time(uint8_t joint_id)
{
    if (joint_id < MOTOR_CAN_NUM_JOINTS) {
        return g_last_rx_time[joint_id];
    }
    return 0;
}

/**
 * @brief 重置电机通信超时计时器
 */
void motor_can_reset_timeout(uint8_t joint_id)
{
    uint32_t now = get_tick_ms();
    if (joint_id == 0xFF) {
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            g_last_rx_time[i] = now;
        }
    } else if (joint_id < MOTOR_CAN_NUM_JOINTS) {
        g_last_rx_time[joint_id] = now;
    }
    g_timeout_enabled = true;
}

/**
 * @brief 获取CAN调试信息
 */
void motor_can_get_debug_info(uint32_t *tx_count, uint32_t *error_flags)
{
    if (tx_count != NULL) {
        *tx_count = g_tx_complete_count;
    }
    if (error_flags != NULL) {
        *error_flags = g_can_error_flags;
    }
}

/**
 * @brief CAN发送完成回调 (在CAN中断中调用)
 * @note 由FSP CAN驱动调用
 */
void canfd_tx_callback(can_callback_args_t *p_args)
{
    if (p_args->event == CAN_EVENT_TX_COMPLETE) {
        g_tx_complete_count++;
        if (g_tx_pending > 0) {
            g_tx_pending--;
        }

        /* 释放信号量，唤醒等待的发送任务 */
        if (g_tx_sem != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(g_tx_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else if (p_args->event == CAN_EVENT_ERR_WARNING ||
               p_args->event == CAN_EVENT_ERR_PASSIVE ||
               p_args->event == CAN_EVENT_ERR_BUS_OFF) {
        g_can_error_flags |= (1 << p_args->event);
    }
}

/**
 * @brief 清除堵转保护 (兼容接口)
 */
int motor_can_clear_stall(uint8_t joint_id)
{
    /* dummy机械臂的堵转保护由STM32F103电机控制器处理 */
    /* 这里只是兼容接口，实际不需要操作 */
    (void)joint_id;
    return 0;
}

/**
 * @brief 急停 (兼容接口)
 */
void motor_can_emergency_stop(void)
{
    /* 发送失能命令给所有电机 */
    motor_can_set_enable(0xFF, false);
}

/**
 * @brief CAN接收回调 (FSP回调函数)
 * @note 由FSP CAN驱动调用
 */
void canfd_callback(can_callback_args_t *p_args)
{
    if (p_args->event == CAN_EVENT_RX_COMPLETE) {
        /* 调用我们的接收处理函数 */
        motor_can_rx_callback(p_args->frame.id, p_args->frame.data, p_args->frame.data_length_code);
    } else if (p_args->event == CAN_EVENT_TX_COMPLETE) {
        /* 调用发送完成回调 */
        canfd_tx_callback(p_args);
    }
}

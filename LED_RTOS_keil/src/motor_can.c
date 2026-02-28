/**
 ******************************************************************************
 * @file    motor_can.c
 * @brief   ZDT闭环步进电机CAN驱动实现
 * @note    包含发送流控，防止TX Mailbox溢出
 ******************************************************************************
 */

#include "motor_can.h"
#include "gripper.h"
#include "hal_data.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <math.h>

/* ========== ZDT Emm协议定义 ========== */
#define CMD_POSITION_EMM        0xFD    /* Emm位置模式 */
#define CMD_SYNC_TRIGGER        0xFF    /* 同步触发 */
#define CMD_EMERGENCY_STOP      0xFE    /* 急停 */
#define CMD_ENABLE              0xF3    /* 使能控制 */
#define CMD_READ_STATUS         0x3A    /* 读取状态 */
#define CMD_READ_CONFIG         0x42    /* 读取驱动配置 (参数0x6C) */
#define CMD_WRITE_CONFIG        0x48    /* 写入驱动配置 (参数0xD1) */
#define CMD_READ_POSITION       0x36    /* 读取实时位置 */
#define CMD_READ_POS_ERROR      0x37    /* 读取位置误差 */

#define SYNC_FLAG_IMMEDIATE     0x00    /* 立即执行 */
#define SYNC_FLAG_WAIT          0x01    /* 等待同步触发 */

#define POS_MODE_ABSOLUTE       0x01    /* 绝对位置 */
#define POS_MODE_RELATIVE       0x00    /* 相对位置 */

#define BROADCAST_ADDR          0x00    /* 广播地址 */
#define SYNC_TRIGGER_BYTE       0x66    /* 同步触发标识 */

/* ========== 发送流控配置 ========== */
#define CAN_TX_MAILBOX_COUNT    4       /* TXMB 0-3 */
#define CAN_TX_RETRY_MAX        100     /* 最大重试次数 */
#define CAN_TX_RETRY_DELAY_US   50      /* 重试间隔 (微秒) */

/* ========== 内部状态 ========== */
static motor_can_config_t g_config = {
    .base_addr = 1,
    .max_speed_rpm = 300.0f,
    .use_high_precision = false,    /* Emm固件不使用高精度模式 */
    .use_extended_frame = true
};

static motor_state_t g_motor_states[MOTOR_CAN_NUM_JOINTS] = {0};
static bool g_initialized = false;
static uint8_t g_tx_mailbox_idx = 0;    /* 轮询使用的TX Mailbox索引 */

/* ========== 发送完成信号量 (P2优化) ========== */
static SemaphoreHandle_t g_tx_sem = NULL;
static StaticSemaphore_t g_tx_sem_buffer;  /* 静态信号量缓冲区 */
static volatile uint8_t g_tx_pending = 0;  /* 待发送完成的帧数 */
static volatile uint32_t g_tx_complete_count = 0;  /* TX完成计数 */
static volatile uint32_t g_can_error_flags = 0;    /* CAN错误标志 */

/* ========== 超时检测 ========== */
#define MOTOR_TIMEOUT_MS    200     /* 电机通信超时阈值 (ms) */
static volatile uint32_t g_last_rx_time[MOTOR_CAN_NUM_JOINTS] = {0};  /* 最后接收时间 */
static volatile bool g_timeout_enabled = false;    /* 超时检测使能 */

/* ========== 参数配置状态机 (Read-Modify-Write) ========== */
typedef enum {
    PARAM_STATE_IDLE = 0,
    PARAM_STATE_READING,
    PARAM_STATE_READY
} param_state_t;

static struct {
    param_state_t state;
    uint8_t target_joint;           /* 当前操作的关节 */
    uint8_t buffer[PARAM_BLOCK_SIZE]; /* 32字节参数缓冲区 */
    uint8_t rx_count;               /* 已接收字节数 */
    uint8_t rx_frame_idx;           /* 当前接收帧索引 */
} g_param_ctx = {0};

/* 内部函数声明 */
static int CAN_Transmit(uint32_t can_id, const uint8_t *data, uint8_t len);
static int CAN_TransmitLong(uint8_t addr, const uint8_t *payload, uint8_t payload_len);
static void delay_us(uint32_t us);
static bool check_motors_safe(void);

/* ========== 微秒延时 (简单实现) ========== */
static void delay_us(uint32_t us)
{
    /* 基于200MHz主频的粗略延时 */
    volatile uint32_t count = us * 40;  /* 调整系数 */
    while (count--) {
        __NOP();
    }
}

/* ========== 底层CAN发送接口 (带流控) ========== */

/**
 * @brief CAN发送函数 (优化版：信号量等待 + Mailbox轮询)
 * @param can_id  CAN ID (扩展帧29-bit)
 * @param data    数据指针
 * @param len     数据长度 (1-8)
 * @return 0=成功, -1=失败
 */
static int CAN_Transmit(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    can_frame_t frame = {
        .id               = can_id,
        .id_mode          = g_config.use_extended_frame ? CAN_ID_MODE_EXTENDED : CAN_ID_MODE_STANDARD,
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
            /* FSP_ERR_CAN_TRANSMIT_NOT_READY: 该Mailbox忙，尝试下一个 */
        }

        /* 所有Mailbox都忙，等待发送完成信号量 (最多1ms) */
        if (g_tx_sem != NULL) {
            xSemaphoreTake(g_tx_sem, pdMS_TO_TICKS(1));
        } else {
            /* 信号量未初始化，使用旧的延时方式 */
            delay_us(CAN_TX_RETRY_DELAY_US);
        }
    }

    /* 超时失败 */
    return -1;
}

/**
 * @brief CAN长帧分包发送 (Emm协议: >8字节自动分包)
 * @param addr        电机地址
 * @param payload     数据载荷 (不含地址, 首字节为功能码)
 * @param payload_len 载荷长度
 * @return 0=成功, -1=失败
 * @note  分包规则: CAN_ID = (addr<<8) + 包序号
 *        每包首字节都是功能码
 */
static int CAN_TransmitLong(uint8_t addr, const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len <= 8) {
        /* 短帧直接发送 */
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;
        return CAN_Transmit(can_id, payload, payload_len);
    }

    /* 长帧分包 */
    uint8_t func_code = payload[0];
    uint8_t packet_idx = 0;
    uint8_t offset = 0;

    while (offset < payload_len) {
        uint32_t can_id = g_config.use_extended_frame ?
                          (((uint32_t)addr << 8) | packet_idx) :
                          (addr | packet_idx);

        uint8_t frame_data[8];
        uint8_t frame_len;

        if (packet_idx == 0) {
            /* 第一包: 直接发送前8字节 */
            frame_len = (payload_len > 8) ? 8 : payload_len;
            memcpy(frame_data, payload, frame_len);
            offset = frame_len;
        } else {
            /* 后续包: 功能码 + 剩余数据 */
            frame_data[0] = func_code;
            uint8_t remain = payload_len - offset;
            uint8_t copy_len = (remain > 7) ? 7 : remain;
            memcpy(&frame_data[1], &payload[offset], copy_len);
            frame_len = copy_len + 1;
            offset += copy_len;
        }

        if (CAN_Transmit(can_id, frame_data, frame_len) != 0) {
            return -1;
        }

        packet_idx++;
        delay_us(500);  /* 包间延时防冲突 */
    }

    return 0;
}

/* ========== 公共接口实现 ========== */

void motor_can_init(const motor_can_config_t *config)
{
    if (config) {
        memcpy(&g_config, config, sizeof(motor_can_config_t));
    }

    memset(g_motor_states, 0, sizeof(g_motor_states));
    g_tx_mailbox_idx = 0;
    g_tx_pending = 0;

    /* 创建发送完成信号量 (P2优化) - 使用静态分配 */
    if (g_tx_sem == NULL) {
        g_tx_sem = xSemaphoreCreateBinaryStatic(&g_tx_sem_buffer);
    }

    /* 初始化CAN FD外设 */
    fsp_err_t err = R_CANFD_Open(&g_can0_ctrl, &g_can0_cfg);
    if (FSP_SUCCESS != err && FSP_ERR_ALREADY_OPEN != err) {
        /* 初始化失败 */
        debug_print("[CAN] Init FAILED! err=");
        debug_print_int((int)err);
        debug_println("");
        g_initialized = false;
        return;
    }

    debug_println("[CAN] Init OK");
    g_initialized = true;
}

/* ========== 运动前安全检查 ========== */
static bool check_motors_safe(void)
{
    for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        /* 检查故障标志 */
        if (g_motor_states[i].error) {
            return false;
        }
        /* 检查堵转标志 (status bit 4) */
        if (g_motor_states[i].status & 0x10) {
            return false;
        }
    }
    return true;
}

int motor_can_send_sync_position(const float joint_angles[MOTOR_CAN_NUM_JOINTS],
                                  float max_speed)
{
    if (!g_initialized) return -1;

    /* 运动前安全检查 */
    if (!check_motors_safe()) {
        return -2;  /* 电机故障或堵转 */
    }

    float speed = (max_speed > 0) ? max_speed : g_config.max_speed_rpm;
    uint16_t speed_raw = (uint16_t)(speed * MOTOR_CAN_SPEED_SCALE);

    /* 发送6个电机的位置指令 (Emm 0xFD格式) */
    for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        uint8_t addr = g_config.base_addr + i;
        float target = joint_angles[i];

        /* 角度转脉冲 */
        uint8_t dir = (target >= 0) ? 0x00 : 0x01;
        uint32_t clk = (uint32_t)(fabsf(target) * MOTOR_CAN_DEG_TO_PULSES);

        /* 构建Emm 0xFD命令载荷 (不含地址, 12字节) */
        uint8_t payload[12];
        payload[0]  = CMD_POSITION_EMM;          /* 0xFD */
        payload[1]  = dir;
        payload[2]  = (uint8_t)(speed_raw >> 8); /* 速度高字节 */
        payload[3]  = (uint8_t)(speed_raw & 0xFF);
        payload[4]  = MOTOR_CAN_DEFAULT_ACC;     /* 加速度 */
        payload[5]  = (uint8_t)(clk >> 24);      /* 脉冲数 */
        payload[6]  = (uint8_t)(clk >> 16);
        payload[7]  = (uint8_t)(clk >> 8);
        payload[8]  = (uint8_t)(clk & 0xFF);
        payload[9]  = POS_MODE_ABSOLUTE;         /* 绝对位置 */
        payload[10] = SYNC_FLAG_WAIT;            /* 等待同步触发 */
        payload[11] = MOTOR_CAN_CHECKSUM;        /* 0x6B */

        if (CAN_TransmitLong(addr, payload, 12) != 0) {
            return -1;
        }

        /* 更新目标状态 */
        g_motor_states[i].target_angle = target;
        g_motor_states[i].reached = false;
    }

    /* 发送同步触发广播 */
    motor_can_trigger_sync();

    return 0;
}

int motor_can_send_position(uint8_t joint_id, float angle, float speed, bool sync)
{
    if (!g_initialized || joint_id >= MOTOR_CAN_NUM_JOINTS) return -1;

    /* 运动前安全检查 (单轴) */
    if (g_motor_states[joint_id].error || (g_motor_states[joint_id].status & 0x10)) {
        return -2;  /* 电机故障或堵转 */
    }

    uint8_t addr = g_config.base_addr + joint_id;
    uint16_t speed_raw = (uint16_t)(speed * MOTOR_CAN_SPEED_SCALE);

    /* 角度转脉冲 */
    uint8_t dir = (angle >= 0) ? 0x00 : 0x01;
    uint32_t clk = (uint32_t)(fabsf(angle) * MOTOR_CAN_DEG_TO_PULSES);

    /* 构建Emm 0xFD命令载荷 (12字节) */
    uint8_t payload[12];
    payload[0]  = CMD_POSITION_EMM;
    payload[1]  = dir;
    payload[2]  = (uint8_t)(speed_raw >> 8);
    payload[3]  = (uint8_t)(speed_raw & 0xFF);
    payload[4]  = MOTOR_CAN_DEFAULT_ACC;
    payload[5]  = (uint8_t)(clk >> 24);
    payload[6]  = (uint8_t)(clk >> 16);
    payload[7]  = (uint8_t)(clk >> 8);
    payload[8]  = (uint8_t)(clk & 0xFF);
    payload[9]  = POS_MODE_ABSOLUTE;
    payload[10] = sync ? SYNC_FLAG_WAIT : SYNC_FLAG_IMMEDIATE;
    payload[11] = MOTOR_CAN_CHECKSUM;

    CAN_TransmitLong(addr, payload, 12);

    g_motor_states[joint_id].target_angle = angle;
    g_motor_states[joint_id].reached = false;

    return 0;
}

void motor_can_trigger_sync(void)
{
    uint32_t broadcast_id = g_config.use_extended_frame ?
                            ((uint32_t)BROADCAST_ADDR << 8) : BROADCAST_ADDR;

    uint8_t sync_data[3] = {
        CMD_SYNC_TRIGGER,
        SYNC_TRIGGER_BYTE,
        MOTOR_CAN_CHECKSUM
    };

    CAN_Transmit(broadcast_id, sync_data, 3);
}

void motor_can_emergency_stop(void)
{
    /* 广播急停 */
    uint32_t broadcast_id = g_config.use_extended_frame ?
                            ((uint32_t)BROADCAST_ADDR << 8) : BROADCAST_ADDR;

    uint8_t stop_data[2] = {CMD_EMERGENCY_STOP, MOTOR_CAN_CHECKSUM};
    CAN_Transmit(broadcast_id, stop_data, 2);

    /* 标记所有电机为未到位 */
    for (int i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        g_motor_states[i].reached = false;
    }
}

void motor_can_set_enable(bool enable)
{
    if (!g_initialized) {
        debug_println("[CAN] Not initialized!");
        return;
    }

    uint32_t broadcast_id = g_config.use_extended_frame ?
                            ((uint32_t)BROADCAST_ADDR << 8) : BROADCAST_ADDR;

    /* ZDT使能命令格式: 功能码(0xF3) + 参数(0xAB) + 状态 + 同步标志 + 校验 */
    uint8_t enable_data[5] = {
        CMD_ENABLE,             /* 0xF3 */
        0xAB,                   /* 参数字节 */
        enable ? 0x01 : 0x00,   /* 使能状态 */
        0x00,                   /* 同步标志 (0=不使用) */
        MOTOR_CAN_CHECKSUM      /* 0x6B */
    };

    debug_print("[CAN] TX ID=0x");
    /* 打印十六进制ID */
    char hex[9];
    for (int i = 7; i >= 0; i--) {
        hex[7-i] = "0123456789ABCDEF"[(broadcast_id >> (i*4)) & 0x0F];
    }
    hex[8] = '\0';
    debug_print(hex);
    debug_print(" Data=");
    for (int i = 0; i < 5; i++) {
        char h[3];
        h[0] = "0123456789ABCDEF"[(enable_data[i] >> 4) & 0x0F];
        h[1] = "0123456789ABCDEF"[enable_data[i] & 0x0F];
        h[2] = '\0';
        debug_print(h);
        debug_print(" ");
    }
    debug_println("");

    int ret = CAN_Transmit(broadcast_id, enable_data, 5);
    debug_print("[CAN] TX result=");
    debug_print_int(ret);
    debug_println(ret == 0 ? " (OK)" : " (FAIL)");
}

void motor_can_get_state(uint8_t joint_id, motor_state_t *state)
{
    if (joint_id < MOTOR_CAN_NUM_JOINTS && state) {
        memcpy(state, &g_motor_states[joint_id], sizeof(motor_state_t));
    }
}

bool motor_can_all_reached(void)
{
    for (int i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        if (!g_motor_states[i].reached) {
            return false;
        }
    }
    return true;
}

void motor_can_get_debug_info(uint32_t *tx_count, uint32_t *error_flags)
{
    if (tx_count) *tx_count = g_tx_complete_count;
    if (error_flags) *error_flags = g_can_error_flags;
}

void motor_can_clear_error_flags(void)
{
    g_can_error_flags = 0;
}

void motor_can_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (len < 2) return;

    /* 解析电机地址 */
    uint8_t addr = g_config.use_extended_frame ?
                   (uint8_t)(can_id >> 8) : (uint8_t)can_id;

    if (addr < g_config.base_addr ||
        addr >= g_config.base_addr + MOTOR_CAN_NUM_JOINTS) {
        return;
    }

    uint8_t joint_id = addr - g_config.base_addr;
    uint8_t cmd = data[0];

    /* 更新最后接收时间 (超时检测用) */
    g_last_rx_time[joint_id] = xTaskGetTickCount();

    /* 处理到位返回 (Emm: 0xFD + 0x9F = 到位) */
    if (cmd == CMD_POSITION_EMM && len >= 2) {
        if (data[1] == 0x9F) {  /* Emm到位标志 */
            g_motor_states[joint_id].reached = true;
        }
    }

    /* 处理状态返回 */
    if (cmd == CMD_READ_STATUS && len >= 4) {
        g_motor_states[joint_id].status = data[1];
        g_motor_states[joint_id].error = (data[1] & 0x80) != 0;
    }

    /* 处理位置返回 (0x36) - Emm格式 */
    if (cmd == CMD_READ_POSITION && len >= 6) {
        /* Emm返回: [0x36][dir][pos_3][pos_2][pos_1][pos_0][0x6B] */
        uint8_t dir = data[1];
        uint32_t pos_raw = ((uint32_t)data[2] << 24) |
                           ((uint32_t)data[3] << 16) |
                           ((uint32_t)data[4] << 8)  |
                           ((uint32_t)data[5]);
        /* Emm位置单位: 编码器值, 转换为角度: pos * 360 / 65536 */
        float angle = (float)pos_raw * 360.0f / 65536.0f;
        if (dir) angle = -angle;
        g_motor_states[joint_id].current_angle = angle;
    }

    /* 处理位置误差返回 (0x37) - 大端序 */
    if (cmd == CMD_READ_POS_ERROR && len >= 3) {
        /* data[1..2] = 位置误差 (大端序) */
        /* 可扩展: 存储到 motor_state_t 中 */
    }

    /* 处理配置读取返回 (0x42) - 分包接收 */
    if (cmd == CMD_READ_CONFIG && g_param_ctx.state == PARAM_STATE_READING &&
        joint_id == g_param_ctx.target_joint) {
        /*
         * 0x42 返回格式 (说明书第62页):
         * 第1帧: 0x42 + Length(37) + Count(24) + D0 + D1 + D2 + D3 + D4
         * 第2帧: D5 + D6 + D7 + ... (后续数据)
         * 总共返回 37 字节，其中前3字节是头部，后32字节是参数
         */
        if (g_param_ctx.rx_frame_idx == 0 && len >= 4) {
            /* 第一帧: 跳过 cmd(1) + length(1) + count(1)，从 data[3] 开始是参数 */
            uint8_t copy_len = (len > 3) ? (len - 3) : 0;
            if (copy_len > 0 && g_param_ctx.rx_count + copy_len <= PARAM_BLOCK_SIZE) {
                memcpy(&g_param_ctx.buffer[g_param_ctx.rx_count], &data[3], copy_len);
                g_param_ctx.rx_count += copy_len;
            }
        } else {
            /* 后续帧: 直接复制数据 */
            uint8_t copy_len = len;
            if (g_param_ctx.rx_count + copy_len <= PARAM_BLOCK_SIZE) {
                memcpy(&g_param_ctx.buffer[g_param_ctx.rx_count], data, copy_len);
                g_param_ctx.rx_count += copy_len;
            }
        }
        g_param_ctx.rx_frame_idx++;

        /* 检查是否接收完成 */
        if (g_param_ctx.rx_count >= PARAM_BLOCK_SIZE) {
            g_param_ctx.state = PARAM_STATE_READY;
        }
    }
}

/* ========== 电机参数配置 ========== */

/* ZDT_X_V2 命令码 */
#define CMD_SET_ACCELERATION    0xF6    /* 设置加速度 */
#define CMD_SET_POS_KP          0x8A    /* 位置环 Kp */
#define CMD_SET_VEL_KP          0x8B    /* 速度环 Kp */
#define CMD_SET_VEL_KI          0x8C    /* 速度环 Ki */
#define CMD_READ_STALL          0x3E    /* 读取堵转状态 */
#define CMD_CLEAR_STALL         0x3F    /* 清除堵转保护 */
#define CMD_READ_POS_ERROR      0x37    /* 读取位置误差 */
#define CMD_SET_CURRENT_LIMIT   0xF5    /* 设置电流限制 */

int motor_can_set_acceleration(uint8_t joint_id, uint16_t acc)
{
    if (joint_id != 0xFF && joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t start = (joint_id == 0xFF) ? 0 : joint_id;
    uint8_t end = (joint_id == 0xFF) ? MOTOR_CAN_NUM_JOINTS : (joint_id + 1);

    for (uint8_t i = start; i < end; i++) {
        uint8_t addr = g_config.base_addr + i;
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;

        uint8_t data[8] = {
            CMD_SET_ACCELERATION,
            0x00,                       /* 保留 */
            (uint8_t)(acc >> 8),        /* 加速度高字节 */
            (uint8_t)(acc & 0xFF),      /* 加速度低字节 */
            0x00, 0x00, 0x00,
            MOTOR_CAN_CHECKSUM
        };

        if (CAN_Transmit(can_id, data, 8) != 0) {
            return -1;
        }
    }
    return 0;
}

int motor_can_set_pid(uint8_t joint_id, uint16_t kp, uint16_t kv, uint16_t ki)
{
    if (joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t addr = g_config.base_addr + joint_id;
    uint32_t can_id = g_config.use_extended_frame ?
                      ((uint32_t)addr << 8) : addr;

    /* 设置位置环 Kp */
    uint8_t data_kp[8] = {
        CMD_SET_POS_KP, 0x00,
        (uint8_t)(kp >> 8), (uint8_t)(kp & 0xFF),
        0x00, 0x00, 0x00, MOTOR_CAN_CHECKSUM
    };
    if (CAN_Transmit(can_id, data_kp, 8) != 0) return -1;

    /* 设置速度环 Kp */
    uint8_t data_kv[8] = {
        CMD_SET_VEL_KP, 0x00,
        (uint8_t)(kv >> 8), (uint8_t)(kv & 0xFF),
        0x00, 0x00, 0x00, MOTOR_CAN_CHECKSUM
    };
    if (CAN_Transmit(can_id, data_kv, 8) != 0) return -1;

    /* 设置速度环 Ki */
    uint8_t data_ki[8] = {
        CMD_SET_VEL_KI, 0x00,
        (uint8_t)(ki >> 8), (uint8_t)(ki & 0xFF),
        0x00, 0x00, 0x00, MOTOR_CAN_CHECKSUM
    };
    if (CAN_Transmit(can_id, data_ki, 8) != 0) return -1;

    return 0;
}

int motor_can_get_stall_status(uint8_t joint_id)
{
    if (joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    /* 检查状态字节的堵转标志位 */
    return (g_motor_states[joint_id].status & 0x10) ? 1 : 0;
}

int motor_can_read_status(uint8_t joint_id)
{
    if (joint_id != 0xFF && joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t start = (joint_id == 0xFF) ? 0 : joint_id;
    uint8_t end = (joint_id == 0xFF) ? MOTOR_CAN_NUM_JOINTS : (joint_id + 1);

    for (uint8_t i = start; i < end; i++) {
        uint8_t addr = g_config.base_addr + i;
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;

        /* ZDT_X_V2 读取状态命令: 0x3A + 0x6B */
        uint8_t data[2] = {
            CMD_READ_STATUS,
            MOTOR_CAN_CHECKSUM
        };

        if (CAN_Transmit(can_id, data, 2) != 0) {
            return -1;
        }
    }
    return 0;
}

int motor_can_ping(uint8_t joint_id)
{
    if (joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    /* 清空该电机的状态标志，用于检测是否收到新响应 */
    uint8_t old_status = g_motor_states[joint_id].status;
    g_motor_states[joint_id].status = 0xFF;  /* 标记为未更新 */

    /* 发送读取状态命令 */
    if (motor_can_read_status(joint_id) != 0) {
        g_motor_states[joint_id].status = old_status;  /* 恢复 */
        return -1;
    }

    /* 等待响应 (最多500ms) */
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (g_motor_states[joint_id].status != 0xFF) {
            /* 收到响应 */
            return 0;
        }
    }

    /* 超时 */
    g_motor_states[joint_id].status = old_status;  /* 恢复 */
    return -1;
}

int motor_can_clear_stall(uint8_t joint_id)
{
    if (joint_id != 0xFF && joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t start = (joint_id == 0xFF) ? 0 : joint_id;
    uint8_t end = (joint_id == 0xFF) ? MOTOR_CAN_NUM_JOINTS : (joint_id + 1);

    for (uint8_t i = start; i < end; i++) {
        uint8_t addr = g_config.base_addr + i;
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;

        uint8_t data[3] = {
            CMD_CLEAR_STALL,
            0x01,                       /* 清除标志 */
            MOTOR_CAN_CHECKSUM
        };

        if (CAN_Transmit(can_id, data, 3) != 0) {
            return -1;
        }
    }
    return 0;
}

int motor_can_read_position_error(uint8_t joint_id)
{
    if (joint_id != 0xFF && joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t start = (joint_id == 0xFF) ? 0 : joint_id;
    uint8_t end = (joint_id == 0xFF) ? MOTOR_CAN_NUM_JOINTS : (joint_id + 1);

    for (uint8_t i = start; i < end; i++) {
        uint8_t addr = g_config.base_addr + i;
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;

        /* ZDT_X_V2 读取位置误差: 0x37 + 0x6B */
        uint8_t data[2] = {
            CMD_READ_POS_ERROR,
            MOTOR_CAN_CHECKSUM
        };

        if (CAN_Transmit(can_id, data, 2) != 0) {
            return -1;
        }
    }
    return 0;
}

int motor_can_set_current_limit(uint8_t joint_id, uint16_t ma_limit)
{
    if (joint_id != 0xFF && joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t start = (joint_id == 0xFF) ? 0 : joint_id;
    uint8_t end = (joint_id == 0xFF) ? MOTOR_CAN_NUM_JOINTS : (joint_id + 1);

    for (uint8_t i = start; i < end; i++) {
        uint8_t addr = g_config.base_addr + i;
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;

        /* ZDT_X_V2 设置电流限制: 0xF5 + 高字节 + 低字节 + 0x6B */
        uint8_t data[4] = {
            CMD_SET_CURRENT_LIMIT,
            (uint8_t)(ma_limit >> 8),
            (uint8_t)(ma_limit & 0xFF),
            MOTOR_CAN_CHECKSUM
        };

        if (CAN_Transmit(can_id, data, 4) != 0) {
            return -1;
        }
    }
    return 0;
}

/* ========== 位置清零命令 ========== */
#define CMD_RESET_POS_TO_ZERO   0x0A    /* 将当前位置清零 */
#define CMD_RESET_POS_PARAM     0x6D    /* 清零命令参数 */
#define CMD_GO_HOME             0x9A    /* 回零命令 */
#define CMD_SET_HOME_SPEED      0x9B    /* 设置回零速度 */

int motor_can_reset_position_to_zero(uint8_t joint_id)
{
    if (joint_id != 0xFF && joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t start = (joint_id == 0xFF) ? 0 : joint_id;
    uint8_t end = (joint_id == 0xFF) ? MOTOR_CAN_NUM_JOINTS : (joint_id + 1);

    for (uint8_t i = start; i < end; i++) {
        uint8_t addr = g_config.base_addr + i;
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;

        /* ZDT_X_V2 将当前位置清零命令:
         * CAN ID = (addr << 8) | packNum
         * Data[0] = 0x0A (功能码)
         * Data[1] = 0x6D (参数)
         * Data[2] = 0x6B (校验字节)
         */
        uint8_t data[3] = {
            CMD_RESET_POS_TO_ZERO,      /* 功能码 0x0A */
            CMD_RESET_POS_PARAM,        /* 参数 0x6D */
            MOTOR_CAN_CHECKSUM          /* 校验 0x6B */
        };

        if (CAN_Transmit(can_id, data, 3) != 0) {
            return -1;
        }

        /* 清零后重置软件侧状态 */
        g_motor_states[i].current_angle = 0.0f;
        g_motor_states[i].target_angle = 0.0f;
        g_motor_states[i].reached = true;
    }
    return 0;
}

int motor_can_go_home(uint8_t joint_id, uint16_t speed, uint8_t direction)
{
    if (joint_id != 0xFF && joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t start = (joint_id == 0xFF) ? 0 : joint_id;
    uint8_t end = (joint_id == 0xFF) ? MOTOR_CAN_NUM_JOINTS : (joint_id + 1);

    for (uint8_t i = start; i < end; i++) {
        uint8_t addr = g_config.base_addr + i;
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;

        /* ZDT_X_V2 回零命令:
         * Data[0] = 0x9A (功能码)
         * Data[1] = 方向 (0=正向, 1=反向)
         * Data[2] = 速度高字节
         * Data[3] = 速度低字节
         * Data[4] = 0x6B (校验)
         */
        uint8_t data[5] = {
            CMD_GO_HOME,
            direction & 0x01,
            (uint8_t)(speed >> 8),
            (uint8_t)(speed & 0xFF),
            MOTOR_CAN_CHECKSUM
        };

        if (CAN_Transmit(can_id, data, 5) != 0) {
            return -1;
        }

        g_motor_states[i].reached = false;
    }
    return 0;
}

void motor_can_config_fast_mode(void)
{
    /* 高速模式参数 (激进) */
    motor_can_set_acceleration(0xFF, 3000);  /* 所有电机加速度 3000 */

    /* 各关节 PID (根据负载调整) */
    /* 基座关节 (惯性大，需要更高 Kp) */
    motor_can_set_pid(0, 2500, 1500, 500);
    /* 大臂关节 */
    motor_can_set_pid(1, 2000, 1500, 500);
    /* 小臂关节 */
    motor_can_set_pid(2, 2000, 1500, 500);
    /* 腕部关节 (惯性小，可以更激进) */
    motor_can_set_pid(3, 1500, 1200, 400);
    motor_can_set_pid(4, 1500, 1200, 400);
    motor_can_set_pid(5, 1500, 1200, 400);
}

void motor_can_config_smooth_mode(void)
{
    /* 平稳模式参数 (保守) */
    motor_can_set_acceleration(0xFF, 1500);  /* 所有电机加速度 1500 */

    /* 各关节 PID (较低增益) */
    motor_can_set_pid(0, 1200, 1000, 300);
    motor_can_set_pid(1, 1000, 1000, 300);
    motor_can_set_pid(2, 1000, 1000, 300);
    motor_can_set_pid(3, 800, 800, 200);
    motor_can_set_pid(4, 800, 800, 200);
    motor_can_set_pid(5, 800, 800, 200);
}

/* ========== 状态读取函数 (替代复杂的0x43分包) ========== */

int motor_can_read_position(uint8_t joint_id)
{
    if (joint_id != 0xFF && joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    uint8_t start = (joint_id == 0xFF) ? 0 : joint_id;
    uint8_t end = (joint_id == 0xFF) ? MOTOR_CAN_NUM_JOINTS : (joint_id + 1);

    for (uint8_t i = start; i < end; i++) {
        uint8_t addr = g_config.base_addr + i;
        uint32_t can_id = g_config.use_extended_frame ?
                          ((uint32_t)addr << 8) : addr;

        /* ZDT_X_V2 读取实时位置: 0x36 + 0x6B */
        uint8_t data[2] = {
            CMD_READ_POSITION,
            MOTOR_CAN_CHECKSUM
        };

        if (CAN_Transmit(can_id, data, 2) != 0) {
            return -1;
        }
    }
    return 0;
}

/* ========== 驱动参数配置 (Read-Modify-Write via 0x42/0x48) ========== */

/**
 * @brief 发送读取配置命令 (0x42)
 */
static int param_send_read_cmd(uint8_t joint_id)
{
    uint8_t addr = g_config.base_addr + joint_id;
    uint32_t can_id = g_config.use_extended_frame ?
                      ((uint32_t)addr << 8) : addr;

    /* 0x42 + 0x6C + 0x6B */
    uint8_t data[3] = { CMD_READ_CONFIG, 0x6C, MOTOR_CAN_CHECKSUM };

    g_param_ctx.state = PARAM_STATE_READING;
    g_param_ctx.target_joint = joint_id;
    g_param_ctx.rx_count = 0;
    g_param_ctx.rx_frame_idx = 0;
    memset(g_param_ctx.buffer, 0, PARAM_BLOCK_SIZE);

    return CAN_Transmit(can_id, data, 3);
}

/**
 * @brief 发送写入配置命令 (0x48) - 分包发送37字节
 * @param joint_id 关节ID
 * @param param_data 32字节参数数据
 * @param save_to_eeprom 是否保存到EEPROM (true=永久保存, false=掉电丢失)
 * @note 格式: Addr + 0x48 + 0xD1 + Store + [32字节参数] + Checksum
 *       总长度 = 1(cmd) + 1(param) + 1(store) + 32(data) + 1(checksum) = 36字节
 *       需要拆分为 5 个 CAN 帧发送
 */
static int param_send_write_cmd(uint8_t joint_id, const uint8_t *param_data, bool save_to_eeprom)
{
    uint8_t addr = g_config.base_addr + joint_id;
    uint32_t can_id_base = g_config.use_extended_frame ?
                           ((uint32_t)addr << 8) : addr;

    /* 构建完整的发送缓冲区 (36字节) */
    uint8_t tx_buf[36];
    tx_buf[0] = CMD_WRITE_CONFIG;   /* 0x48 */
    tx_buf[1] = 0xD1;               /* 参数 */
    tx_buf[2] = save_to_eeprom ? 0x01 : 0x00;  /* Store: 1=保存, 0=不保存 */
    memcpy(&tx_buf[3], param_data, PARAM_BLOCK_SIZE);  /* 32字节参数 */
    tx_buf[35] = MOTOR_CAN_CHECKSUM; /* 校验 */

    /* 分包发送: 每帧最多8字节，使用 CAN ID 的低位区分包号 */
    uint8_t frame_data[8];
    uint8_t offset = 0;
    uint8_t frame_idx = 0;

    while (offset < 36) {
        uint8_t len = (36 - offset > 8) ? 8 : (36 - offset);
        memcpy(frame_data, &tx_buf[offset], len);

        /* CAN ID = base | frame_idx (用于多帧识别) */
        uint32_t can_id = can_id_base | frame_idx;

        if (CAN_Transmit(can_id, frame_data, len) != 0) {
            return -1;
        }

        offset += len;
        frame_idx++;

        /* 帧间延时 1ms，防止驱动器接收缓冲区溢出 */
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return 0;
}

/**
 * @brief 等待参数读取完成
 * @param timeout_ms 超时时间(毫秒)
 * @return 0=成功, -1=超时
 */
static int param_wait_ready(uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount();
    while (g_param_ctx.state != PARAM_STATE_READY) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            g_param_ctx.state = PARAM_STATE_IDLE;
            return -1;  /* 超时 */
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return 0;
}

/**
 * @brief 通用参数更新函数 (Read-Modify-Write)
 * @param save_to_eeprom 是否保存到EEPROM
 */
static int motor_can_update_param(uint8_t joint_id, uint8_t offset, uint8_t value, bool save_to_eeprom)
{
    if (joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return -1;
    }

    /* Step 1: READ - 发送读取命令 */
    if (param_send_read_cmd(joint_id) != 0) {
        return -1;
    }

    /* Step 2: 等待接收完成 (最多500ms) */
    if (param_wait_ready(500) != 0) {
        return -2;  /* 读取超时 */
    }

    /* Step 3: MODIFY - 修改指定参数 */
    g_param_ctx.buffer[offset] = value;

    /* 强制开启高精度模式 (Minimum Jerk 必需) */
    g_param_ctx.buffer[OFFSET_S_POSTDP] = 0x01;

    /* Step 4: WRITE - 写回配置 */
    if (param_send_write_cmd(joint_id, g_param_ctx.buffer, save_to_eeprom) != 0) {
        return -1;
    }

    g_param_ctx.state = PARAM_STATE_IDLE;
    return 0;
}

int motor_can_set_mplyer(uint8_t joint_id, uint8_t enable)
{
    if (joint_id == 0xFF) {
        /* 所有电机 */
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            int ret = motor_can_update_param(i, OFFSET_MPLYER, enable ? 1 : 0, false);
            if (ret != 0) return ret;
        }
        return 0;
    }
    return motor_can_update_param(joint_id, OFFSET_MPLYER, enable ? 1 : 0, false);
}

int motor_can_set_lpfilter(uint8_t joint_id, uint8_t level)
{
    if (level > 7) level = 7;

    if (joint_id == 0xFF) {
        /* 所有电机 */
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            int ret = motor_can_update_param(i, OFFSET_LPFILTER, level, false);
            if (ret != 0) return ret;
        }
        return 0;
    }
    return motor_can_update_param(joint_id, OFFSET_LPFILTER, level, false);
}

int motor_can_config_minimum_jerk_mode(void)
{
    return motor_can_config_minimum_jerk_mode_ex(false);
}

int motor_can_config_minimum_jerk_mode_ex(bool save_to_eeprom)
{
    int ret;

    /* 设置所有电机: MPlyer=1, LPFilter=0, S_PosTDP=1 */
    for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        /* MPlyer = 1 (开启插补) */
        ret = motor_can_update_param(i, OFFSET_MPLYER, 1, save_to_eeprom);
        if (ret != 0) return ret;

        /* LPFilter = 0 (最低滤波) */
        ret = motor_can_update_param(i, OFFSET_LPFILTER, 0, save_to_eeprom);
        if (ret != 0) return ret;
    }

    return 0;
}

/* ========== FSP CAN FD 回调函数 ========== */

/**
 * @brief CAN FD 中断回调 (由FSP调用)
 * @param p_args 回调参数
 */
void canfd_callback(can_callback_args_t * p_args)
{
    switch (p_args->event) {
        case CAN_EVENT_RX_COMPLETE:
            /* 根据CAN ID分发到不同处理函数 */
            if (p_args->frame.id == GRIPPER_CAN_ID) {
                /* 夹爪帧 */
                gripper_rx_callback(
                    p_args->frame.id,
                    p_args->frame.data,
                    p_args->frame.data_length_code
                );
            } else {
                /* 电机帧 */
                motor_can_rx_callback(
                    p_args->frame.id,
                    p_args->frame.data,
                    p_args->frame.data_length_code
                );
            }
            break;

        case CAN_EVENT_TX_COMPLETE:
            /* 发送完成，释放信号量通知等待的任务 */
            g_tx_complete_count++;  /* 记录TX完成次数 */
            if (g_tx_pending > 0) {
                g_tx_pending--;
            }
            if (g_tx_sem != NULL) {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGiveFromISR(g_tx_sem, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            break;

        case CAN_EVENT_ERR_WARNING:
            /* TEC或REC达到警告阈值 */
            g_can_error_flags |= 0x01;
            break;

        case CAN_EVENT_ERR_PASSIVE:
            /* 进入错误被动状态 - 严重！ */
            g_can_error_flags |= 0x02;
            break;

        case CAN_EVENT_ERR_BUS_OFF:
            /* 总线关闭 - 非常严重！需要检查硬件连接 */
            g_can_error_flags |= 0x04;
            break;

        case CAN_EVENT_BUS_RECOVERY:
            /* 总线恢复 */
            g_can_error_flags &= ~0x04;  /* 清除BUS_OFF标志 */
            break;

        case CAN_EVENT_ERR_BUS_LOCK:
            g_can_error_flags |= 0x08;
            break;

        case CAN_EVENT_ERR_CHANNEL:
            g_can_error_flags |= 0x10;
            break;

        case CAN_EVENT_ERR_GLOBAL:
            /* 其他CAN错误 */
            g_can_error_flags |= 0x20;
            break;

        case CAN_EVENT_TX_ABORTED:
        case CAN_EVENT_TX_FIFO_EMPTY:
        default:
            break;
    }
}

/* ========== 超时检测实现 ========== */

int motor_can_check_timeout(void)
{
    if (!g_timeout_enabled || !g_initialized) {
        return -1;
    }

    uint32_t now = xTaskGetTickCount();

    for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        uint32_t elapsed = now - g_last_rx_time[i];
        if (elapsed > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS)) {
            /* 电机i超时 */
            return (int)i;
        }
    }

    return -1;  /* 无超时 */
}

uint32_t motor_can_get_last_rx_time(uint8_t joint_id)
{
    if (joint_id >= MOTOR_CAN_NUM_JOINTS) {
        return 0;
    }
    return g_last_rx_time[joint_id];
}

void motor_can_reset_timeout(uint8_t joint_id)
{
    uint32_t now = xTaskGetTickCount();

    if (joint_id == 0xFF) {
        /* 重置所有电机 */
        for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
            g_last_rx_time[i] = now;
        }
        g_timeout_enabled = true;
    } else if (joint_id < MOTOR_CAN_NUM_JOINTS) {
        g_last_rx_time[joint_id] = now;
        g_timeout_enabled = true;
    }
}

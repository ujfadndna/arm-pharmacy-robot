/**
 ******************************************************************************
 * @file    motor_sca.c
 * @brief   INNFOS SCA智能执行器CAN驱动实现
 * @note    协议参考: dummy-auk/firmware/.../mintasca/sca_protocol.c
 *          HAL适配: STM32 HAL_CAN → RA6M5 FSP R_CANFD_Write
 *
 *          CAN帧格式:
 *            ID   = 执行器编号 (1~6)，标准11-bit帧
 *            Data = [cmd, data...]
 *
 *          位置单位换算:
 *            pos_rev = angle_deg / 360.0f * reduction_ratio
 *            IQ24    = (int32_t)(pos_rev * 16777216.0f)
 ******************************************************************************
 */

#include "motor_sca.h"
#include "hal_data.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <math.h>

/* ========== SCA协议指令 (来自sca_protocol.h) ========== */
#define SCA_W3_POSITION     0x0A    /* 写位置 (5字节: cmd + IQ24) */
#define SCA_W1_POWER        0x2A    /* 写电源状态 (2字节: cmd + state) */
#define SCA_W1_MODE         0x07    /* 写工作模式 (2字节: cmd + mode) */
#define SCA_R1_HEARTBEAT    0x00    /* 读心跳 (1字节: cmd) */
#define SCA_R3_POSITION     0x06    /* 读位置 (1字节: cmd，响应5字节) */

#define SCA_MODE_POSITION   0x03    /* 位置模式 */
#define SCA_POWER_ENABLE    0x01    /* 上电 */
#define SCA_POWER_DISABLE   0x00    /* 下电 */

#define IQ24_SCALE          16777216.0f   /* 2^24 */

/* ========== 发送流控 ========== */
#define CAN_TX_MAILBOX_COUNT    4
#define CAN_TX_RETRY_MAX        200
#define CAN_TX_RETRY_DELAY_US   50

/* ========== 内部状态 ========== */
static motor_state_t g_sca_states[MOTOR_SCA_NUM_JOINTS];
static float g_target_rev[MOTOR_SCA_NUM_JOINTS];   /* 目标位置（圈数） */
static bool  g_initialized = false;
static uint8_t g_tx_mb_idx = 0;

/* TX信号量（与motor_can.c共享CAN外设，TX_COMPLETE由canfd_callback触发） */
static SemaphoreHandle_t g_sca_tx_sem = NULL;
static StaticSemaphore_t g_sca_tx_sem_buf;

/* ========== 微秒延时 ========== */
static void sca_delay_us(uint32_t us)
{
    volatile uint32_t n = us * 40;  /* 200MHz粗略延时 */
    while (n--) { __NOP(); }
}

/* ========== 底层CAN发送（标准11-bit帧） ========== */
static int sca_can_send(uint8_t sca_id, const uint8_t *data, uint8_t len)
{
    can_frame_t frame = {
        .id               = sca_id,
        .id_mode          = CAN_ID_MODE_STANDARD,
        .type             = CAN_FRAME_TYPE_DATA,
        .data_length_code = len,
        .options          = 0
    };
    memcpy(frame.data, data, len);

    for (uint32_t retry = 0; retry < CAN_TX_RETRY_MAX; retry++) {
        for (uint8_t mb = 0; mb < CAN_TX_MAILBOX_COUNT; mb++) {
            uint8_t mailbox = (g_tx_mb_idx + mb) % CAN_TX_MAILBOX_COUNT;
            fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, mailbox, &frame);
            if (FSP_SUCCESS == err) {
                g_tx_mb_idx = (mailbox + 1) % CAN_TX_MAILBOX_COUNT;
                return 0;
            }
        }
        if (g_sca_tx_sem != NULL) {
            xSemaphoreTake(g_sca_tx_sem, pdMS_TO_TICKS(1));
        } else {
            sca_delay_us(CAN_TX_RETRY_DELAY_US);
        }
    }
    return -1;
}

/* 发送1字节命令（心跳/读请求） */
static int sca_send_cmd1(uint8_t id, uint8_t cmd)
{
    uint8_t buf[1] = { cmd };
    return sca_can_send(id, buf, 1);
}

/* 发送2字节命令（W1类型：cmd + uint8） */
static int sca_send_cmd2(uint8_t id, uint8_t cmd, uint8_t val)
{
    uint8_t buf[2] = { cmd, val };
    return sca_can_send(id, buf, 2);
}

/* 发送位置命令（W3_Position：cmd + IQ24，5字节） */
static int sca_send_position(uint8_t id, float pos_rev)
{
    int32_t iq24 = (int32_t)(pos_rev * IQ24_SCALE);
    uint8_t buf[5] = {
        SCA_W3_POSITION,
        (uint8_t)(iq24 >> 24),
        (uint8_t)(iq24 >> 16),
        (uint8_t)(iq24 >> 8),
        (uint8_t)(iq24 >> 0)
    };
    return sca_can_send(id, buf, 5);
}

/* ========== 公共接口实现 ========== */

void motor_sca_init(void)
{
    memset(g_sca_states, 0, sizeof(g_sca_states));
    memset(g_target_rev, 0, sizeof(g_target_rev));
    g_tx_mb_idx = 0;

    if (g_sca_tx_sem == NULL) {
        g_sca_tx_sem = xSemaphoreCreateBinaryStatic(&g_sca_tx_sem_buf);
    }

    /* 打开CAN外设（1Mbps标准CAN，已在hal_data.c配置） */
    fsp_err_t err = R_CANFD_Open(&g_can0_ctrl, &g_can0_cfg);
    if (FSP_SUCCESS != err && FSP_ERR_ALREADY_OPEN != err) {
        debug_print("[SCA] CAN init FAILED! err=");
        debug_print_int((int)err);
        debug_println("");
        g_initialized = false;
        return;
    }

    debug_println("[SCA] Init OK (1Mbps standard CAN)");
    g_initialized = true;
}

int motor_sca_enable_all(void)
{
    if (!g_initialized) return -1;
    int ret = 0;
    for (uint8_t i = 0; i < MOTOR_SCA_NUM_JOINTS; i++) {
        uint8_t sca_id = MOTOR_SCA_BASE_ID + i;
        /* 1. 切换到位置模式 */
        if (sca_send_cmd2(sca_id, SCA_W1_MODE, SCA_MODE_POSITION) != 0) ret = -1;
        sca_delay_us(500);
        /* 2. 上电使能 */
        if (sca_send_cmd2(sca_id, SCA_W1_POWER, SCA_POWER_ENABLE) != 0) ret = -1;
        sca_delay_us(500);
    }
    debug_println("[SCA] Enable all");
    return ret;
}

int motor_sca_disable_all(void)
{
    if (!g_initialized) return -1;
    int ret = 0;
    for (uint8_t i = 0; i < MOTOR_SCA_NUM_JOINTS; i++) {
        uint8_t sca_id = MOTOR_SCA_BASE_ID + i;
        if (sca_send_cmd2(sca_id, SCA_W1_POWER, SCA_POWER_DISABLE) != 0) ret = -1;
        sca_delay_us(200);
    }
    debug_println("[SCA] Disable all");
    return ret;
}

int motor_sca_send_sync_position(const float joint_angles[MOTOR_SCA_NUM_JOINTS],
                                  float max_speed)
{
    (void)max_speed;  /* TODO: 通过W3_PPMaxVelocity设置速度限制 */
    if (!g_initialized) return -1;

    int ret = 0;
    for (uint8_t i = 0; i < MOTOR_SCA_NUM_JOINTS; i++) {
        uint8_t sca_id = MOTOR_SCA_BASE_ID + i;
        float pos_rev = joint_angles[i] / 360.0f * MOTOR_SCA_REDUCTION_RATIO;

        if (sca_send_position(sca_id, pos_rev) != 0) {
            ret = -1;
        } else {
            g_target_rev[i] = pos_rev;
            g_sca_states[i].target_angle = joint_angles[i];
            g_sca_states[i].reached = false;
        }
    }
    return ret;
}

void motor_sca_emergency_stop(void)
{
    /* 立即失能所有执行器 */
    for (uint8_t i = 0; i < MOTOR_SCA_NUM_JOINTS; i++) {
        uint8_t sca_id = MOTOR_SCA_BASE_ID + i;
        sca_send_cmd2(sca_id, SCA_W1_POWER, SCA_POWER_DISABLE);
    }
    debug_println("[SCA] EMERGENCY STOP");
}

bool motor_sca_all_reached(void)
{
    for (int i = 0; i < MOTOR_SCA_NUM_JOINTS; i++) {
        if (!g_sca_states[i].reached) return false;
    }
    return true;
}

void motor_sca_get_state(uint8_t id, motor_state_t *state)
{
    if (id >= MOTOR_SCA_NUM_JOINTS || state == NULL) return;
    *state = g_sca_states[id];
}

int motor_sca_ping(uint8_t id)
{
    if (id >= MOTOR_SCA_NUM_JOINTS) return -1;
    uint8_t sca_id = MOTOR_SCA_BASE_ID + id;

    /* 发送心跳请求 */
    if (sca_send_cmd1(sca_id, SCA_R1_HEARTBEAT) != 0) return -1;

    /* 等待响应（简单轮询，最多10ms） */
    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(10)) {
        if (g_sca_states[id].status != 0) return 0;
        vTaskDelay(1);
    }
    return -1;  /* 超时 */
}

/**
 * @brief CAN接收回调
 *
 * SCA响应帧格式（标准11-bit ID = 执行器编号）:
 *   R3_Position (0x06): [0x06, pos>>24, pos>>16, pos>>8, pos>>0]
 *   R4_CVP      (0x94): [0x94, cur_hi, cur_lo, vel×4, pos×4]
 *   W3_Position (0x0A): echo回显，data同发送内容
 *   R1_Heartbeat(0x00): [0x00, mode, power_state]
 */
void motor_sca_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    /* 检查ID范围 (1~6) */
    if (can_id < MOTOR_SCA_BASE_ID ||
        can_id >= (uint32_t)(MOTOR_SCA_BASE_ID + MOTOR_SCA_NUM_JOINTS)) {
        return;
    }
    if (len < 1) return;

    uint8_t joint_idx = (uint8_t)(can_id - MOTOR_SCA_BASE_ID);
    uint8_t cmd = data[0];

    /* 解析位置反馈 (R3_Position 或 W3_Position echo) */
    if ((cmd == SCA_R3_POSITION || cmd == SCA_W3_POSITION) && len >= 5) {
        int32_t iq24 = ((int32_t)data[1] << 24) |
                       ((int32_t)data[2] << 16) |
                       ((int32_t)data[3] << 8)  |
                       ((int32_t)data[4] << 0);
        float pos_rev = (float)iq24 / IQ24_SCALE;
        float angle_deg = pos_rev / MOTOR_SCA_REDUCTION_RATIO * 360.0f;

        g_sca_states[joint_idx].current_angle = angle_deg;
        g_sca_states[joint_idx].status = 1;  /* 有通信 */

        /* 到位判断 */
        float err = pos_rev - g_target_rev[joint_idx];
        if (err < 0) err = -err;
        g_sca_states[joint_idx].reached = (err < MOTOR_SCA_REACHED_THRESHOLD);
    }
    /* 解析CVP反馈 (R4_CVP = 0x94，11字节) */
    else if (cmd == 0x94 && len >= 11) {
        /* position在最后4字节 */
        int32_t iq24 = ((int32_t)data[7] << 24) |
                       ((int32_t)data[8] << 16) |
                       ((int32_t)data[9] << 8)  |
                       ((int32_t)data[10] << 0);
        float pos_rev = (float)iq24 / IQ24_SCALE;
        float angle_deg = pos_rev / MOTOR_SCA_REDUCTION_RATIO * 360.0f;

        g_sca_states[joint_idx].current_angle = angle_deg;
        g_sca_states[joint_idx].status = 1;

        float err = pos_rev - g_target_rev[joint_idx];
        if (err < 0) err = -err;
        g_sca_states[joint_idx].reached = (err < MOTOR_SCA_REACHED_THRESHOLD);
    }
    /* 心跳响应 */
    else if (cmd == SCA_R1_HEARTBEAT && len >= 1) {
        g_sca_states[joint_idx].status = 1;
    }
}

/* ========== TX信号量通知（由canfd_callback调用） ========== */
void motor_sca_tx_complete_notify(void)
{
    if (g_sca_tx_sem != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_sca_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

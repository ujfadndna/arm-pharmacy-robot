/**
 ******************************************************************************
 * @file    gripper.c
 * @brief   夹爪CAN控制实现
 * @note    协议来自jiazhao工程，通过STM32中转控制25kg舵机和吸盘
 ******************************************************************************
 */

#include "gripper.h"
#include "hal_data.h"
#include <string.h>

/* ========== 发送流控配置 (与motor_can.c一致) ========== */
#define CAN_TX_MAILBOX_COUNT    4       /* TXMB 0-3 */
#define CAN_TX_RETRY_MAX        100     /* 最大重试次数 */
#define CAN_TX_RETRY_DELAY_US   50      /* 重试间隔 (微秒) */

/* ========== 内部状态 ========== */
static bool g_initialized = false;
static uint8_t g_current_angle = GRIPPER_ANGLE_OPEN;
static bool g_vacuum_state = false;

/* ========== 微秒延时 ========== */
static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 40;  /* 基于200MHz */
    while (count--) {
        __NOP();
    }
}

/* ========== 内部CAN发送 (带流控) ========== */
static int gripper_can_send(const uint8_t *data, uint8_t len)
{
    can_frame_t frame = {
        .id               = GRIPPER_CAN_ID,
        .id_mode          = CAN_ID_MODE_EXTENDED,
        .type             = CAN_FRAME_TYPE_DATA,
        .data_length_code = len,
        .options          = 0
    };
    memcpy(frame.data, data, len);

    static uint8_t tx_mb_idx = 0;

    /* 尝试多个TX Mailbox，带重试 */
    for (uint32_t retry = 0; retry < CAN_TX_RETRY_MAX; retry++) {
        for (uint8_t mb = 0; mb < CAN_TX_MAILBOX_COUNT; mb++) {
            uint8_t mailbox = (tx_mb_idx + mb) % CAN_TX_MAILBOX_COUNT;

            fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, mailbox, &frame);
            if (FSP_SUCCESS == err) {
                tx_mb_idx = (mailbox + 1) % CAN_TX_MAILBOX_COUNT;
                return 0;
            }
        }
        delay_us(CAN_TX_RETRY_DELAY_US);
    }

    return -1;  /* 超时 */
}

/* ========== 公共接口实现 ========== */

void gripper_init(void)
{
    g_current_angle = GRIPPER_ANGLE_OPEN;
    g_vacuum_state = false;
    g_initialized = true;
}

int gripper_open(void)
{
    return gripper_set_angle(GRIPPER_ANGLE_OPEN);
}

int gripper_close(void)
{
    return gripper_set_angle(GRIPPER_ANGLE_CLOSE);
}

int gripper_set_angle(uint8_t angle)
{
    if (!g_initialized) return -1;

    /* 协议: {0xAE, 0x01, angle} */
    uint8_t data[3] = {
        GRIPPER_CMD_SERVO,
        0x01,           /* 使能标志 */
        angle
    };

    int ret = gripper_can_send(data, 3);
    if (ret == 0) {
        g_current_angle = angle;
    }
    return ret;
}

int vacuum_on(void)
{
    if (!g_initialized) return -1;

    /* 协议: {0xAD, 0x01} */
    uint8_t data[2] = {
        GRIPPER_CMD_VACUUM,
        0x01            /* 开启 */
    };

    int ret = gripper_can_send(data, 2);
    if (ret == 0) {
        g_vacuum_state = true;
    }
    return ret;
}

int vacuum_off(void)
{
    if (!g_initialized) return -1;

    /* 协议: {0xAD, 0x00} */
    uint8_t data[2] = {
        GRIPPER_CMD_VACUUM,
        0x00            /* 关闭 */
    };

    int ret = gripper_can_send(data, 2);
    if (ret == 0) {
        g_vacuum_state = false;
    }
    return ret;
}

void gripper_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    /* 夹爪返回数据处理 (如有需要) */
    (void)can_id;
    (void)data;
    (void)len;
}

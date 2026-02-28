/**
 ******************************************************************************
 * @file    canfd_afl.c
 * @brief   CAN FD Acceptance Filter List (AFL) 配置
 * @note    Promiscuous Mode: 接收总线上所有CAN消息
 ******************************************************************************
 */

#include "hal_data.h"

/**
 * AFL规则说明 (Promiscuous Mode):
 *
 * 配置为接收所有CAN消息:
 *   ID   = 0x00000000
 *   Mask = 0x00000000 (全0 = Don't Care, 匹配任何ID)
 *
 * mask_id = 0 表示不检查任何ID位，接收所有ID
 * mask_frame_type = 0 表示不检查帧类型，接收数据帧和远程帧
 * mask_id_mode = 0 表示不检查ID模式，接收标准帧和扩展帧
 */

/* 空规则宏定义 (用于填充未使用的AFL槽位) */
#define AFL_UNUSED_RULE \
    { \
        .id = { .id = 0, .frame_type = CAN_FRAME_TYPE_DATA, .id_mode = CAN_ID_MODE_STANDARD }, \
        .mask = { .mask_id = 0x7FF, .mask_frame_type = 0, .mask_id_mode = 0 }, \
        .destination = { .minimum_dlc = CANFD_MINIMUM_DLC_0, .rx_buffer = CANFD_RX_MB_NONE, .fifo_select_flags = 0 } \
    }

const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM] =
{
    /* Rule 0: 接收所有CAN消息 (Promiscuous Mode) */
    {
        .id =
        {
            .id         = 0x00000000,           /* 基础ID (配合mask=0，匹配任何ID) */
            .frame_type = CAN_FRAME_TYPE_DATA,  /* 数据帧 (配合mask=0，实际接收所有) */
            .id_mode    = CAN_ID_MODE_EXTENDED  /* 扩展帧 (配合mask=0，实际接收所有) */
        },
        .mask =
        {
            .mask_id         = 0x00000000,      /* 掩码=0: Don't Care, 接收任何ID */
            .mask_frame_type = 0,               /* 不检查帧类型 */
            .mask_id_mode    = 0                /* 不检查ID模式 (接收标准帧+扩展帧) */
        },
        .destination =
        {
            .minimum_dlc       = CANFD_MINIMUM_DLC_0,  /* 不检查DLC */
            .rx_buffer         = CANFD_RX_MB_NONE,     /* 不使用RX Buffer */
            .fifo_select_flags = CANFD_RX_FIFO_0       /* 发送到FIFO 0 */
        }
    },

    /* Rule 1-31: 未使用 */
    AFL_UNUSED_RULE, /* 1 */
    AFL_UNUSED_RULE, /* 2 */
    AFL_UNUSED_RULE, /* 3 */
    AFL_UNUSED_RULE, /* 4 */
    AFL_UNUSED_RULE, /* 5 */
    AFL_UNUSED_RULE, /* 6 */
    AFL_UNUSED_RULE, /* 7 */
    AFL_UNUSED_RULE, /* 8 */
    AFL_UNUSED_RULE, /* 9 */
    AFL_UNUSED_RULE, /* 10 */
    AFL_UNUSED_RULE, /* 11 */
    AFL_UNUSED_RULE, /* 12 */
    AFL_UNUSED_RULE, /* 13 */
    AFL_UNUSED_RULE, /* 14 */
    AFL_UNUSED_RULE, /* 15 */
    AFL_UNUSED_RULE, /* 16 */
    AFL_UNUSED_RULE, /* 17 */
    AFL_UNUSED_RULE, /* 18 */
    AFL_UNUSED_RULE, /* 19 */
    AFL_UNUSED_RULE, /* 20 */
    AFL_UNUSED_RULE, /* 21 */
    AFL_UNUSED_RULE, /* 22 */
    AFL_UNUSED_RULE, /* 23 */
    AFL_UNUSED_RULE, /* 24 */
    AFL_UNUSED_RULE, /* 25 */
    AFL_UNUSED_RULE, /* 26 */
    AFL_UNUSED_RULE, /* 27 */
    AFL_UNUSED_RULE, /* 28 */
    AFL_UNUSED_RULE, /* 29 */
    AFL_UNUSED_RULE, /* 30 */
    AFL_UNUSED_RULE  /* 31 */
};

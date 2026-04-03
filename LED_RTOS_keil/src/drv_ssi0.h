#ifndef DRV_SSI0_H
#define DRV_SSI0_H

#include <stdint.h>

/**
 * @brief SSI0 寄存器级 I2S 驱动 (中断 + ping-pong 缓冲)
 *
 * 引脚: P112(SSIBCK0), P113(SSILRCK0), P115(SSITXD0), P406(SSIRXD0)
 * 基地址: 0x4009D000
 * 格式: I2S Philips, 16-bit, 立体声, MCU 从机
 * 时钟: BCK/LRCK 由 WM8960 主机提供
 */

/** 缓冲区回调: DTC/中断完成后调用, 用户填充/处理下一个缓冲区 */
typedef void (*ssi0_tx_callback_t)(int16_t *next_buf, uint32_t *samples);
typedef void (*ssi0_rx_callback_t)(int16_t *filled_buf, uint32_t samples);

/** SSI0 诊断快照 */
typedef struct st_ssi0_diag
{
    uint32_t ssicr;
    uint32_t ssisr;
    uint32_t ssifcr;
    uint32_t ssifsr;
    uint32_t ssiofr;
    uint32_t ssiscr;
    uint32_t txi_count;
    uint32_t rxi_count;
} ssi0_diag_t;

/** 初始化 SSI0 外设 */
void drv_ssi0_init(void);

/** 开始播放 (中断驱动, ping-pong 双缓冲) */
void drv_ssi0_start_playback(int16_t *buf, uint32_t samples);

/** 开始录音 (中断驱动, ping-pong 双缓冲) */
void drv_ssi0_start_record(int16_t *buf, uint32_t samples);

/** 停止播放/录音 */
void drv_ssi0_stop(void);

/** 注册 TX 完成回调 */
void drv_ssi0_set_tx_callback(ssi0_tx_callback_t cb);

/** 注册 RX 完成回调 */
void drv_ssi0_set_rx_callback(ssi0_rx_callback_t cb);

/** Polling 模式: 写一个样本到 TX FIFO (用于初期调试), 成功返回0, 超时返回-1 */
int drv_ssi0_poll_write(uint32_t sample);

/** Polling 模式: 从 RX FIFO 读一个样本 */
uint32_t drv_ssi0_poll_read(void);

/** 使能 TX (开始发送) */
void drv_ssi0_tx_enable(void);

/** 使能 RX (开始接收) */
void drv_ssi0_rx_enable(void);

/** 获取 SSI0 运行状态快照（用于串口诊断） */
void drv_ssi0_get_diag(ssi0_diag_t * p_diag);

#endif /* DRV_SSI0_H */

#include "drv_ssi0.h"
#include "bsp_api.h"

/*
 * SSI0 寄存器级 I2S 驱动
 * 引脚: P112(BCK), P113(LRCK/WS), P115(TXD), P406(RXD)  PSEL=0x12
 * 基地址: 0x4009D000, MSTPCRC bit8
 * I2S Philips, 16-bit data, 32-bit system word, stereo
 * 时钟方案: SSI0 从机, BCK/LRCK 由 WM8960 主机外部输入
 */

/* ---------- 寄存器 ---------- */
#define SSI0_BASE   0x4009D000UL
#define SSICR       (*(volatile uint32_t *)(SSI0_BASE + 0x00))
#define SSISR       (*(volatile uint32_t *)(SSI0_BASE + 0x04))
#define SSIFCR      (*(volatile uint32_t *)(SSI0_BASE + 0x10))
#define SSIFSR      (*(volatile uint32_t *)(SSI0_BASE + 0x14))
#define SSIFTDR     (*(volatile uint32_t *)(SSI0_BASE + 0x18))
#define SSIFRDR     (*(volatile uint32_t *)(SSI0_BASE + 0x1C))
#define SSIOFR      (*(volatile uint32_t *)(SSI0_BASE + 0x20))
#define SSISCR      (*(volatile uint32_t *)(SSI0_BASE + 0x24))

/* MSTP */
#define MSTP_BASE   0x40084000UL
#define MSTPCRC     (*(volatile uint32_t *)(MSTP_BASE + 0x08))

/* PRCR 写保护寄存器 */
#define PRCR_REG    (*(volatile uint16_t *)(0x4001E3FEUL))
#define PRCR_UNLOCK 0xA502U
#define PRCR_LOCK   0xA500U

/* ---------- SSICR 位域 ---------- */
#define SSICR_REN       (1UL << 0)
#define SSICR_TEN       (1UL << 1)
#define SSICR_MUEN      (1UL << 3)
#define SSICR_DEL_I2S   (0UL << 8)   /* I2S: LRCK 与数据间 1bit delay */
#define SSICR_SDTA      (0UL << 10)  /* TX data: SDATA first */
#define SSICR_SPDP      (0UL << 11)  /* Padding: low */
#define SSICR_LRCKP     (0UL << 12)  /* LRCK polarity: low=L */
#define SSICR_BCKP      (0UL << 13)  /* BCK polarity: rising edge */
#define SSICR_SWL_32    (3UL << 16)  /* System word = 32 bit */
#define SSICR_DWL_16    (1UL << 19)  /* Data word = 16 bit */
#define SSICR_FRM_STEREO (0UL << 22) /* 2 channels */
#define SSICR_TUIEN     (1UL << 29)  /* TX underflow IRQ enable */
#define SSICR_TOIEN     (1UL << 28)  /* TX overflow IRQ enable */

/* ---------- SSIFCR 位域 ---------- */
#define SSIFCR_RFRST    (1UL << 0)
#define SSIFCR_TFRST    (1UL << 1)
#define SSIFCR_RIE      (1UL << 2)
#define SSIFCR_TIE      (1UL << 3)
/* RTRG: RX trigger (bits 5:4), TTRG: TX trigger (bits 7:6) */
#define SSIFCR_RTRG_1   (0UL << 4)   /* RX trigger: 1 data */
#define SSIFCR_TTRG_4   (1UL << 6)   /* TX trigger: 4 empty slots */
/* ---------- SSIFSR 位域 ---------- */
#define SSIFSR_RDF      (1UL << 0)   /* RX data full */
#define SSIFSR_TDE      (1UL << 16)  /* TX data empty */

/* ---------- SSISR 位域 ---------- */
#define SSISR_IDST      (1UL << 0)
#define SSISR_TUIRQ     (1UL << 29)
#define SSISR_TOIRQ     (1UL << 28)
#define SSISR_RUIRQ     (1UL << 27)
#define SSISR_ROIRQ     (1UL << 26)

/* ---------- PFS 引脚配置 (不依赖 RASC) ---------- */
#define PFS_BASE     0x40080800UL
/* P112: port=1, pin=12 → +0x40 + 12*4 = 0x70  */
/* P113: port=1, pin=13 → +0x40 + 13*4 = 0x74  */
/* P115: port=1, pin=15 → +0x40 + 15*4 = 0x7C  */
/* P406: port=4, pin=6  → +0x100 + 6*4 = 0x118 */
#define P112PFS      (*(volatile uint32_t *)(PFS_BASE + 0x40 + 12*4))
#define P113PFS      (*(volatile uint32_t *)(PFS_BASE + 0x40 + 13*4))
#define P115PFS      (*(volatile uint32_t *)(PFS_BASE + 0x40 + 15*4))
#define P406PFS      (*(volatile uint32_t *)(PFS_BASE + 0x100 + 6*4))

#define PWPR_ADDR    (*(volatile uint8_t *)(0x40080D03UL))
#define PWPR_B0WI    (1U << 7)
#define PWPR_PFSWE   (1U << 6)
#define PFS_PMR      (1UL << 16)
#define PFS_PSEL_SSI (0x12UL << 24)  /* PSEL=0x12 (SSI) */

static void ssi0_pin_init(void)
{
    PWPR_ADDR = 0x00;
    PWPR_ADDR = PWPR_PFSWE;

    P112PFS = PFS_PMR | PFS_PSEL_SSI;  /* P112 = SSIBCK0  */
    P113PFS = PFS_PMR | PFS_PSEL_SSI;  /* P113 = SSILRCK0 */
    P115PFS = PFS_PMR | PFS_PSEL_SSI;  /* P115 = SSITXD0  */
    P406PFS = PFS_PMR | PFS_PSEL_SSI;  /* P406 = SSIRXD0  */

    PWPR_ADDR = 0x00;
    PWPR_ADDR = PWPR_B0WI;
}

/* ---------- 回调 ---------- */
static ssi0_tx_callback_t g_tx_cb = NULL;
static ssi0_rx_callback_t g_rx_cb = NULL;
static volatile uint32_t g_ssi0_txi_count = 0;
static volatile uint32_t g_ssi0_rxi_count = 0;

/* ---------- Ping-pong 缓冲管理 ---------- */
static int16_t *g_tx_buf = NULL;
static uint32_t g_tx_samples = 0;
static uint32_t g_tx_pos = 0;

static int16_t *g_rx_buf = NULL;
static uint32_t g_rx_samples = 0;
static uint32_t g_rx_pos = 0;

void drv_ssi0_init(void)
{
    /* 0. 配置引脚 (不依赖 RASC) */
    ssi0_pin_init();

    /* 1. 使能 SSI0 模块时钟: 解锁PRCR → 清除 MSTPCRC bit8 → 锁定 */
    PRCR_REG = PRCR_UNLOCK;
    MSTPCRC &= ~(1UL << 8);
    PRCR_REG = PRCR_LOCK;
    for (volatile int i = 0; i < 100; i++) {}  /* 等待时钟稳定 */

    /* 2. 禁用 SSI (TEN=0, REN=0) */
    SSICR = 0;

    /* 3. 复位 FIFO */
    SSIFCR = SSIFCR_TFRST | SSIFCR_RFRST;

    /* 4. 配置 SSICR:
     * MST=0 (从机), DEL=0 (I2S 1-bit delay)
     * DWL=001 (16-bit), SWL=011 (32-bit system word)
     * FRM=00 (stereo), BCKP=0, LRCKP=0, SPDP=0, SDTA=0
     */
    SSICR = SSICR_DEL_I2S | SSICR_DWL_16 | SSICR_SWL_32 | SSICR_FRM_STEREO;

    /* 5. 配置 SSIFCR: 解除 FIFO 复位, 设置触发级别 */
    SSIFCR = SSIFCR_TTRG_4 | SSIFCR_RTRG_1;

    /* 6. SSIOFR: 标准 I2S 模式 */
    SSIOFR = 0;

    /* 7. SSISCR: 默认触发条件 */
    SSISCR = 0;

    /* 8. 显式打开 NVIC IRQ（避免仅有向量映射但未使能） */
    R_BSP_IrqCfgEnable((IRQn_Type)29, 6U, NULL);  /* SSI0 TXI, vector[29] */
    R_BSP_IrqCfgEnable((IRQn_Type)30, 6U, NULL);  /* SSI0 RXI, vector[30] */
}

void drv_ssi0_set_tx_callback(ssi0_tx_callback_t cb)
{
    g_tx_cb = cb;
}

void drv_ssi0_set_rx_callback(ssi0_rx_callback_t cb)
{
    g_rx_cb = cb;
}

int drv_ssi0_poll_write(uint32_t sample)
{
    /* 等待 TX FIFO 非满（TDC: FIFO 内当前数据单元数，FIFO 深度=8） */
    uint32_t guard = 5000000U;
    while (((SSIFSR >> 24) & 0x3FU) >= 8U)
    {
        if (0U == guard--)
        {
            return -1;
        }
    }
    SSIFTDR = sample;
    return 0;
}

uint32_t drv_ssi0_poll_read(void)
{
    /* 等待 RX FIFO 非空（RDC: FIFO 内当前可读数据单元数） */
    while (((SSIFSR >> 8) & 0x3FU) == 0U) {}
    return SSIFRDR;
}

void drv_ssi0_tx_enable(void)
{
    /* 复位 TX FIFO, 然后使能 */
    SSIFCR |= SSIFCR_TFRST;
    SSIFCR &= ~SSIFCR_TFRST;
    SSICR |= SSICR_TEN;
}

void drv_ssi0_rx_enable(void)
{
    SSIFCR |= SSIFCR_RFRST;
    SSIFCR &= ~SSIFCR_RFRST;
    SSICR |= SSICR_REN;
}

void drv_ssi0_start_playback(int16_t *buf, uint32_t samples)
{
    g_tx_buf = buf;
    g_tx_samples = samples;
    g_tx_pos = 0;

    /* 预填充 TX FIFO (8 个 32-bit 槽) */
    for (int i = 0; i < 8 && g_tx_pos < samples; i++) {
        /* 立体声: 左16bit | 右16bit 打包成 32-bit */
        uint32_t left  = (uint16_t)buf[g_tx_pos * 2];
        uint32_t right = (uint16_t)buf[g_tx_pos * 2 + 1];
        SSIFTDR = (left << 16) | right;
        g_tx_pos++;
    }

    /* 使能 TX 中断 + TX */
    SSIFCR |= SSIFCR_TIE;
    SSICR  |= SSICR_TEN;
}

void drv_ssi0_start_record(int16_t *buf, uint32_t samples)
{
    g_rx_buf = buf;
    g_rx_samples = samples;
    g_rx_pos = 0;

    /* 使能 RX 中断 + RX */
    SSIFCR |= SSIFCR_RIE;
    SSICR  |= SSICR_REN;
}

void drv_ssi0_stop(void)
{
    /* 禁用 TX/RX */
    SSICR &= ~(SSICR_TEN | SSICR_REN);

    /* 禁用中断 */
    SSIFCR &= ~(SSIFCR_TIE | SSIFCR_RIE);

    /* 复位 FIFO */
    SSIFCR |= SSIFCR_TFRST | SSIFCR_RFRST;
    SSIFCR &= ~(SSIFCR_TFRST | SSIFCR_RFRST);

    /* 清除错误标志 */
    SSISR &= ~(SSISR_TUIRQ | SSISR_TOIRQ | SSISR_RUIRQ | SSISR_ROIRQ);

    g_tx_buf = NULL;
    g_rx_buf = NULL;
}

void drv_ssi0_get_diag(ssi0_diag_t * p_diag)
{
    if (!p_diag)
    {
        return;
    }

    p_diag->ssicr     = SSICR;
    p_diag->ssisr     = SSISR;
    p_diag->ssifcr    = SSIFCR;
    p_diag->ssifsr    = SSIFSR;
    p_diag->ssiofr    = SSIOFR;
    p_diag->ssiscr    = SSISCR;
    p_diag->txi_count = g_ssi0_txi_count;
    p_diag->rxi_count = g_ssi0_rxi_count;
}

/*
 * SSI0 TXI 中断处理: TX FIFO 需要数据
 * 向量号在 vector_data.c 中注册
 */
void ssi0_txi_isr(void)
{
    g_ssi0_txi_count++;

    if (g_tx_buf && g_tx_pos < g_tx_samples) {
        /* 填充 FIFO (最多 4 个样本, 匹配 TTRG 触发级别) */
        for (int i = 0; i < 4 && g_tx_pos < g_tx_samples; i++) {
            uint32_t left  = (uint16_t)g_tx_buf[g_tx_pos * 2];
            uint32_t right = (uint16_t)g_tx_buf[g_tx_pos * 2 + 1];
            SSIFTDR = (left << 16) | right;
            g_tx_pos++;
        }
    } else {
        /* 缓冲区用完, 调用回调获取下一个 */
        if (g_tx_cb) {
            int16_t *next = NULL;
            uint32_t next_samples = 0;
            g_tx_cb(next, &next_samples);
            /* 回调应该设置新的 buf/samples, 这里不做 */
        }
        /* 静音填充防止 underflow */
        SSIFTDR = 0;
    }

}

/*
 * SSI0 RXI 中断处理: RX FIFO 有数据
 */
void ssi0_rxi_isr(void)
{
    g_ssi0_rxi_count++;

    if (g_rx_buf && g_rx_pos < g_rx_samples) {
        uint32_t data = SSIFRDR;
        g_rx_buf[g_rx_pos * 2]     = (int16_t)(data >> 16);
        g_rx_buf[g_rx_pos * 2 + 1] = (int16_t)(data & 0xFFFF);
        g_rx_pos++;

        if (g_rx_pos >= g_rx_samples && g_rx_cb) {
            g_rx_cb(g_rx_buf, g_rx_samples);
        }
    } else {
        /* 丢弃数据 */
        (void)SSIFRDR;
    }

}

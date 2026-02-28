/**
 ******************************************************************************
 * @file    watchdog.c
 * @brief   软件看门狗模块实现
 ******************************************************************************
 */

#include "watchdog.h"
#include "hal_data.h"

/* 看门狗状态 */
static bool g_wdt_enabled = false;

/* WDT寄存器地址 (RA6M5) */
#define WDT_BASE        0x40044200UL
#define WDT_WDTRR       (*(volatile uint8_t  *)(WDT_BASE + 0x00))  /* 刷新寄存器 */
#define WDT_WDTCR       (*(volatile uint16_t *)(WDT_BASE + 0x02))  /* 控制寄存器 */
#define WDT_WDTSR       (*(volatile uint16_t *)(WDT_BASE + 0x04))  /* 状态寄存器 */
#define WDT_WDTRCR      (*(volatile uint8_t  *)(WDT_BASE + 0x06))  /* 复位控制 */

/* 刷新序列 */
#define WDT_REFRESH_1   0x00U
#define WDT_REFRESH_2   0xFFU

void watchdog_init(bool enable)
{
    g_wdt_enabled = enable;

    if (!enable) {
        return;
    }

    /*
     * WDT配置 (软件启动模式):
     * - 时钟: PCLKB/8192 ≈ 6.1kHz (PCLKB=50MHz)
     * - 周期: 16384 cycles ≈ 2.7秒
     * - 窗口: 100% (任何时候都可以喂狗)
     * - 复位使能
     */

    /* 配置WDT控制寄存器 */
    /* WDTCR: TOPS=3(16384), CKS=7(PCLKB/8192), RPES=3(100%), RPSS=3(100%) */
    WDT_WDTCR = (3U << 0)   |  /* TOPS: 16384 cycles */
                (7U << 4)   |  /* CKS: PCLKB/8192 */
                (3U << 8)   |  /* RPES: 100% window end */
                (3U << 12);    /* RPSS: 100% window start */

    /* 启用复位输出 */
    WDT_WDTRCR = 0x80U;  /* RSTIRQS=1: 复位输出 */

    /* 启动WDT (写入刷新序列) */
    WDT_WDTRR = WDT_REFRESH_1;
    WDT_WDTRR = WDT_REFRESH_2;
}

void watchdog_refresh(void)
{
    if (!g_wdt_enabled) {
        return;
    }

    /* 写入刷新序列 */
    WDT_WDTRR = WDT_REFRESH_1;
    WDT_WDTRR = WDT_REFRESH_2;
}

bool watchdog_was_reset(void)
{
    /* 检查复位状态寄存器 (RSTSR0) */
    /* Bit 1: WDTRF - WDT复位标志 */
    volatile uint8_t *rstsr0 = (volatile uint8_t *)0x40010410UL;
    return ((*rstsr0) & 0x02U) != 0;
}

#include "drv_iic0.h"
#include "bsp_api.h"
#include "debug_uart.h"

/*
 * IIC2 寄存器级主机驱动 (polling 模式)
 * 引脚: P414 (SDA2), P415 (SCL2)  PSEL=0x07
 * 基地址: 0x4009F200, MSTPCRB bit7
 * ~125kHz Standard Mode, PCLKB=50MHz (内部上拉不足以支撑 400kHz)
 */

/* ---------- 寄存器偏移 ---------- */
#define IIC0_BASE   0x4009F200UL  /* 实际使用 IIC2 */
#define ICCR1       (*(volatile uint8_t  *)(IIC0_BASE + 0x00))
#define ICCR2       (*(volatile uint8_t  *)(IIC0_BASE + 0x01))
#define ICMR1       (*(volatile uint8_t  *)(IIC0_BASE + 0x02))
#define ICMR2       (*(volatile uint8_t  *)(IIC0_BASE + 0x03))
#define ICMR3       (*(volatile uint8_t  *)(IIC0_BASE + 0x04))
#define ICFER       (*(volatile uint8_t  *)(IIC0_BASE + 0x05))
#define ICSER       (*(volatile uint8_t  *)(IIC0_BASE + 0x06))
#define ICIER       (*(volatile uint8_t  *)(IIC0_BASE + 0x07))
#define ICSR2       (*(volatile uint8_t  *)(IIC0_BASE + 0x09))
#define ICBRL       (*(volatile uint8_t  *)(IIC0_BASE + 0x10))
#define ICBRH       (*(volatile uint8_t  *)(IIC0_BASE + 0x11))
#define ICDRT       (*(volatile uint8_t  *)(IIC0_BASE + 0x12))
#define ICDRR       (*(volatile uint8_t  *)(IIC0_BASE + 0x13))

/* ---------- 位定义 ---------- */
/* ICCR1 */
#define ICCR1_ICE       (1U << 7)
#define ICCR1_IICRST    (1U << 6)
#define ICCR1_CLO       (1U << 5)
#define ICCR1_SOWP      (1U << 4)
#define ICCR1_SCLO      (1U << 3)
#define ICCR1_SDAO      (1U << 2)
/* SCL/SDA not driven: SOWP=1 保护 + SCLO/SDAO=1 + SCLI/SDAI=1(只读) */
#define ICCR1_NOT_DRIVEN  (0x1FU)
/* ICBRL/ICBRH bits 7:5 必须写 1 (RA6M5 HW Manual) */
#define IIC_BRx_RESERVED  (0xE0U)
/* ICCR2 */
#define ICCR2_BBSY      (1U << 7)
#define ICCR2_MST       (1U << 6)
#define ICCR2_TRS       (1U << 5)
#define ICCR2_SP        (1U << 3)
#define ICCR2_RS        (1U << 2)
#define ICCR2_ST        (1U << 1)
/* ICSR2 */
#define ICSR2_TDRE      (1U << 7)
#define ICSR2_TEND      (1U << 6)
#define ICSR2_RDRF      (1U << 5)
#define ICSR2_NACKF     (1U << 4)
#define ICSR2_STOP      (1U << 3)
#define ICSR2_START     (1U << 2)
#define ICSR2_AL        (1U << 1)
/* ICMR3 */
#define ICMR3_ACKWP     (1U << 4)
#define ICMR3_ACKBT     (1U << 3)
/* ICFER */
#define ICFER_NFE        (1U << 5)
#define ICFER_NACKE      (1U << 4)
#define ICFER_SCLE       (1U << 6)
#define ICFER_MALE       (1U << 1)
/* ---------- 超时计数 ---------- */
#define IIC_TIMEOUT  50000U   /* polling 循环上限 */

/* MSTPCRB 寄存器 (模块停止控制) */
#define MSTP_BASE    0x40084000UL
#define MSTPCRB      (*(volatile uint32_t *)(MSTP_BASE + 0x04))

/* PSARB 寄存器 (外设安全属性) */
#define PSCU_BASE    0x400E0000UL
#define PSARB        (*(volatile uint32_t *)(PSCU_BASE + 0x04))

/* PRCR 写保护寄存器 — 必须先解锁才能写 MSTP */
#define PRCR         (*(volatile uint16_t *)(0x4001E3FEUL))
#define PRCR_UNLOCK  0xA502U  /* KEY=0xA5, PRC1=1 (解锁MSTP) */
#define PRCR_LOCK    0xA500U  /* KEY=0xA5, PRC1=0 (锁定) */

/* PFS 引脚功能选择寄存器 — 直接配置引脚, 不依赖 RASC 生成的 pin_data.c */
#define PFS_BASE     0x40080800UL
/* PmnPFS = PFS_BASE + port*0x40 + pin*0x04
 * P414: port=4, pin=14 → 0x40080800 + 4*0x40 + 14*0x04 = 0x40080938
 * P415: port=4, pin=15 → 0x40080800 + 4*0x40 + 15*0x04 = 0x4008093C
 */
#define P414PFS      (*(volatile uint32_t *)(PFS_BASE + 0x100 + 14*4))  /* port4 = +0x100 */
#define P415PFS      (*(volatile uint32_t *)(PFS_BASE + 0x100 + 15*4))

/* PWPR 引脚写保护寄存器 */
#define PWPR         (*(volatile uint8_t *)(0x40080D03UL))
#define PWPR_B0WI    (1U << 7)
#define PWPR_PFSWE   (1U << 6)

/* PFS 位域 */
#define PFS_PMR      (1UL << 16)   /* 1=外设模式 */
#define PFS_NMOS     (1UL << 6)    /* NMOS 开漏 */
#define PFS_PULLUP   (1UL << 4)    /* 内部上拉 */
#define PFS_PSEL_IIC (0x07UL << 24) /* PSEL=0x07 (IIC) */

static void iic0_wait_us(volatile uint32_t us)
{
    /* 微秒级延时: 200MHz Cortex-M33, ~5 cycles/iteration → 200/5=40 iterations per μs */
    volatile uint32_t count = us * 40;
    while (count--) {
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }
}

static void iic2_pin_init(void)
{
    /* 解锁 PFS 写保护: PWPR.B0WI=0, PWPR.PFSWE=1 */
    PWPR = 0x00;         /* B0WI=0 */
    PWPR = PWPR_PFSWE;   /* PFSWE=1, 允许写 PFS */

    /* 调试: 确认 PWPR 解锁状态 */
    {
        uint8_t pw = PWPR;
        debug_print("[I2C] PWPR after unlock=0x");
        debug_print_hex(pw);
        debug_println(pw & PWPR_PFSWE ? " (PFSWE=1 OK)" : " (PFSWE=0 FAIL!)");
    }

    /* P414 = IIC2_SDA: 外设模式, PSEL=IIC, NMOS开漏, 上拉 */
    P414PFS = PFS_PMR | PFS_PSEL_IIC | PFS_NMOS | PFS_PULLUP;

    /* P415 = IIC2_SCL: 同上 */
    P415PFS = PFS_PMR | PFS_PSEL_IIC | PFS_NMOS | PFS_PULLUP;

    /* 锁定 PFS */
    PWPR = 0x00;         /* PFSWE=0 */
    PWPR = PWPR_B0WI;    /* B0WI=1 */
}

void drv_iic0_init(void)
{
    /* 0. 配置 P414/P415 引脚为 IIC2 功能 (不依赖 RASC) */
    iic2_pin_init();

    /* 调试: 回读 PFS 确认引脚配置 */
    {
        uint32_t pfs414 = P414PFS;
        uint32_t pfs415 = P415PFS;
        debug_print("[I2C] P414PFS=0x");
        debug_print_hex(pfs414);
        debug_print(" P415PFS=0x");
        debug_print_hex(pfs415);
        debug_println("");
        /* 预期: PMR=1(bit16), PSEL=0x07(bit24-28), NMOS=1(bit6), PULLUP=1(bit4)
         * → 0x07010050 */
        if (!(pfs414 & PFS_PMR))
            debug_println("[I2C] WARNING: P414 PMR=0, not in peripheral mode!");
        if (((pfs414 >> 24) & 0x1F) != 0x07)
            debug_println("[I2C] WARNING: P414 PSEL != 0x07 (IIC)!");
        if (!(pfs415 & PFS_PMR))
            debug_println("[I2C] WARNING: P415 PMR=0, not in peripheral mode!");
        if (((pfs415 >> 24) & 0x1F) != 0x07)
            debug_println("[I2C] WARNING: P415 PSEL != 0x07 (IIC)!");
    }

    /* 1. 使能 IIC2 模块时钟: 解锁PRCR → 清除 MSTPCRB bit7 → 锁定 */
    PRCR = PRCR_UNLOCK;
    MSTPCRB &= ~(1UL << 7);
    PRCR = PRCR_LOCK;
    iic0_wait_us(50);  /* 模块时钟稳定 */

    /* 调试: 确认 MSTPCRB bit7 已清零 */
    {
        uint32_t mstp = MSTPCRB;
        debug_print("[I2C] MSTPCRB=0x");
        debug_print_hex(mstp);
        debug_println(mstp & (1UL << 7) ? " MSTPB7=1(FAIL! clock stopped)" : " MSTPB7=0(OK)");
    }

    /* 2. IIC 复位序列 (参考 FSP r_iic_master.c iic_master_open_hw_master) */
    ICCR1 = ICCR1_NOT_DRIVEN;                              /* 0x1F: ICE=0, SCL/SDA not driven */
    ICCR1 = ICCR1_IICRST | ICCR1_NOT_DRIVEN;              /* 0x5F: 内部复位 */
    iic0_wait_us(5);
    ICCR1 = ICCR1_ICE | ICCR1_IICRST | ICCR1_NOT_DRIVEN;  /* 0xDF: 使能+复位保持, 寄存器可写 */
    iic0_wait_us(5);

    /* 3. 配置波特率 (ICE=1, IICRST=1 状态下写入)
     * fIIC = PCLKB / (2^CKS), CKS=3 → 50MHz/8 = 6.25MHz
     * BRL=29 → tLOW=4.8us, BRH=19 → tHIGH=3.2us → ~125kHz
     */
    ICMR1 = (3U << 4) | 0x08;          /* CKS=3 + BCWP=1 */
    ICBRL = IIC_BRx_RESERVED | 0x1D;   /* 0xFD: BRL=29, 保留位=1 */
    ICBRH = IIC_BRx_RESERVED | 0x13;   /* 0xF3: BRH=19, 保留位=1 */

    /* 4. SDA 输出延迟 */
    ICMR2 = (0x03 << 4);               /* SDDL=3 */
    ICMR3 = 0x00;                      /* NF=00 (1 stage filter) */

    /* 5. 功能使能 */
    ICFER = ICFER_SCLE | ICFER_NFE | ICFER_NACKE | ICFER_MALE;

    /* 6. 禁用从机地址 */
    ICSER = 0x00;

    /* 7. 禁用中断 (polling) */
    ICIER = 0x00;

    /* 8. 释放内部复位, 总线就绪 */
    ICCR1 = ICCR1_ICE | ICCR1_NOT_DRIVEN;  /* 0x9F: ICE=1, IICRST=0 */
    iic0_wait_us(100);

    /* 调试: 确认最终状态 */
    {
        uint8_t cr1 = ICCR1;
        debug_print("[I2C] Final ICCR1=0x");
        debug_print_hex(cr1);
        debug_print(cr1 & ICCR1_ICE ? " ICE=1(OK)" : " ICE=0(BAD!)");
        debug_println("");
    }
}

/* ---------- 内部: 等待标志位 ---------- */
static int iic0_wait_flag(uint8_t flag)
{
    for (uint32_t i = 0; i < IIC_TIMEOUT; i++) {
        if (ICSR2 & flag) return 0;
    }
    return -2;  /* 超时 */
}

int drv_iic0_master_write(uint8_t addr_7bit, const uint8_t *data, uint32_t len)
{
    int ret = 0;

    /* 等待总线空闲 */
    if (ICCR2 & ICCR2_BBSY) {
        for (uint32_t i = 0; i < IIC_TIMEOUT; i++) {
            if (!(ICCR2 & ICCR2_BBSY)) break;
            if (i == IIC_TIMEOUT - 1) return -3;
        }
    }

    /* 清除状态标志 */
    ICSR2 &= ~(ICSR2_NACKF | ICSR2_STOP | ICSR2_START | ICSR2_AL);

    /* 发送 START + 从机地址 (写) */
    ICCR2 = ICCR2_MST | ICCR2_TRS | ICCR2_ST;

    /* 等待 TDRE (可以写地址) */
    if (iic0_wait_flag(ICSR2_TDRE) != 0) { ret = -2; goto cleanup; }

    /* 写从机地址 (7-bit << 1, R/W=0) */
    ICDRT = (uint8_t)(addr_7bit << 1);

    /* 等待 TEND (地址发送完成) */
    if (iic0_wait_flag(ICSR2_TEND) != 0) { ret = -2; goto cleanup; }

    /* 检查 ACK */
    if (ICSR2 & ICSR2_NACKF) { ret = -1; goto cleanup; }

    /* 发送数据字节 */
    for (uint32_t i = 0; i < len; i++) {
        ICDRT = data[i];

        /* 等待 TEND */
        if (iic0_wait_flag(ICSR2_TEND) != 0) { ret = -2; goto cleanup; }

        /* 检查 ACK */
        if (ICSR2 & ICSR2_NACKF) { ret = -1; goto cleanup; }
    }

cleanup:
    /* 发送 STOP */
    ICSR2 &= ~ICSR2_STOP;
    ICCR2 = ICCR2_SP;

    /* 等待 STOP 完成 */
    iic0_wait_flag(ICSR2_STOP);

    /* 清除 NACKF 和 STOP */
    ICSR2 &= ~(ICSR2_NACKF | ICSR2_STOP);

    return ret;
}

/* ========== 诊断函数 ========== */

void drv_iic0_scan(void)
{
    debug_println("[I2C] Scanning bus (0x03-0x77)...");

    /* 先对 0x1A (WM8960) 做详细诊断 */
    {
        uint8_t tgt = 0x1A;
        debug_println("[I2C] Detailed probe for 0x1A (WM8960):");

        debug_print("  ICCR1=0x"); debug_print_hex(ICCR1);
        debug_print(" ICCR2=0x"); debug_print_hex(ICCR2);
        debug_print(" ICSR2=0x"); debug_print_hex(ICSR2);
        debug_println("");

        /* 等待总线空闲 */
        if (ICCR2 & ICCR2_BBSY) {
            debug_println("  Bus BUSY, waiting...");
            for (uint32_t i = 0; i < IIC_TIMEOUT; i++) {
                if (!(ICCR2 & ICCR2_BBSY)) break;
            }
        }

        /* 清除状态 */
        ICSR2 &= ~(ICSR2_NACKF | ICSR2_STOP | ICSR2_START | ICSR2_AL);

        /* 发 START */
        ICCR2 = ICCR2_MST | ICCR2_TRS | ICCR2_ST;
        debug_print("  After ST: ICCR2=0x"); debug_print_hex(ICCR2);
        debug_print(" ICSR2=0x"); debug_print_hex(ICSR2);
        debug_println("");

        /* 等 TDRE */
        int r = iic0_wait_flag(ICSR2_TDRE);
        debug_print("  TDRE wait: ret="); debug_print_int(r);
        debug_print(" ICSR2=0x"); debug_print_hex(ICSR2);
        debug_println("");

        if (r == 0) {
            /* 写地址 */
            ICDRT = (uint8_t)(tgt << 1);
            debug_println("  Addr 0x34 written to ICDRT");

            /* 等 TEND */
            r = iic0_wait_flag(ICSR2_TEND);
            debug_print("  TEND wait: ret="); debug_print_int(r);
            debug_print(" ICSR2=0x"); debug_print_hex(ICSR2);
            debug_println("");

            if (ICSR2 & ICSR2_NACKF) {
                debug_println("  -> NACK received");
            } else {
                debug_println("  -> ACK received!");
            }
        }

        /* 发 STOP */
        ICSR2 &= ~ICSR2_STOP;
        ICCR2 = ICCR2_SP;
        iic0_wait_flag(ICSR2_STOP);
        ICSR2 &= ~(ICSR2_NACKF | ICSR2_STOP);
        iic0_wait_us(200);

        debug_println("");
    }

    /* 正常扫描 */
    int found = 0;

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        int ret = drv_iic0_master_write(addr, (const uint8_t *)0, 0);
        if (ret == 0) {
            debug_print("  Found device at 0x");
            debug_print_hex(addr);
            debug_println("");
            found++;
        }
        /* 短暂延时避免总线连续冲击 */
        iic0_wait_us(100);
    }

    debug_print("[I2C] Scan done. Found ");
    debug_print_int(found);
    debug_println(" device(s).");

    if (found == 0) {
        debug_println("[I2C] No devices found! Check wiring/pull-ups.");
    }
}

void drv_iic0_gpio_test(void)
{
    debug_println("[I2C] GPIO pin level test...");

    /* 1. 禁用 IIC 外设 */
    ICCR1 = 0x00;  /* ICE=0 */
    iic0_wait_us(10);

    /* 2. 解锁 PFS, 将 P414/P415 设为 GPIO 输入 + 上拉 */
    PWPR = 0x00;
    PWPR = PWPR_PFSWE;

    /* GPIO 输入模式: PMR=0 (GPIO), PDR=0 (input), 上拉使能 */
    P414PFS = PFS_PULLUP;  /* PMR=0, PSEL=0, PDR=0, PULLUP=1 */
    P415PFS = PFS_PULLUP;

    PWPR = 0x00;
    PWPR = PWPR_B0WI;

    iic0_wait_us(50);  /* 等待引脚稳定 */

    /* 3. 读引脚电平: PFS bit1 = PIDR (Pin Input Data) */
    uint32_t sda_pfs = P414PFS;
    uint32_t scl_pfs = P415PFS;
    int sda_level = (sda_pfs >> 1) & 1;
    int scl_level = (scl_pfs >> 1) & 1;

    debug_print("  P414(SDA) = ");
    debug_println(sda_level ? "HIGH" : "LOW");
    debug_print("  P415(SCL) = ");
    debug_println(scl_level ? "HIGH" : "LOW");

    if (sda_level && scl_level) {
        debug_println("  -> Pull-ups OK (both HIGH at idle)");
    } else {
        debug_println("  -> WARNING: line stuck LOW! Check for short/missing pull-up");
    }

    /* 4. 恢复 IIC 外设模式 */
    iic2_pin_init();

    /* 重新使能 IIC (FSP 正确序列) */
    ICCR1 = ICCR1_NOT_DRIVEN;
    ICCR1 = ICCR1_IICRST | ICCR1_NOT_DRIVEN;
    iic0_wait_us(5);
    ICCR1 = ICCR1_ICE | ICCR1_IICRST | ICCR1_NOT_DRIVEN;
    iic0_wait_us(5);
    ICMR1 = (3U << 4) | 0x08;
    ICBRL = IIC_BRx_RESERVED | 0x1D;
    ICBRH = IIC_BRx_RESERVED | 0x13;
    ICMR2 = (0x03 << 4);
    ICMR3 = 0x00;
    ICFER = ICFER_SCLE | ICFER_NFE | ICFER_NACKE | ICFER_MALE;
    ICSER = 0x00;
    ICIER = 0x00;
    ICCR1 = ICCR1_ICE | ICCR1_NOT_DRIVEN;
    iic0_wait_us(100);

    debug_println("[I2C] IIC2 peripheral restored.");
}

void drv_iic0_dump_regs(void)
{
    debug_println("[I2C] IIC2 register dump:");

    debug_print("  ICCR1 = 0x");
    debug_print_hex(ICCR1);
    debug_println("");

    debug_print("  ICCR2 = 0x");
    debug_print_hex(ICCR2);
    debug_println("");

    debug_print("  ICSR2 = 0x");
    debug_print_hex(ICSR2);
    debug_println("");

    debug_print("  ICMR1 = 0x");
    debug_print_hex(ICMR1);
    debug_println("");

    debug_print("  ICBRL = 0x");
    debug_print_hex(ICBRL);
    debug_println("");

    debug_print("  ICBRH = 0x");
    debug_print_hex(ICBRH);
    debug_println("");

    debug_print("  ICFER = 0x");
    debug_print_hex(ICFER);
    debug_println("");

    debug_print("  ICMR2 = 0x");
    debug_print_hex(ICMR2);
    debug_println("");

    debug_print("  ICMR3 = 0x");
    debug_print_hex(ICMR3);
    debug_println("");

    /* MSTP 模块时钟状态 */
    debug_print("  MSTPCRB = 0x");
    debug_print_hex(MSTPCRB);
    debug_println("");
    if (MSTPCRB & (1UL << 7)) {
        debug_println("  -> WARNING: MSTPB7=1, IIC2 clock is STOPPED!");
    } else {
        debug_println("  -> MSTPB7=0, IIC2 clock is running");
    }

    /* PSARB 安全属性 */
    debug_print("  PSARB  = 0x");
    debug_print_hex(PSARB);
    debug_println("");
    if (PSARB & (1UL << 7)) {
        debug_println("  -> WARNING: PSARB7=1, IIC2 is SECURE! Non-secure writes ignored!");
    } else {
        debug_println("  -> PSARB7=0, IIC2 is non-secure (OK)");
    }

    /* 预期值提示 */
    debug_println("  Expected: ICCR1=0x9F ICMR1=0x38 ICBRL=0xFD ICBRH=0xF3");
}

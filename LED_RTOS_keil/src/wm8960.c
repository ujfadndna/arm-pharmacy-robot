#include "wm8960.h"
#include "drv_iic0.h"
#include "debug_uart.h"

/*
 * WM8960 编解码器驱动
 * 移植自 STM32 参考例程, HAL_I2C_Master_Transmit → drv_iic0_master_write
 * I2C 地址: 0x1A (7-bit)
 * 寄存器协议: byte0 = (reg<<1)|((dat>>8)&1), byte1 = (dat&0xFF)
 */

#define WM8960_ADDR  0x1A

/* 影子寄存器 (WM8960 只写不可读) */
static uint16_t wm8960_regs[56] = {
    0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000A,
    0x01C0, 0x0000, 0x00FF, 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x007B, 0x0100, 0x0032, 0x0000, 0x00C3, 0x00C3, 0x01C0,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000,
    0x0000, 0x0037, 0x004D, 0x0080, 0x0008, 0x0031, 0x0026, 0x00ED
};

int wm8960_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)((reg << 1) | ((val >> 8) & 0x01));
    buf[1] = (uint8_t)(val & 0xFF);

    int ret = drv_iic0_master_write(WM8960_ADDR, buf, 2);
    if (ret == 0 && reg < 56)
        wm8960_regs[reg] = val;
    return ret;
}

static void wm8960_delay_short(void)
{
    for (volatile uint32_t i = 0; i < 20000U; i++)
    {
        /* busy wait */
    }
}

static int wm8960_write_checked(uint8_t reg, uint16_t val)
{
    int last_ret = -1;
    for (int attempt = 0; attempt < 3; attempt++)
    {
        int ret = wm8960_write_reg(reg, val);
        if (ret == 0)
        {
            return 0;
        }

        last_ret = ret;
        wm8960_delay_short();
    }

    debug_print("[WM8960] write fail reg=0x");
    debug_print_hex(reg);
    debug_print(" val=0x");
    debug_print_hex(val);
    debug_print(" ret=");
    debug_print_int(last_ret);
    debug_println("");

    return -1;
}

uint16_t wm8960_read_reg(uint8_t reg)
{
    if (reg < 56)
        return wm8960_regs[reg];
    return 0;
}

int wm8960_init(void)
{
    /*
     * 初始化顺序对齐 Waveshare WM8960_Audio_Board_Code 参考例程
     * 关键变更: 去掉 PLL, 直接使用板载 24MHz MCLK
     * WM8960 工作在 I2S master 模式, 由 WM8960 输出 BCK/LRCK 给 MCU SSI0
     */

    /* 1. 软件复位 */
    if (wm8960_write_checked(WM8960_REG_RESET, 0x0000) != 0) return -1;

    /* 2. 电源管理
     * Power1 (0x19): VMIDSEL=11(bit8:7, 快速启动), VREF=1(bit6)
     *   = 0x01C0 (对齐参考例程, ADC/MIC 电源在 config_record 中按需开启)
     * Power2 (0x1A): DACL=1, DACR=1, LOUT1=1, ROUT1=1, SPKL=1, SPKR=1
     *   = 0x01F8
     * Power3 (0x2F): LOMIX=1, ROMIX=1 (播放不需要 MIC power)
     *   = 0x000C
     */
    if (wm8960_write_checked(WM8960_REG_POWER1, 0x01C0) != 0) return -1;  /* VMIDSEL=11(bit8:7), VREF=1(bit6) */
    /* VMID 电容充电: 数据手册要求 ≥100ms (fast startup mode) */
    for (volatile uint32_t _d = 0; _d < 4000000U; _d++) { /* ~100ms @200MHz */ }
    if (wm8960_write_checked(WM8960_REG_POWER2, 0x01F8) != 0) return -1;
    if (wm8960_write_checked(WM8960_REG_POWER3, 0x000C) != 0) return -1;

    /* 3. 时钟: 不用 PLL, MCLK=24MHz 直通
     * CLOCK1 (0x04) = 0x0090: DACDIV=/512, ADCDIV=/512 → fs≈46.875kHz
     * CLOCK2 (0x08) = 0x0007: BCLKDIV=8 → BCK=3MHz, LRCK=46.875kHz
     * 注: 24MHz 无法精确产生 48kHz, 46.875kHz 偏差 2.3%, 可接受
     */
    if (wm8960_write_checked(WM8960_REG_CLOCK1, 0x0090) != 0) return -1;  /* DACDIV=/512, ADCDIV=/512 */
    if (wm8960_write_checked(WM8960_REG_CLOCK2, 0x0007) != 0) return -1;  /* BCLKDIV=/8 → BCK=3MHz */

    /* 4. DAC 控制: unmute */
    if (wm8960_write_checked(WM8960_REG_DACCTL1, 0x0000) != 0) return -1;

    /* 6. HP 输出音量 (参考例程先设 HP 再设 SPK) */
    if (wm8960_write_checked(WM8960_REG_LOUT1, 0x006F | 0x0100) != 0) return -1;
    if (wm8960_write_checked(WM8960_REG_ROUT1, 0x006F | 0x0100) != 0) return -1;

    /* 7. SPK 输出音量 */
    if (wm8960_write_checked(WM8960_REG_LOUT2, 0x007F | 0x0100) != 0) return -1;
    if (wm8960_write_checked(WM8960_REG_ROUT2, 0x007F | 0x0100) != 0) return -1;

    /* 8. Class D 扬声器使能 */
    if (wm8960_write_checked(WM8960_REG_CLASSD1, 0x00F7) != 0) return -1;

    /* 9. DAC 音量: 0xFF = 0dB */
    if (wm8960_write_checked(WM8960_REG_LDAC, 0x00FF | 0x0100) != 0) return -1;
    if (wm8960_write_checked(WM8960_REG_RDAC, 0x00FF | 0x0100) != 0) return -1;

    /* 10. 输出混音器: DAC → Output Mixer
     * bit8 = LD2LO/RD2RO (DAC to output mixer)
     * bit7 = LI2LO/RI2RO (参考例程也开了此位) */
    if (wm8960_write_checked(WM8960_REG_LOUTMIX, 0x0180) != 0) return -1;  /* DAC→mixer + LINE→mixer */
    if (wm8960_write_checked(WM8960_REG_ROUTMIX, 0x0180) != 0) return -1;  /* DAC→mixer + LINE→mixer */

    /* 11. Jack detect / Additional controls (参考例程) */
    if (wm8960_write_checked(WM8960_REG_ADDCTL2, 0x0040) != 0) return -1;  /* HPSWEN=1 */
    if (wm8960_write_checked(WM8960_REG_ADDCTL1, 0x0183) != 0) return -1;  /* TOEN=1, TOCLKSEL=1, DATSEL=00(正常L/R), DMONOMIX=0 */
    if (wm8960_write_checked(WM8960_REG_ADDCTL4, 0x0009) != 0) return -1;  /* GPIOPOL=0, GPIOSEL=001, HPSEL=01 */

    /* 12. 音频接口放在最后切到 master，避免影响前序寄存器写入稳定性 */
    if (wm8960_write_checked(WM8960_REG_IFACE1, 0x0042) != 0) return -1;

    return 0;
}

int wm8960_config_playback(void)
{
    /* 播放通路已在 wm8960_init() 中完整配置 (DAC vol, mixer, HP/SPK vol, ClassD)
     * 此函数保留接口兼容, 无需额外操作 */
    return 0;
}

int wm8960_config_record(void)
{
    int r = 0;

    /* 补充录音所需的电源位 (init 中 POWER1=0x01C0 开了 VREF+VMID) */
    uint16_t pwr1 = wm8960_read_reg(WM8960_REG_POWER1);
    pwr1 |= (1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1);  /* AINL, AINR, ADCL, ADCR, MICB */
    r += wm8960_write_reg(WM8960_REG_POWER1, pwr1);

    /* 补充录音所需的电源位: LMIC, RMIC (init 中 POWER3=0x000C 只开了 mixer) */
    uint16_t pwr3 = wm8960_read_reg(WM8960_REG_POWER3);
    pwr3 |= (1<<5) | (1<<4);  /* LMIC=1, RMIC=1 */
    r += wm8960_write_reg(WM8960_REG_POWER3, pwr3);

    /* 输入 PGA 音量: 0x3F = +30dB, bit8=VU update */
    r += wm8960_write_reg(WM8960_REG_LINVOL, 0x003F | 0x0100);
    r += wm8960_write_reg(WM8960_REG_RINVOL, 0x003F | 0x0100);

    /* 输入信号路径: 板载麦克风 (LINPUT1 → Left PGA → Left Boost) */
    r += wm8960_write_reg(WM8960_REG_LINPATH, 0x0008 | 0x0100);  /* LMN1=1, LMIC2B=1 */
    r += wm8960_write_reg(WM8960_REG_RINPATH, 0x0000);            /* 右通道不用 */

    /* Boost Mixer: 不额外增益 */
    r += wm8960_write_reg(WM8960_REG_INBMIX1, 0x0000);
    r += wm8960_write_reg(WM8960_REG_INBMIX2, 0x0000);

    /* ADC 数字音量 */
    r += wm8960_write_reg(WM8960_REG_LADC, 0x00C3 | 0x0100);
    r += wm8960_write_reg(WM8960_REG_RADC, 0x00C3 | 0x0100);

    /* 噪声门 */
    r += wm8960_write_reg(WM8960_REG_NOISEG, 0x00F9);

    return (r != 0) ? -1 : 0;
}

void wm8960_set_volume(uint8_t vol)
{
    /* vol: 0~100 → 寄存器值 0x30~0x7F (48~127) */
    if (vol > 100) vol = 100;
    uint16_t regval = 0x30 + (uint16_t)vol * (0x7F - 0x30) / 100;
    regval |= 0x0100;  /* VU update bit */

    wm8960_write_reg(WM8960_REG_LOUT1, regval);
    wm8960_write_reg(WM8960_REG_ROUT1, regval);
    wm8960_write_reg(WM8960_REG_LOUT2, regval);
    wm8960_write_reg(WM8960_REG_ROUT2, regval);
}

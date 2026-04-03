#ifndef WM8960_H
#define WM8960_H

#include <stdint.h>

/**
 * @brief WM8960 音频编解码器驱动
 *
 * I2C 地址: 0x1A (7-bit)
 * 时钟: 板载 24MHz MCLK 直通 (无 PLL), I2S master 输出 BCK/LRCK
 * 移植自 STM32 参考例程, 使用 drv_iic0 通信
 */

/* WM8960 寄存器地址 */
#define WM8960_REG_LINVOL       0x00
#define WM8960_REG_RINVOL       0x01
#define WM8960_REG_LOUT1        0x02
#define WM8960_REG_ROUT1        0x03
#define WM8960_REG_CLOCK1       0x04
#define WM8960_REG_DACCTL1      0x05
#define WM8960_REG_DACCTL2      0x06
#define WM8960_REG_IFACE1       0x07
#define WM8960_REG_CLOCK2       0x08
#define WM8960_REG_IFACE2       0x09
#define WM8960_REG_LDAC         0x0A
#define WM8960_REG_RDAC         0x0B
#define WM8960_REG_RESET        0x0F
#define WM8960_REG_3D           0x10
#define WM8960_REG_ALC1         0x11
#define WM8960_REG_ALC2         0x12
#define WM8960_REG_ALC3         0x13
#define WM8960_REG_NOISEG       0x14
#define WM8960_REG_LADC         0x15
#define WM8960_REG_RADC         0x16
#define WM8960_REG_ADDCTL1      0x17
#define WM8960_REG_ADDCTL2      0x18
#define WM8960_REG_POWER1       0x19
#define WM8960_REG_POWER2       0x1A
#define WM8960_REG_ADDCTL3      0x1B
#define WM8960_REG_APOP1        0x1C
#define WM8960_REG_APOP2        0x1D
#define WM8960_REG_LINPATH      0x20
#define WM8960_REG_RINPATH      0x21
#define WM8960_REG_LOUTMIX      0x22
#define WM8960_REG_ROUTMIX      0x25
#define WM8960_REG_MONOMIX1     0x26
#define WM8960_REG_MONOMIX2     0x27
#define WM8960_REG_LOUT2        0x28
#define WM8960_REG_ROUT2        0x29
#define WM8960_REG_MONO         0x2A
#define WM8960_REG_INBMIX1      0x2B
#define WM8960_REG_INBMIX2      0x2C
#define WM8960_REG_BYPASS1      0x2D
#define WM8960_REG_BYPASS2      0x2E
#define WM8960_REG_POWER3       0x2F
#define WM8960_REG_ADDCTL4      0x30
#define WM8960_REG_CLASSD1      0x31
#define WM8960_REG_CLASSD3      0x33
#define WM8960_REG_PLL1         0x34
#define WM8960_REG_PLL2         0x35
#define WM8960_REG_PLL3         0x36
#define WM8960_REG_PLL4         0x37

/** 初始化 WM8960: 复位 + 电源 + 时钟/PLL + I2S 格式 */
int wm8960_init(void);

/** 配置播放通路: DAC 音量 + 混音器 + HP/SPK 输出 */
int wm8960_config_playback(void);

/** 配置录音通路: 输入 PGA + Boost + ADC + 噪声门 */
int wm8960_config_record(void);

/** 音量控制 (0~100 映射到 HP 输出) */
void wm8960_set_volume(uint8_t vol);

/** 写单个寄存器 (供调试用) */
int wm8960_write_reg(uint8_t reg, uint16_t val);

/** 读影子寄存器 (WM8960 不可读, 返回缓存值) */
uint16_t wm8960_read_reg(uint8_t reg);

#endif /* WM8960_H */

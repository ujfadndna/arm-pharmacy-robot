#ifndef DRV_IIC0_H
#define DRV_IIC0_H

#include <stdint.h>

/**
 * @brief IIC0 寄存器级主机驱动 (polling 模式)
 *
 * 引脚: P414 (SDA2), P415 (SCL2) — 使用 IIC2 外设
 * 基地址: 0x4009F200
 * 速率: 400kHz Fast Mode (PCLKB=50MHz)
 */

/** 初始化 IIC0 外设 (使能模块时钟, 配置寄存器, 400kHz) */
void drv_iic0_init(void);

/**
 * @brief I2C 主机写操作 (polling)
 * @param addr_7bit  7-bit 从机地址
 * @param data       发送数据缓冲区
 * @param len        数据长度 (字节)
 * @return 0=成功, -1=NACK, -2=超时, -3=总线忙
 */
int drv_iic0_master_write(uint8_t addr_7bit, const uint8_t *data, uint32_t len);

/** I2C bus scan: 扫描 0x03~0x77, 通过 debug_println 输出结果 */
void drv_iic0_scan(void);

/** I2C GPIO 回读: 临时切 GPIO 读 P414/P415 电平 */
void drv_iic0_gpio_test(void);

/** I2C 寄存器 dump */
void drv_iic0_dump_regs(void);

#endif /* DRV_IIC0_H */

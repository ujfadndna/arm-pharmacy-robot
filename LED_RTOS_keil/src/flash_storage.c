/**
 * @file    flash_storage.c
 * @brief   Data Flash 简单读写封装
 *
 * RASC 未生成 g_flash0 实例，在此手动声明。
 * BGO (Background Operation) 已关闭，所有操作均为阻塞同步模式。
 */

#include "flash_storage.h"
#include "r_flash_hp.h"
#include "debug_uart.h"
#include <string.h>

/* ========== Data Flash 常量 ========== */
#define DF_BASE_ADDR     (0x08000000U)  /* RA6M5 Data Flash 起始地址 */
#define DF_BLOCK_SIZE    (64U)          /* 每块 64 字节（擦除最小单位）*/
#define DF_WRITE_SIZE    (4U)           /* 写入最小单位 4 字节 */
#define FLASH_MAGIC      (0x5A7E1234U)  /* 用于判断数据是否有效 */
#define SAVE_BUF_SIZE    (512U)         /* 最大存储缓冲区 */

/* ========== Flash 驱动实例（手动声明，不依赖 RASC 生成）========== */
static flash_hp_instance_ctrl_t g_flash0_ctrl;
static const flash_cfg_t g_flash0_cfg =
{
    .data_flash_bgo = false,    /* 关闭 BGO，使用阻塞模式 */
    .p_callback     = NULL,     /* BGO 关闭时不需要回调 */
    .p_extend       = NULL,
    .p_context      = NULL,
    .ipl            = (BSP_IRQ_DISABLED),
    .irq            = FSP_INVALID_VECTOR,
    .err_ipl        = (BSP_IRQ_DISABLED),
    .err_irq        = FSP_INVALID_VECTOR,
};

/* ========== 头部结构（8 字节）========== */
typedef struct {
    uint32_t magic;     /* FLASH_MAGIC */
    uint32_t crc;       /* 数据区 CRC */
} FlashHeader_t;

/* ========== 内部工具 ========== */

static uint32_t calc_crc(const void *data, uint32_t size)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0;
    for (uint32_t i = 0; i < size; i++) {
        crc += p[i];
    }
    return crc ^ 0xDEADBEEFU;
}

/* ========== 公共接口 ========== */

bool flash_storage_save(const void *data, uint32_t size)
{
    if (data == NULL || size == 0) return false;

    /* 计算总写入字节（头 + 数据），向上对齐到 DF_WRITE_SIZE */
    uint32_t total  = (uint32_t)sizeof(FlashHeader_t) + size;
    uint32_t padded = (total + DF_WRITE_SIZE - 1U) & ~(DF_WRITE_SIZE - 1U);
    uint32_t blocks = (padded + DF_BLOCK_SIZE - 1U) / DF_BLOCK_SIZE;

    if (padded > SAVE_BUF_SIZE) {
        debug_println("[FLASH] Data too large to save");
        return false;
    }

    /* 在 RAM 里构建写入缓冲区 */
    static uint8_t buf[SAVE_BUF_SIZE];
    memset(buf, 0xFFU, sizeof(buf));

    FlashHeader_t *hdr = (FlashHeader_t *)buf;
    hdr->magic = FLASH_MAGIC;
    hdr->crc   = calc_crc(data, size);
    memcpy(buf + sizeof(FlashHeader_t), data, size);

    /* 开启 Flash 驱动 */
    fsp_err_t err = R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
    if (err != FSP_SUCCESS && err != FSP_ERR_ALREADY_OPEN) {
        debug_println("[FLASH] Open failed");
        return false;
    }

    /* 擦除目标块 */
    err = R_FLASH_HP_Erase(&g_flash0_ctrl, DF_BASE_ADDR, blocks);
    if (err != FSP_SUCCESS) {
        debug_println("[FLASH] Erase failed");
        R_FLASH_HP_Close(&g_flash0_ctrl);
        return false;
    }

    /* 写入数据 */
    err = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)buf, DF_BASE_ADDR, padded);
    R_FLASH_HP_Close(&g_flash0_ctrl);

    if (err != FSP_SUCCESS) {
        debug_println("[FLASH] Write failed");
        return false;
    }

    return true;
}

bool flash_storage_load(void *data, uint32_t size)
{
    if (data == NULL || size == 0) return false;

    /* Data Flash 可直接内存映射读取，无需驱动 */
    const FlashHeader_t *hdr = (const FlashHeader_t *)DF_BASE_ADDR;

    if (hdr->magic != FLASH_MAGIC) {
        return false;  /* 没有有效数据 */
    }

    const uint8_t *src = (const uint8_t *)(DF_BASE_ADDR + sizeof(FlashHeader_t));
    uint32_t crc = calc_crc(src, size);
    if (crc != hdr->crc) {
        debug_println("[FLASH] CRC mismatch");
        return false;
    }

    memcpy(data, src, size);
    return true;
}

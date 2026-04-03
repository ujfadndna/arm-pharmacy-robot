/**
 * @file    flash_storage.h
 * @brief   Data Flash 简单读写封装（阻塞模式，BGO 关闭）
 *
 * 存储布局 (0x08000000):
 *   [0..7]  FlashHeader_t  (magic + crc + size + reserved)
 *   [8..N]  实际数据
 */

#ifndef FLASH_STORAGE_H_
#define FLASH_STORAGE_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 将数据写入 Data Flash
 * @param data 数据指针（必须在 RAM 中）
 * @param size 数据字节数
 * @return true=成功, false=失败
 */
bool flash_storage_save(const void *data, uint32_t size);

/**
 * @brief 从 Data Flash 读取数据（校验 magic + CRC）
 * @param data 输出缓冲区
 * @param size 期望读取的字节数（必须与写入时一致）
 * @return true=成功, false=无有效数据或校验失败
 */
bool flash_storage_load(void *data, uint32_t size);

#endif /* FLASH_STORAGE_H_ */

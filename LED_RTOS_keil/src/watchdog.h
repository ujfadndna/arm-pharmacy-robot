/**
 ******************************************************************************
 * @file    watchdog.h
 * @brief   软件看门狗模块 (基于WDT外设)
 * @note    调试时可禁用，演示时启用
 ******************************************************************************
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化看门狗 (软件启动模式)
 * @param enable true=启用看门狗, false=不启用
 * @note  超时时间约 2 秒
 */
void watchdog_init(bool enable);

/**
 * @brief 喂狗 (刷新看门狗计数器)
 * @note  必须在超时前调用，否则系统复位
 */
void watchdog_refresh(void);

/**
 * @brief 检查是否因看门狗复位
 * @return true=上次是看门狗复位, false=正常上电
 */
bool watchdog_was_reset(void);

#endif /* WATCHDOG_H_ */

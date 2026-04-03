#ifndef VACUUM_PUMP_H
#define VACUUM_PUMP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * 引脚定义
 * P600 → 继电器 IN（高电平触发）→ 真空泵 12V
 * ---------------------------------------------------------------- */
#define VACUUM_PUMP_PIN   BSP_IO_PORT_06_PIN_00

/* 继电器逻辑（常见光耦隔离模块，高电平触发） */
#define VACUUM_PUMP_ON_LEVEL   BSP_IO_LEVEL_HIGH
#define VACUUM_PUMP_OFF_LEVEL  BSP_IO_LEVEL_LOW

/* ----------------------------------------------------------------
 * API
 * ---------------------------------------------------------------- */

/**
 * @brief 初始化真空泵 GPIO，默认关闭
 *        在系统初始化阶段调用一次
 */
void vacuum_pump_init(void);

/**
 * @brief 开启真空泵（P600 拉高 → 继电器吸合 → 12V 供泵）
 */
void vacuum_pump_on(void);

/**
 * @brief 关闭真空泵（P600 拉低 → 继电器释放）
 */
void vacuum_pump_off(void);

/**
 * @brief 查询真空泵当前状态
 * @return true = 运行中，false = 已停止
 */
bool vacuum_pump_is_on(void);

#ifdef __cplusplus
}
#endif

#endif /* VACUUM_PUMP_H */

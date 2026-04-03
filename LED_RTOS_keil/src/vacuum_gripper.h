/**
 ******************************************************************************
 * @file    vacuum_gripper.h
 * @brief   真空泵夹爪驱动接口
 * @note    通过继电器控制真空泵与三通电磁阀，实现吸附/释放
 ******************************************************************************
 */

#ifndef VACUUM_GRIPPER_H
#define VACUUM_GRIPPER_H

#include <stdint.h>
#include <stdbool.h>

/* 引脚定义 */
#define VACUUM_PUMP_PIN   BSP_IO_PORT_06_PIN_00   /* P600 → 继电器1 → 真空泵12V */
#define VACUUM_VALVE_PIN  BSP_IO_PORT_06_PIN_01   /* P601 → 继电器2 → 三通电磁阀12V */

/* 继电器逻辑：高电平触发（常见光耦隔离继电器模块） */
#define RELAY_ON   BSP_IO_LEVEL_HIGH
#define RELAY_OFF  BSP_IO_LEVEL_LOW

/* 时序参数（ms） */
#define VACUUM_PUMP_STARTUP_MS   50U   /* 泵启动后等待建立负压 */
#define VACUUM_VALVE_CLOSE_MS   100U   /* 阀关闭后等待气压平衡 */

void vacuum_gripper_init(void);
void vacuum_gripper_suck(void);
void vacuum_gripper_release(void);
bool vacuum_gripper_is_sucking(void);

#endif /* VACUUM_GRIPPER_H */


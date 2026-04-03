/**
 ******************************************************************************
 * @file    vacuum_gripper.c
 * @brief   真空泵夹爪驱动实现
 * @note    通过继电器控制真空泵与电磁阀，实现吸附/释放功能
 ******************************************************************************
 */

#include "vacuum_gripper.h"
#include "hal_data.h"
#include "r_ioport.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========== 内部状态 ========== */

/* 初始化标志 */
static bool g_initialized = false;

/* 当前是否处于吸附状态 */
static bool g_sucking = false;

/* ========== 静态辅助函数 ========== */

/**
 * @brief 配置真空泵与电磁阀引脚为输出并默认关闭
 */
static void vacuum_gripper_config_pins(void)
{
    /* 配置P600/P601为推挽输出，高电平有效 */
    R_IOPORT_PinCfg(&g_ioport_ctrl, VACUUM_PUMP_PIN,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW);
    R_IOPORT_PinCfg(&g_ioport_ctrl, VACUUM_VALVE_PIN,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW);

    /* 默认关闭继电器 */
    R_IOPORT_PinWrite(&g_ioport_ctrl, VACUUM_PUMP_PIN, RELAY_OFF);
    R_IOPORT_PinWrite(&g_ioport_ctrl, VACUUM_VALVE_PIN, RELAY_OFF);
}

/* ========== 公共接口 ========== */

/**
 * @brief 真空夹爪驱动初始化
 *
 * 配置控制真空泵与电磁阀的GPIO为输出，并默认关闭。
 */
void vacuum_gripper_init(void)
{
    if (g_initialized)
    {
        return;
    }

    vacuum_gripper_config_pins();

    g_sucking = false;
    g_initialized = true;
}

/**
 * @brief 启动吸附
 *
 * 顺序：
 * 1. 打开真空泵继电器（P600 高电平）
 * 2. 延时 VACUUM_PUMP_STARTUP_MS 毫秒等待负压建立
 * 3. 打开电磁阀继电器（P601 高电平）
 */
void vacuum_gripper_suck(void)
{
    if (!g_initialized)
    {
        vacuum_gripper_init();
    }

    /* 先启动真空泵 */
    R_IOPORT_PinWrite(&g_ioport_ctrl, VACUUM_PUMP_PIN, RELAY_ON);
    vTaskDelay(pdMS_TO_TICKS(VACUUM_PUMP_STARTUP_MS));

    /* 再打开电磁阀，建立吸附通路 */
    R_IOPORT_PinWrite(&g_ioport_ctrl, VACUUM_VALVE_PIN, RELAY_ON);

    g_sucking = true;
}

/**
 * @brief 释放吸附
 *
 * 顺序：
 * 1. 先关闭电磁阀继电器（P601 低电平），切断真空通路
 * 2. 延时 VACUUM_VALVE_CLOSE_MS 毫秒等待气压平衡
 * 3. 再关闭真空泵继电器（P600 低电平）
 */
void vacuum_gripper_release(void)
{
    if (!g_initialized)
    {
        vacuum_gripper_init();
    }

    /* 先关闭电磁阀，防止瞬间冲击 */
    R_IOPORT_PinWrite(&g_ioport_ctrl, VACUUM_VALVE_PIN, RELAY_OFF);
    vTaskDelay(pdMS_TO_TICKS(VACUUM_VALVE_CLOSE_MS));

    /* 再关闭真空泵 */
    R_IOPORT_PinWrite(&g_ioport_ctrl, VACUUM_PUMP_PIN, RELAY_OFF);

    g_sucking = false;
}

/**
 * @brief 查询当前是否处于吸附状态
 *
 * @return true: 正在吸附, false: 未吸附
 */
bool vacuum_gripper_is_sucking(void)
{
    return g_sucking;
}


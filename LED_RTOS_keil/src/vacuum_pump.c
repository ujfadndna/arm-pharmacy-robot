/**
 * @file vacuum_pump.c
 * @brief 真空泵驱动
 *
 * 通过 GPIO P600 控制 5V 继电器，继电器切换 12V 为真空泵供电。
 * 高电平触发：P600 HIGH → 继电器吸合 → 真空泵启动。
 */

#include "vacuum_pump.h"
#include "hal_data.h"

/* ----------------------------------------------------------------
 * 内部状态
 * ---------------------------------------------------------------- */
static bool g_initialized = false;
static bool g_pump_on     = false;

/* ----------------------------------------------------------------
 * 公开 API
 * ---------------------------------------------------------------- */

void vacuum_pump_init(void)
{
    /* 配置 P600 为推挽输出，初始低电平（泵默认关闭） */
    R_IOPORT_PinCfg(&g_ioport_ctrl,
                    VACUUM_PUMP_PIN,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW);

    g_pump_on     = false;
    g_initialized = true;
}

void vacuum_pump_on(void)
{
    if (!g_initialized)
    {
        return;
    }

    R_IOPORT_PinWrite(&g_ioport_ctrl, VACUUM_PUMP_PIN, VACUUM_PUMP_ON_LEVEL);
    g_pump_on = true;
}

void vacuum_pump_off(void)
{
    if (!g_initialized)
    {
        return;
    }

    R_IOPORT_PinWrite(&g_ioport_ctrl, VACUUM_PUMP_PIN, VACUUM_PUMP_OFF_LEVEL);
    g_pump_on = false;
}

bool vacuum_pump_is_on(void)
{
    return g_pump_on;
}

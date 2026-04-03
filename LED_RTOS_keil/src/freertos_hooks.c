/* FreeRTOS Hooks */
#include "FreeRTOS.h"
#include "task.h"
#include "hal_data.h"

void vApplicationIdleHook(void)
{
    /* 空实现 */
}

/* 栈溢出检测钩子 - configCHECK_FOR_STACK_OVERFLOW >= 1 时需要 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    /* LED快闪报警 + 死循环，方便调试器断点 */
    __disable_irq();
    for (;;) {
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_HIGH);
        for (volatile uint32_t i = 0; i < 200000; i++) {}
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_LOW);
        for (volatile uint32_t i = 0; i < 200000; i++) {}
    }
}

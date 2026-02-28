/**
 * @file    debug_uart.c
 * @brief   UART4调试输出/输入实现 (P511-RXD4, P512-TXD4)
 */

#include "debug_uart.h"
#include "hal_data.h"
#include "new_thread0.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

static volatile bool g_debug_tx_complete = true;

/* 发送完成信号量 (DTC优化) */
static SemaphoreHandle_t g_tx_sem = NULL;
static StaticSemaphore_t g_tx_sem_buffer;

/* 接收缓冲区 */
#define DEBUG_RX_BUF_SIZE 256
static char g_rx_buffer[DEBUG_RX_BUF_SIZE];
static volatile uint16_t g_rx_idx = 0;
static volatile bool g_rx_line_ready = false;

void debug_uart_init(void)
{
    g_debug_tx_complete = true;
    g_rx_idx = 0;
    g_rx_line_ready = false;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));

    /* 创建发送完成信号量 (DTC优化) */
    if (g_tx_sem == NULL) {
        g_tx_sem = xSemaphoreCreateBinaryStatic(&g_tx_sem_buffer);
    }

    R_SCI_UART_Open(&g_uart4_ctrl, &g_uart4_cfg);
}

void debug_print(const char *str)
{
    if (!str) return;
    uint32_t len = strlen(str);
    if (len == 0) return;

    /* 简单轮询等待上次发送完成 */
    uint32_t wait = 100000;
    while (!g_debug_tx_complete && wait > 0) {
        wait--;
    }

    g_debug_tx_complete = false;
    R_SCI_UART_Write(&g_uart4_ctrl, (uint8_t *)str, len);

    /* 等待发送完成 */
    wait = 100000;
    while (!g_debug_tx_complete && wait > 0) {
        wait--;
    }

    /* 超时后强制设为完成，防止下次卡死 */
    g_debug_tx_complete = true;
}

void debug_println(const char *str)
{
    debug_print(str);
    debug_print("\r\n");
}

void debug_print_int(int value)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", value);
    debug_print(buf);
}

void debug_print_hex(uint32_t value)
{
    char buf[16];
    /* 根据值大小选择格式 */
    if (value <= 0xFF) {
        snprintf(buf, sizeof(buf), "%02X", (unsigned int)value);
    } else if (value <= 0xFFFF) {
        snprintf(buf, sizeof(buf), "%04X", (unsigned int)value);
    } else {
        snprintf(buf, sizeof(buf), "%08lX", (unsigned long)value);
    }
    debug_print(buf);
}

void debug_tx_complete(void)
{
    g_debug_tx_complete = true;

    /* 释放信号量，唤醒等待的任务 (从ISR调用) */
    if (g_tx_sem != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* 接收功能 */
bool debug_has_line(void)
{
    return g_rx_line_ready;
}

int debug_read_line(char *buf, uint32_t buf_len)
{
    if (!g_rx_line_ready || buf == NULL || buf_len == 0) {
        return -1;
    }

    /* 复制数据 */
    uint32_t copy_len = (g_rx_idx < buf_len - 1) ? g_rx_idx : (buf_len - 1);
    memcpy(buf, g_rx_buffer, copy_len);
    buf[copy_len] = '\0';

    /* 去掉末尾的\r\n */
    while (copy_len > 0 && (buf[copy_len-1] == '\r' || buf[copy_len-1] == '\n')) {
        buf[--copy_len] = '\0';
    }

    /* 清空接收缓冲区 */
    g_rx_idx = 0;
    g_rx_line_ready = false;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));

    return (int)copy_len;
}

/* UART4回调 */
void uart4_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            debug_tx_complete();
            break;

        case UART_EVENT_RX_CHAR:
        {
            uint8_t ch = (uint8_t)p_args->data;

            /* 回显字符 */
            if (ch >= 0x20 && ch < 0x7F) {
                char echo[2] = {(char)ch, '\0'};
                /* 注意：不能在ISR中调用debug_print，直接写 */
                R_SCI_UART_Write(&g_uart4_ctrl, (uint8_t *)echo, 1);
            } else if (ch == '\r' || ch == '\n') {
                R_SCI_UART_Write(&g_uart4_ctrl, (uint8_t *)"\r\n", 2);
            }

            /* 存入缓冲区 */
            if (ch == '\r' || ch == '\n') {
                if (g_rx_idx > 0) {
                    g_rx_line_ready = true;
                }
            } else if (g_rx_idx < DEBUG_RX_BUF_SIZE - 1) {
                g_rx_buffer[g_rx_idx++] = (char)ch;
            }
            break;
        }

        default:
            break;
    }
}

/* UART7回调 (保留空实现，g_uart7_cfg仍引用) */
void uart7_callback(uart_callback_args_t *p_args)
{
    (void)p_args;
}

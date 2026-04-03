/**
 * @file    debug_uart.c
 * @brief   UART4调试输出/输入实现 (P511-RXD4, P512-TXD4)
 */

#include "debug_uart.h"
#include "hal_data.h"
#include "new_thread0.h"
#include "maixcam_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

static volatile bool g_debug_tx_complete = true;

/* 接收缓冲区 — 简单单缓冲，无队列 */
#define DEBUG_RX_BUF_SIZE 256

static char g_rx_buffer[DEBUG_RX_BUF_SIZE];
static volatile uint16_t g_rx_idx = 0;
static volatile bool g_rx_line_ready = false;

/* 统计 */
static volatile uint32_t g_debug_rx_bytes = 0;
static volatile uint32_t g_rx_uart_errors = 0;

void debug_uart_init(void)
{
    g_debug_tx_complete = true;
    g_rx_idx = 0;
    g_rx_line_ready = false;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));

    fsp_err_t err = R_SCI_UART_Open(&g_uart4_ctrl, &g_uart4_cfg);
    if (err == FSP_ERR_ALREADY_OPEN) {
        R_SCI_UART_Close(&g_uart4_ctrl);
        err = R_SCI_UART_Open(&g_uart4_ctrl, &g_uart4_cfg);
    }
    /* 开启失败：P400 LED 快闪 5 次报警 */
    if (err != FSP_SUCCESS) {
        for (int i = 0; i < 10; i++) {
            R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_00,
                              (i & 1) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
            R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);
        }
    }
}

void debug_print(const char *str)
{
    if (!str) return;
    uint32_t len = strlen(str);
    if (len == 0) return;

    /* 等待上一次TX完成 (轮询volatile标志，无信号量) */
    for (int wait = 0; wait < 200 && !g_debug_tx_complete; wait++) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!g_debug_tx_complete) {
        /* 上一次TX超时200ms未完成，强制恢复 */
        R_SCI_UART_Close(&g_uart4_ctrl);
        R_SCI_UART_Open(&g_uart4_ctrl, &g_uart4_cfg);
        g_debug_tx_complete = true;
    }

    g_debug_tx_complete = false;

    fsp_err_t err = R_SCI_UART_Write(&g_uart4_ctrl, (uint8_t *)str, len);
    if (err != FSP_SUCCESS) {
        g_debug_tx_complete = true;
        return;
    }

    /* 等待本次TX完成 */
    for (int wait = 0; wait < 200 && !g_debug_tx_complete; wait++) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    g_debug_tx_complete = true;
}

void debug_println(const char *str)
{
    if (!str || strlen(str) == 0) {
        debug_print("\r\n");
        return;
    }
    /* 合并为单次TX，避免两次debug_print导致的状态问题 */
    uint32_t slen = strlen(str);
    if (slen < 250) {
        char buf[256];
        memcpy(buf, str, slen);
        buf[slen]     = '\r';
        buf[slen + 1] = '\n';
        buf[slen + 2] = '\0';
        debug_print(buf);
    } else {
        debug_print(str);
        debug_print("\r\n");
    }
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
}

bool debug_has_line(void)
{
    return g_rx_line_ready;
}

int debug_read_line(char *buf, uint32_t buf_len)
{
    if (!g_rx_line_ready || buf == NULL || buf_len == 0) {
        return -1;
    }

    uint32_t copy_len = (g_rx_idx < buf_len - 1U) ? g_rx_idx : (buf_len - 1U);
    memcpy(buf, g_rx_buffer, copy_len);
    buf[copy_len] = '\0';

    /* 去掉末尾 \r\n */
    while (copy_len > 0 && (buf[copy_len - 1] == '\r' || buf[copy_len - 1] == '\n')) {
        buf[--copy_len] = '\0';
    }

    g_rx_idx = 0;
    g_rx_line_ready = false;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));

    return (int)copy_len;
}

uint32_t debug_uart_get_rx_bytes(void)
{
    return g_debug_rx_bytes;
}

uint16_t debug_uart_get_rx_idx(void)
{
    return g_rx_idx;
}

uint32_t debug_uart_get_errors(void)
{
    return g_rx_uart_errors;
}

void debug_uart_get_stats(debug_uart_stats_t *stats)
{
    if (stats == NULL) return;
    stats->queue_count    = g_rx_line_ready ? 1U : 0U;
    stats->dropped_lines  = 0;
    stats->overflow_chars = 0;
    stats->uart_errors    = g_rx_uart_errors;
}

/* UART4 回调 */
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
            g_debug_rx_bytes++;

            /* 存入缓冲区 */
            if (ch == '\r' || ch == '\n') {
                if (g_rx_idx > 0 && !g_rx_line_ready) {
                    g_rx_line_ready = true;
                }
            } else if (g_rx_idx < (DEBUG_RX_BUF_SIZE - 1U) && !g_rx_line_ready) {
                g_rx_buffer[g_rx_idx++] = (char)ch;
            }
            break;
        }

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_BREAK_DETECT:
            g_rx_uart_errors++;
            break;

        default:
            break;
    }
}

/* UART9 回调 (MaixCam 逐字节解析) */
void uart9_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_RX_CHAR) {
        maixcam_uart_rx_byte((uint8_t)p_args->data);
    }
}

/* UART7 回调 (保留空实现) */
void uart7_callback(uart_callback_args_t *p_args)
{
    (void)p_args;
}

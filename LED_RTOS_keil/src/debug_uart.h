/**
 * @file    debug_uart.h
 * @brief   UART4调试输出/输入接口 (P511-RXD4, P512-TXD4)
 */

#ifndef DEBUG_UART_H_
#define DEBUG_UART_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void debug_uart_init(void);
void debug_print(const char *str);
void debug_println(const char *str);
void debug_print_int(int value);
void debug_print_hex(uint32_t value);  /* 打印十六进制 */

typedef struct
{
    uint32_t queue_count;      /* 当前待处理行数 */
    uint32_t dropped_lines;    /* 队列满导致丢行次数 */
    uint32_t overflow_chars;   /* 单行过长导致溢出字符数 */
    uint32_t uart_errors;      /* UART底层错误事件计数 */
} debug_uart_stats_t;

/* 接收功能 */
bool debug_has_line(void);                          /* 是否收到完整一行（以回车结束） */
int debug_read_line(char *buf, uint32_t buf_len);   /* 读取一行，返回长度，-1表示无数据 */
void debug_uart_get_stats(debug_uart_stats_t *stats);

/* 调试统计 */
uint32_t debug_uart_get_rx_bytes(void);   /* 累计收到的RX字节数 */
uint16_t debug_uart_get_rx_idx(void);     /* 当前缓冲区写入位置 */
uint32_t debug_uart_get_errors(void);     /* UART错误事件计数 */

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART_H_ */

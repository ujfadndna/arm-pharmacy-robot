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

/* 接收功能 */
bool debug_has_line(void);                          /* 是否收到完整一行（以回车结束） */
int debug_read_line(char *buf, uint32_t buf_len);   /* 读取一行，返回长度，-1表示无数据 */

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART_H_ */

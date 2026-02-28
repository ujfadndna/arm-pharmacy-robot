/**
 * @file    log.c
 * @brief   日志系统实现 - 带级别、时间戳、模块标识
 * @note    使用FreeRTOS xTaskGetTickCount()获取时间戳
 *          输出格式: [时间戳ms][级别][模块] 消息
 */

#include "log.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 日志级别字符串 */
static const char *g_level_str[] = {
    "DEBUG",
    "INFO ",
    "WARN ",
    "ERROR"
};

/* 当前日志级别 (默认INFO) */
static log_level_t g_log_level = LOG_INFO;

/* 日志缓冲区大小 */
#define LOG_BUF_SIZE 256

void log_init(void)
{
    debug_uart_init();
    g_log_level = LOG_INFO;
}

void log_set_level(log_level_t level)
{
    if (level <= LOG_ERROR) {
        g_log_level = level;
    }
}

log_level_t log_get_level(void)
{
    return g_log_level;
}

void log_printf(log_level_t level, const char *module, const char *fmt, ...)
{
    /* 级别过滤 */
    if (level < g_log_level) {
        return;
    }

    /* 参数检查 */
    if (module == NULL || fmt == NULL) {
        return;
    }

    char buf[LOG_BUF_SIZE];
    int offset = 0;

    /* 获取时间戳 (ms) */
    uint32_t tick = (uint32_t)xTaskGetTickCount();
    uint32_t ms = tick * portTICK_PERIOD_MS;

    /* 格式化头部: [时间戳][级别][模块] */
    offset = snprintf(buf, LOG_BUF_SIZE, "[%lu][%s][%s] ",
                      (unsigned long)ms,
                      g_level_str[level],
                      module);

    /* 格式化用户消息 */
    if (offset > 0 && offset < LOG_BUF_SIZE - 2) {
        va_list args;
        va_start(args, fmt);
        int msg_len = vsnprintf(buf + offset, LOG_BUF_SIZE - offset - 2, fmt, args);
        va_end(args);

        if (msg_len > 0) {
            offset += msg_len;
        }
    }

    /* 确保不越界 */
    if (offset >= LOG_BUF_SIZE - 2) {
        offset = LOG_BUF_SIZE - 3;
    }

    /* 添加换行 */
    buf[offset++] = '\r';
    buf[offset++] = '\n';
    buf[offset] = '\0';

    /* 输出 */
    debug_print(buf);
}

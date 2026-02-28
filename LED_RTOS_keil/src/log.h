/**
 * @file    log.h
 * @brief   日志系统接口 - 带级别、时间戳、模块标识
 * @note    基于debug_uart的上层封装
 */

#ifndef LOG_H_
#define LOG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 日志级别枚举
 */
typedef enum {
    LOG_DEBUG = 0,  /**< 调试信息 */
    LOG_INFO,       /**< 一般信息 */
    LOG_WARN,       /**< 警告信息 */
    LOG_ERROR       /**< 错误信息 */
} log_level_t;

/**
 * @brief 初始化日志系统
 * @note  内部会调用debug_uart_init()
 */
void log_init(void);

/**
 * @brief 设置全局日志级别
 * @param level 最低输出级别，低于此级别的日志不输出
 */
void log_set_level(log_level_t level);

/**
 * @brief 获取当前日志级别
 * @return 当前日志级别
 */
log_level_t log_get_level(void);

/**
 * @brief 格式化日志输出
 * @param level  日志级别
 * @param module 模块名称
 * @param fmt    格式字符串
 * @param ...    可变参数
 */
void log_printf(log_level_t level, const char *module, const char *fmt, ...);

/* 便捷宏 - 带级别、模块的日志输出 */
#define LOG(level, module, fmt, ...) \
    log_printf(level, module, fmt, ##__VA_ARGS__)

#define LOG_D(module, fmt, ...) LOG(LOG_DEBUG, module, fmt, ##__VA_ARGS__)
#define LOG_I(module, fmt, ...) LOG(LOG_INFO,  module, fmt, ##__VA_ARGS__)
#define LOG_W(module, fmt, ...) LOG(LOG_WARN,  module, fmt, ##__VA_ARGS__)
#define LOG_E(module, fmt, ...) LOG(LOG_ERROR, module, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* LOG_H_ */

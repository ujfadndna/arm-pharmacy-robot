/**
 ******************************************************************************
 * @file    w800_driver.h
 * @brief   W800 WiFi模块驱动接口 - 直接UART协议版本 (v2.1)
 * @note    配合W800自定义固件 (wm_llm_demo.c UART命令接口)
 *
 * 协议说明:
 *   RA6M5 → W800:
 *     LLM:<query>\r\n       发起LLM查询
 *     WIFI:<ssid>,<pwd>\r\n 连接WiFi
 *     STATE:<状态>\r\n      设置药柜状态 (v2.1)
 *     STATUS\r\n            查询状态
 *
 *   W800 → RA6M5:
 *     OK:<response>\r\n     成功响应
 *     ERR:<code>\r\n        错误响应
 *     READY/BUSY/NO_WIFI   状态响应
 ******************************************************************************
 */

#ifndef W800_DRIVER_H_
#define W800_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 ========== */
#define W800_RX_BUFFER_SIZE     2048    /* 接收缓冲区大小 */
#define W800_AT_TIMEOUT_MS      30000   /* LLM查询超时 */
#define W800_UDP_PORT           8888    /* 保留兼容 */

/* ========== 状态定义 ========== */
typedef enum {
    W800_STATE_IDLE = 0,
    W800_STATE_WAIT_RESPONSE,
    W800_STATE_TRANSPARENT,     /* 就绪状态 (兼容旧代码) */
    W800_STATE_ERROR
} w800_state_t;

/* ========== W800状态 (新接口) ========== */
typedef enum {
    W800_STATUS_READY = 0,      /* 就绪，可以查询LLM */
    W800_STATUS_BUSY,           /* 正在处理LLM查询 */
    W800_STATUS_NO_WIFI,        /* WiFi未连接 */
    W800_STATUS_ERROR = -1      /* 通信错误 */
} w800_status_t;

/* ========== WiFi配置 (兼容旧接口) ========== */
typedef struct {
    const char *ssid;
    const char *password;
    const char *remote_ip;      /* 不再使用 */
    uint16_t   remote_port;     /* 不再使用 */
    uint16_t   local_port;      /* 不再使用 */
} w800_wifi_config_t;

/* ========== 回调函数类型 ========== */
typedef void (*w800_data_callback_t)(const uint8_t *data, uint32_t len);

/* ========== 核心接口 (v2.0) ========== */

/**
 * @brief 初始化W800驱动
 * @param on_data 数据回调函数 (可选)
 */
void w800_init(w800_data_callback_t on_data);

/**
 * @brief 查询W800状态
 * @return W800_STATUS_READY, W800_STATUS_BUSY, W800_STATUS_NO_WIFI, 或 W800_STATUS_ERROR
 */
w800_status_t w800_get_status(void);

/**
 * @brief 连接WiFi (通过W800固件)
 * @param ssid WiFi名称
 * @param password WiFi密码
 * @return 0=成功, 负数=错误
 */
int w800_connect_wifi_direct(const char *ssid, const char *password);

/**
 * @brief 调用云端LLM (通过W800固件直连)
 * @param query 用户查询 (中文直接传，无需URL编码)
 * @param response 响应缓冲区
 * @param resp_size 缓冲区大小
 * @return 响应长度, 负数=错误
 *
 * 错误码:
 *   -1: 内存分配失败
 *   -2: DNS解析失败 / 超时
 *   -3: Socket创建失败
 *   -4: SSL连接失败
 *   -5: 请求体构造失败
 *   -6: HTTP请求构造失败
 *   -7: 发送失败
 *   -8: 响应解析失败
 *   -9: 无HTTP响应体
 *   -100: WiFi未连接
 *   -101: 正在忙
 *   -102: 参数格式错误
 *   -103: 未知命令
 */
int w800_call_llm(const char *query, char *response, uint32_t resp_size);

/**
 * @brief 调用LLM (带重试机制)
 * @param query 用户查询
 * @param response 响应缓冲区
 * @param resp_size 缓冲区大小
 * @param max_retries 最大重试次数
 * @return 响应长度, 负数=错误
 */
int w800_call_llm_with_retry(const char *query, char *response,
                              uint32_t resp_size, int max_retries);

/**
 * @brief 设置药柜状态 (v2.1)
 * @param state 药柜状态字符串 (由 cabinet_serialize_state 生成)
 * @return 0=成功, 负数=错误
 *
 * 状态字符串格式:
 *   A1:阿莫西林(2026-02-15) A2:布洛芬(2027-01-01) ...
 *
 * 调用此函数后，下次 w800_call_llm() 会自动把状态加入prompt
 */
int w800_set_cabinet_state(const char *state);

/**
 * @brief 调用LLM进行智能整理 (v2.1)
 * @param state 药柜状态字符串
 * @param user_cmd 用户指令 (如 "把快过期的药放前面")
 * @param response 响应缓冲区
 * @param resp_size 缓冲区大小
 * @return 响应长度, 负数=错误
 *
 * 此函数会先发送STATE命令，再发送LLM查询
 */
int w800_call_llm_organize(const char *state, const char *user_cmd,
                           char *response, uint32_t resp_size);

/* ========== UART回调 ========== */

/**
 * @brief UART接收处理 (在UART回调中调用)
 * @param byte 接收到的字节
 */
void w800_rx_byte(uint8_t byte);

/**
 * @brief UART发送完成回调
 */
void w800_tx_complete(void);

/* ========== 兼容旧接口 (不推荐使用) ========== */

/**
 * @brief 获取当前状态 (兼容旧代码)
 */
w800_state_t w800_get_state(void);

/**
 * @brief 复位W800模块 (软件复位)
 */
void w800_reset(void);

/**
 * @brief 硬件复位W800 (通过P508引脚)
 * @note  P508连接到W800的RESET引脚，低电平有效
 */
void w800_hardware_reset(void);

/**
 * @brief 网络连通性测试
 */
void w800_test_network(void);

/**
 * @brief 连接WiFi (兼容旧接口)
 * @deprecated 请使用 w800_connect_wifi_direct()
 */
int w800_connect_wifi(const w800_wifi_config_t *config);

/**
 * @brief 仅连接WiFi (兼容旧接口)
 * @deprecated 请使用 w800_connect_wifi_direct()
 */
int w800_connect_wifi_only(const char *ssid, const char *password);

/**
 * @brief 断开WiFi (直接协议模式暂不支持)
 */
int w800_disconnect_wifi(void);

/**
 * @brief 发送UDP数据 (直接协议模式不支持)
 * @deprecated 直接协议模式不支持UDP
 */
int w800_send_udp(const uint8_t *data, uint32_t len);

/**
 * @brief 获取本地IP (直接协议模式暂不支持)
 */
int w800_get_local_ip(char *ip_buf, uint32_t buf_len);

/**
 * @brief UART发送函数 (兼容旧接口)
 * @deprecated 直接协议模式不使用此接口
 */
int W800_UART_Send(const uint8_t *data, uint32_t len);

/* ========== 调试接口 ========== */

/**
 * @brief 获取调试接收缓冲
 */
const char* w800_get_debug_rx(void);

/**
 * @brief 清空调试接收缓冲
 */
void w800_clear_debug_rx(void);

#ifdef __cplusplus
}
#endif

#endif /* W800_DRIVER_H_ */

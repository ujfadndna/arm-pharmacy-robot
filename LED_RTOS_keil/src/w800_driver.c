/**
 ******************************************************************************
 * @file    w800_driver.c
 * @brief   W800 WiFi模块驱动 - 直接UART协议版本 (v2.0)
 * @note    配合W800自定义固件 (wm_llm_demo.c UART命令接口)
 *
 * 协议说明:
 *   RA6M5 → W800:
 *     LLM:<query>\r\n       发起LLM查询
 *     WIFI:<ssid>,<pwd>\r\n 连接WiFi
 *     STATUS\r\n            查询状态
 *
 *   W800 → RA6M5:
 *     OK:<response>\r\n     成功响应
 *     ERR:<code>\r\n        错误响应
 *     READY/BUSY/NO_WIFI   状态响应
 *
 * 错误码:
 *   -1: 内存分配失败
 *  
 -2: DNS解析失败
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
 ******************************************************************************
 */

#include "w800_driver.h"
#include "hal_data.h"
#include "new_thread0.h"
#include "debug_uart.h"
#include "r_sci_uart.h"
#include "r_ioport.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

/* ========== 配置 ========== */
#define W800_UART_TIMEOUT_MS    30000   /* LLM查询超时 (30秒) */
#define W800_WIFI_TIMEOUT_MS    15000   /* WiFi连接超时 */
#define W800_STATUS_TIMEOUT_MS  1000    /* 状态查询超时 */

/* W800 RESET引脚 (P508) */
#define W800_RESET_PIN          BSP_IO_PORT_05_PIN_08

/* 命令前缀 */
#define CMD_PREFIX_LLM          "LLM:"
#define CMD_PREFIX_WIFI         "WIFI:"
#define CMD_PREFIX_STATE        "STATE:"    /* v2.1: 药柜状态命令 */
#define CMD_STATUS              "STATUS"

/* 响应前缀 */
#define RSP_OK                  "OK:"
#define RSP_ERR                 "ERR:"
#define RSP_READY               "READY"
#define RSP_BUSY                "BUSY"
#define RSP_NO_WIFI             "NO_WIFI"
#define RSP_WIFI_CONNECTING     "WIFI_CONNECTING"
#define RSP_STATE_OK            "STATE_OK"  /* v2.1: 状态设置成功 */

/* ========== 内部状态 ========== */
typedef enum {
    W800_IDLE,
    W800_WAITING_RESPONSE,
    W800_WIFI_CONNECTING
} w800_internal_state_t;

static w800_internal_state_t g_internal_state = W800_IDLE;

/* 接收缓冲 */
#define RX_BUFFER_SIZE  2048
static uint8_t g_rx_buffer[RX_BUFFER_SIZE];
static uint16_t g_rx_idx = 0;

/* 行缓冲 (用于检测\r\n结尾的完整响应) */
static char g_line_buffer[RX_BUFFER_SIZE];
static uint16_t g_line_idx = 0;
static volatile bool g_line_complete = false;

/* 发送完成标志 */
static volatile bool g_uart_tx_complete = true;

/* 回调函数 */
static w800_data_callback_t g_data_callback = NULL;

/* ========== UART发送 ========== */

static void wait_tx_complete(void)
{
    uint32_t timeout = 1000;
    while (!g_uart_tx_complete && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    }
}

static int uart_send(const char *str)
{
    uint32_t len = strlen(str);
    g_uart_tx_complete = false;
    fsp_err_t err = R_SCI_UART_Write(&g_uart6_ctrl, (uint8_t *)str, len);
    if (err == FSP_SUCCESS) {
        wait_tx_complete();
        return 0;
    }
    return -1;
}

/* ========== 接收处理 ========== */

/**
 * @brief 清空接收缓冲区
 */
static void clear_rx_buffer(void)
{
    taskENTER_CRITICAL();
    g_rx_idx = 0;
    g_line_idx = 0;
    g_line_complete = false;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_line_buffer, 0, sizeof(g_line_buffer));
    taskEXIT_CRITICAL();
}

/**
 * @brief 等待完整响应行 (以\r\n结尾)
 * @param timeout_ms 超时时间
 * @return true=收到完整行, false=超时
 */
static bool wait_response_line(uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        if (g_line_complete) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

/**
 * @brief 发送命令并等待响应
 * @param cmd 命令字符串 (不含\r\n)
 * @param response 响应缓冲区
 * @param resp_size 缓冲区大小
 * @param timeout_ms 超时时间
 * @return 响应长度, 负数=错误
 */
static int send_cmd_wait_response(const char *cmd, char *response,
                                   uint32_t resp_size, uint32_t timeout_ms)
{
    char cmd_buf[256];

    /* 清空接收缓冲 */
    clear_rx_buffer();

    /* 构造命令 (添加\r\n) */
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);

    debug_print("[W800] TX: ");
    debug_println(cmd_buf);

    /* 发送命令 */
    if (uart_send(cmd_buf) != 0) {
        debug_println("[W800] UART send failed!");
        return -1;
    }

    /* 等待响应 */
    if (!wait_response_line(timeout_ms)) {
        debug_println("[W800] Response timeout!");
        return -2;
    }

    /* 复制响应 */
    taskENTER_CRITICAL();
    uint32_t len = strlen(g_line_buffer);
    if (len >= resp_size) len = resp_size - 1;
    memcpy(response, g_line_buffer, len);
    response[len] = '\0';
    taskEXIT_CRITICAL();

    debug_print("[W800] RX: ");
    debug_println(response);

    return (int)len;
}

/* ========== 公共接口 ========== */

void w800_init(w800_data_callback_t on_data)
{
    g_data_callback = on_data;
    g_internal_state = W800_IDLE;
    g_rx_idx = 0;
    g_line_idx = 0;
    g_line_complete = false;
    g_uart_tx_complete = true;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_line_buffer, 0, sizeof(g_line_buffer));

    debug_println("[W800] Driver initialized (Direct UART Protocol v2.0)");
}

/**
 * @brief 查询W800状态
 * @return W800_STATUS_READY, W800_STATUS_BUSY, W800_STATUS_NO_WIFI, 或负数错误
 */
w800_status_t w800_get_status(void)
{
    char response[64];

    int ret = send_cmd_wait_response(CMD_STATUS, response, sizeof(response),
                                      W800_STATUS_TIMEOUT_MS);
    if (ret < 0) {
        return W800_STATUS_ERROR;
    }

    if (strstr(response, RSP_READY)) {
        return W800_STATUS_READY;
    } else if (strstr(response, RSP_BUSY)) {
        return W800_STATUS_BUSY;
    } else if (strstr(response, RSP_NO_WIFI)) {
        return W800_STATUS_NO_WIFI;
    }

    return W800_STATUS_ERROR;
}

/**
 * @brief 连接WiFi (通过W800固件)
 * @param ssid WiFi名称
 * @param password WiFi密码
 * @return 0=成功, 负数=错误
 */
int w800_connect_wifi_direct(const char *ssid, const char *password)
{
    char cmd[128];
    char response[64];

    if (!ssid || !password) {
        return -1;
    }

    debug_print("[W800] Connecting to WiFi: ");
    debug_println(ssid);

    /* 构造WiFi命令 */
    snprintf(cmd, sizeof(cmd), "%s%s,%s", CMD_PREFIX_WIFI, ssid, password);

    /* 发送命令 */
    int ret = send_cmd_wait_response(cmd, response, sizeof(response),
                                      W800_WIFI_TIMEOUT_MS);
    if (ret < 0) {
        return ret;
    }

    /* 检查响应 */
    if (strstr(response, RSP_OK) && strstr(response, RSP_WIFI_CONNECTING)) {
        debug_println("[W800] WiFi connecting...");
        /* WiFi连接是异步的，需要等待一段时间 */
        vTaskDelay(pdMS_TO_TICKS(5000));

        /* 检查状态 */
        w800_status_t status = w800_get_status();
        if (status == W800_STATUS_READY) {
            debug_println("[W800] WiFi connected!");
            return 0;
        } else {
            debug_println("[W800] WiFi connection failed!");
            return -3;
        }
    } else if (strstr(response, RSP_ERR)) {
        /* 解析错误码 */
        char *p = strstr(response, RSP_ERR);
        if (p) {
            p += strlen(RSP_ERR);
            int err_code = atoi(p);
            debug_print("[W800] WiFi error: ");
            debug_print_int(err_code);
            debug_println("");
            return -err_code;
        }
        return -2;
    }

    return -1;
}

/**
 * @brief 调用云端LLM (通过W800固件直连)
 * @param query 用户查询
 * @param response 响应缓冲区
 * @param resp_size 缓冲区大小
 * @return 响应长度, 负数=错误
 */
int w800_call_llm(const char *query, char *response, uint32_t resp_size)
{
    char cmd[256];
    static char raw_response[1024];  /* 改为static，避免栈溢出 */

    if (!query || !response) {
        return -1;
    }

    debug_println("[W800] ========== LLM Query ==========");
    debug_print("[W800] Query: ");
    debug_println(query);

    /* 构造LLM命令 */
    snprintf(cmd, sizeof(cmd), "%s%s", CMD_PREFIX_LLM, query);

    /* 发送命令并等待响应 */
    int ret = send_cmd_wait_response(cmd, raw_response, sizeof(raw_response),
                                      W800_UART_TIMEOUT_MS);
    if (ret < 0) {
        debug_print("[W800] LLM query failed: ");
        debug_print_int(ret);
        debug_println("");
        return ret;
    }

    /* 解析响应 */
    if (strncmp(raw_response, RSP_OK, strlen(RSP_OK)) == 0) {
        /* 成功响应: OK:<content> */
        char *content = raw_response + strlen(RSP_OK);
        uint32_t content_len = strlen(content);
        if (content_len >= resp_size) content_len = resp_size - 1;
        memcpy(response, content, content_len);
        response[content_len] = '\0';

        debug_println("[W800] LLM response received:");
        debug_println(response);
        debug_println("[W800] ================================");

        return (int)content_len;
    } else if (strncmp(raw_response, RSP_ERR, strlen(RSP_ERR)) == 0) {
        /* 错误响应: ERR:<code> */
        char *p = raw_response + strlen(RSP_ERR);
        int err_code = atoi(p);
        debug_print("[W800] LLM error: ");
        debug_print_int(err_code);
        debug_println("");
        return -err_code;
    }

    debug_println("[W800] Unknown response format");
    return -1;
}

/**
 * @brief 调用LLM (带重试)
 */
int w800_call_llm_with_retry(const char *query, char *response,
                              uint32_t resp_size, int max_retries)
{
    for (int i = 0; i < max_retries; i++) {
        if (i > 0) {
            debug_print("[W800] Retry ");
            debug_print_int(i);
            debug_println("...");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        int ret = w800_call_llm(query, response, resp_size);
        if (ret > 0) {
            return ret;
        }

        /* 检查是否是"正在忙"错误，等待后重试 */
        if (ret == -101) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    return -1;
}

/**
 * @brief 设置药柜状态 (v2.1)
 * @param state 药柜状态字符串 (由 cabinet_serialize_state 生成)
 * @return 0=成功, 负数=错误
 */
int w800_set_cabinet_state(const char *state)
{
    char cmd[512];
    char response[64];

    if (!state) {
        return -1;
    }

    debug_println("[W800] Setting cabinet state...");

    /* 构造STATE命令 */
    snprintf(cmd, sizeof(cmd), "%s%s", CMD_PREFIX_STATE, state);

    /* 发送命令 */
    int ret = send_cmd_wait_response(cmd, response, sizeof(response),
                                      W800_STATUS_TIMEOUT_MS);
    if (ret < 0) {
        debug_print("[W800] Set state failed: ");
        debug_print_int(ret);
        debug_println("");
        return ret;
    }

    /* 检查响应 */
    if (strstr(response, RSP_OK) && strstr(response, RSP_STATE_OK)) {
        debug_println("[W800] Cabinet state set OK");
        return 0;
    } else if (strstr(response, RSP_ERR)) {
        char *p = strstr(response, RSP_ERR);
        if (p) {
            p += strlen(RSP_ERR);
            int err_code = atoi(p);
            return -err_code;
        }
        return -2;
    }

    return -1;
}

/**
 * @brief 调用LLM进行智能整理 (v2.1)
 * @param state 药柜状态字符串
 * @param user_cmd 用户指令 (如 "把快过期的药放前面")
 * @param response 响应缓冲区
 * @param resp_size 缓冲区大小
 * @return 响应长度, 负数=错误
 */
int w800_call_llm_organize(const char *state, const char *user_cmd,
                           char *response, uint32_t resp_size)
{
    if (!state || !user_cmd || !response) {
        return -1;
    }

    debug_println("[W800] ========== Smart Organize ==========");

    /* 1. 先发送药柜状态 */
    int ret = w800_set_cabinet_state(state);
    if (ret < 0) {
        debug_println("[W800] Failed to set cabinet state");
        return ret;
    }

    /* 2. 发送用户指令进行LLM查询 */
    ret = w800_call_llm(user_cmd, response, resp_size);

    debug_println("[W800] ====================================");

    return ret;
}

/**
 * @brief 网络连通性测试
 */
void w800_test_network(void)
{
    debug_println("[W800] Network test...");

    w800_status_t status = w800_get_status();
    switch (status) {
        case W800_STATUS_READY:
            debug_println("[W800] Status: READY (WiFi connected)");
            break;
        case W800_STATUS_BUSY:
            debug_println("[W800] Status: BUSY");
            break;
        case W800_STATUS_NO_WIFI:
            debug_println("[W800] Status: NO_WIFI");
            break;
        default:
            debug_println("[W800] Status: ERROR or no response");
            break;
    }
}

/* ========== UART回调 ========== */

void w800_rx_byte(uint8_t byte)
{
    /* 存入原始缓冲 */
    if (g_rx_idx < RX_BUFFER_SIZE - 1) {
        g_rx_buffer[g_rx_idx++] = byte;
        g_rx_buffer[g_rx_idx] = '\0';
    }

    /* 行缓冲处理 */
    if (g_line_idx < RX_BUFFER_SIZE - 1) {
        g_line_buffer[g_line_idx++] = byte;
        g_line_buffer[g_line_idx] = '\0';

        /* 检测行结束 (\r\n) */
        if (g_line_idx >= 2 &&
            g_line_buffer[g_line_idx - 2] == '\r' &&
            g_line_buffer[g_line_idx - 1] == '\n') {
            /* 去除\r\n */
            g_line_buffer[g_line_idx - 2] = '\0';
            g_line_idx -= 2;
            g_line_complete = true;
        }
    }

    /* 数据回调 */
    if (g_data_callback) {
        g_data_callback(&byte, 1);
    }
}

void w800_tx_complete(void)
{
    g_uart_tx_complete = true;
}

w800_state_t w800_get_state(void)
{
    w800_status_t status = w800_get_status();
    switch (status) {
        case W800_STATUS_READY:
            return W800_STATE_TRANSPARENT;
        case W800_STATUS_NO_WIFI:
            return W800_STATE_IDLE;
        default:
            return W800_STATE_ERROR;
    }
}

void w800_reset(void)
{
    g_internal_state = W800_IDLE;
    clear_rx_buffer();
    debug_println("[W800] Reset");
}

/**
 * @brief 硬件复位W800 (通过P508引脚)
 * @note  P508连接到W800的RESET引脚，低电平有效
 */
void w800_hardware_reset(void)
{
    debug_println("[W800] Hardware reset via P508...");

    /* 配置P508为输出模式 */
    R_IOPORT_PinCfg(&g_ioport_ctrl, W800_RESET_PIN,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH);

    /* 输出低电平 (复位W800) */
    R_IOPORT_PinWrite(&g_ioport_ctrl, W800_RESET_PIN, BSP_IO_LEVEL_LOW);
    vTaskDelay(pdMS_TO_TICKS(50));  /* 保持50ms */

    /* 输出高电平 (释放复位) */
    R_IOPORT_PinWrite(&g_ioport_ctrl, W800_RESET_PIN, BSP_IO_LEVEL_HIGH);

    debug_println("[W800] Hardware reset done, waiting for boot...");

    /* 等待W800启动 */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 清空接收缓冲 */
    g_internal_state = W800_IDLE;
    clear_rx_buffer();
}

/* ========== 兼容旧接口 (保留但不推荐使用) ========== */

int w800_connect_wifi(const w800_wifi_config_t *config)
{
    if (!config || !config->ssid || !config->password) {
        return -1;
    }
    return w800_connect_wifi_direct(config->ssid, config->password);
}

int w800_connect_wifi_only(const char *ssid, const char *password)
{
    return w800_connect_wifi_direct(ssid, password);
}

int w800_disconnect_wifi(void)
{
    /* W800固件暂不支持断开WiFi命令 */
    debug_println("[W800] Disconnect not supported in direct mode");
    return 0;
}

int w800_send_udp(const uint8_t *data, uint32_t len)
{
    /* 直接协议模式不支持UDP */
    debug_println("[W800] UDP not supported in direct mode");
    return -1;
}

int w800_get_local_ip(char *ip_buf, uint32_t buf_len)
{
    /* 直接协议模式暂不支持获取IP */
    if (ip_buf && buf_len > 0) {
        strncpy(ip_buf, "N/A", buf_len);
    }
    return -1;
}

int W800_UART_Send(const uint8_t *data, uint32_t len)
{
    /* 兼容旧接口 */
    return -1;
}

/* ========== 调试接口 ========== */

static char g_debug_rx_buf[256];
static uint16_t g_debug_rx_idx = 0;

const char* w800_get_debug_rx(void)
{
    return g_debug_rx_buf;
}

void w800_clear_debug_rx(void)
{
    g_debug_rx_idx = 0;
    memset(g_debug_rx_buf, 0, sizeof(g_debug_rx_buf));
}

/* ========== UART6回调 (FSP配置) ========== */

void uart6_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_TX_COMPLETE) {
        w800_tx_complete();
    }
    else if (p_args->event == UART_EVENT_RX_CHAR) {
        uint8_t byte = (uint8_t)p_args->data;
        w800_rx_byte(byte);

        /* 调试缓冲 */
        if (g_debug_rx_idx < sizeof(g_debug_rx_buf) - 1) {
            g_debug_rx_buf[g_debug_rx_idx++] = byte;
            g_debug_rx_buf[g_debug_rx_idx] = '\0';
        }
    }
}

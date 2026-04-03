/**
 ******************************************************************************
 * @file    w800_driver.c
 * @brief   W800 WiFi模块驱动 - 直接UART协议版本 (v2.0)
 * @note    配合W800自定义固�?(wm_llm_demo.c UART命令接口)
 *
 * 协议说明:
 *   RA6M5 �?W800:
 *     LLM:<query>\r\n       发起LLM查询
 *     WIFI:<ssid>,<pwd>\r\n 连接WiFi
 *     STATUS\r\n            查询状�?
 *
 *   W800 �?RA6M5:
 *     OK:<response>\r\n     成功响应
 *     ERR:<code>\r\n        错误响应
 *     READY/BUSY/NO_WIFI   状态响�?
 *
 * 错误�?
 *   -1: 内存分配失败
 *  
 -2: DNS解析失败
 *   -3: Socket创建失败
 *   -4: SSL连接失败
 *   -5: 请求体构造失�?
 *   -6: HTTP请求构造失�?
 *   -7: 发送失�?
 *   -8: 响应解析失败
 *   -9: 无HTTP响应�?
 *   -100: WiFi未连�?
 *   -101: 正在�?
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
#include "watchdog.h"
#include "w800_cmd_handler.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>
#include "maixcam_uart.h"

/* ========== 配置 ========== */
#define W800_UART_TIMEOUT_MS    30000   /* LLM查询超时 (30�? */
#define W800_WIFI_TIMEOUT_MS    15000   /* WiFi连接超时 */
#define W800_STATUS_TIMEOUT_MS  1000    /* 状态查询超�?*/

/* W800 RESET引脚 (P508) */
#define W800_RESET_PIN          BSP_IO_PORT_05_PIN_08

/* 命令前缀 */
#define CMD_PREFIX_LLM          "LLM:"
#define CMD_PREFIX_WIFI         "WIFI:"
#define CMD_PREFIX_STATE        "STATE:"    /* v2.1: 药柜状态命�?*/
#define CMD_STATUS              "STATUS"

/* 响应前缀 */
#define RSP_OK                  "OK:"
#define RSP_ERR                 "ERR:"
#define RSP_READY               "READY"
#define RSP_BUSY                "BUSY"
#define RSP_NO_WIFI             "NO_WIFI"
#define RSP_WIFI_CONNECTING     "WIFI_CONNECTING"
#define RSP_STATE_OK            "STATE_OK"  /* v2.1: 状态设置成�?*/

/* ========== 内部状�?========== */
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

/* 行缓�?(用于检测\r\n结尾的完整响�? */
static char g_line_buffer[RX_BUFFER_SIZE];
static uint16_t g_line_idx = 0;
static volatile bool g_line_complete = false;
/* 已完成行缓冲 (�?send_cmd_wait_response 读取，与 g_line_buffer 分离) */
static char g_completed_line_buf[RX_BUFFER_SIZE];
static bool g_line_is_json_cmd = false;

/*
 * JSON命令队列:
 * OpenClaw 侧可能连续发送多条命令，单缓冲会造成后到命令被覆盖�?
 * 这里使用小型环形队列缓存整行JSON，任务上下文中依次派发�?
 */
#define JSON_CMD_MAX_LEN          512U
#define JSON_CMD_QUEUE_DEPTH      8U
static char g_json_cmd_queue[JSON_CMD_QUEUE_DEPTH][JSON_CMD_MAX_LEN];
static volatile uint8_t g_json_cmd_head = 0U;
static volatile uint8_t g_json_cmd_tail = 0U;
static volatile uint8_t g_json_cmd_count = 0U;
static volatile uint32_t g_json_cmd_drop_count = 0U;

/* 发送完成标�?*/
static volatile bool g_uart_tx_complete = true;

/* UART6 TX 互斥�?*/
static SemaphoreHandle_t g_uart_tx_mutex = NULL;

/* W800上报IP标志 (收到 "IP:..." 行时置位) */
static volatile bool g_w800_got_ip = false;

/* ISR-safe 通知：IP通知延迟到主循环打印 (避免ISR中调用debug_print) */
static volatile bool g_notify_got_ip = false;
static char g_notify_ip_str[48];

/* 回调函数 */
static w800_data_callback_t g_data_callback = NULL;

/* ========== UART发�?========== */

static void json_cmd_queue_reset(void)
{
    taskENTER_CRITICAL();
    g_json_cmd_head = 0U;
    g_json_cmd_tail = 0U;
    g_json_cmd_count = 0U;
    g_json_cmd_drop_count = 0U;
    memset(g_json_cmd_queue, 0, sizeof(g_json_cmd_queue));
    taskEXIT_CRITICAL();
}

/*
 * 在UART回调上下文中调用，只做轻量入队，不做阻塞动作�?
 */
static void json_cmd_queue_push(const char *line, uint16_t line_len)
{
    uint16_t copy_len;
    uint8_t tail;

    if ((line == NULL) || (line_len == 0U)) {
        return;
    }

    if (g_json_cmd_count >= JSON_CMD_QUEUE_DEPTH) {
        g_json_cmd_drop_count++;
        return;
    }

    copy_len = line_len;
    if (copy_len >= JSON_CMD_MAX_LEN) {
        copy_len = (uint16_t)(JSON_CMD_MAX_LEN - 1U);
    }

    tail = g_json_cmd_tail;
    memcpy(g_json_cmd_queue[tail], line, copy_len);
    g_json_cmd_queue[tail][copy_len] = '\0';

    tail++;
    if (tail >= JSON_CMD_QUEUE_DEPTH) {
        tail = 0U;
    }
    g_json_cmd_tail = tail;
    g_json_cmd_count++;
}

static bool json_cmd_queue_pop(char *line, uint32_t line_size)
{
    bool has_cmd = false;

    if ((line == NULL) || (line_size == 0U)) {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_json_cmd_count > 0U) {
        uint8_t head = g_json_cmd_head;
        uint32_t len = strlen(g_json_cmd_queue[head]);

        if (len >= line_size) {
            len = line_size - 1U;
        }

        memcpy(line, g_json_cmd_queue[head], len);
        line[len] = '\0';
        g_json_cmd_queue[head][0] = '\0';

        head++;
        if (head >= JSON_CMD_QUEUE_DEPTH) {
            head = 0U;
        }

        g_json_cmd_head = head;
        g_json_cmd_count--;
        has_cmd = true;
    }
    taskEXIT_CRITICAL();

    return has_cmd;
}

static uint32_t json_cmd_take_drop_count(void)
{
    uint32_t dropped;

    taskENTER_CRITICAL();
    dropped = g_json_cmd_drop_count;
    g_json_cmd_drop_count = 0U;
    taskEXIT_CRITICAL();

    return dropped;
}

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
    int result = -1;
    uint32_t len = strlen(str);

    if (g_uart_tx_mutex != NULL) {
        if (xSemaphoreTake(g_uart_tx_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
            return -1;  /* 超时，避免死锁 */
        }
    }

    g_uart_tx_complete = false;
    fsp_err_t err = R_SCI_UART_Write(&g_uart6_ctrl, (uint8_t *)str, len);
    if (err == FSP_SUCCESS) {
        wait_tx_complete();
        result = 0;
    }

    if (g_uart_tx_mutex != NULL) {
        xSemaphoreGive(g_uart_tx_mutex);
    }
    return result;
}

/* ========== 接收处理 ========== */

/**
 * @brief 清空接收缓冲�?
 */
static void clear_rx_buffer(void)
{
    taskENTER_CRITICAL();
    g_rx_idx = 0;
    g_line_idx = 0;
    g_line_complete = false;
    g_line_is_json_cmd = false;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_line_buffer, 0, sizeof(g_line_buffer));
    memset(g_completed_line_buf, 0, sizeof(g_completed_line_buf));
    taskEXIT_CRITICAL();
}

/**
 * @brief 判断是否为协议层有效响应�?
 */
static bool is_expected_response_line(const char *line)
{
    if (line == NULL) {
        return false;
    }

    if (strncmp(line, RSP_OK, strlen(RSP_OK)) == 0) {
        return true;
    }
    if (strncmp(line, RSP_ERR, strlen(RSP_ERR)) == 0) {
        return true;
    }
    if (strcmp(line, RSP_READY) == 0) {
        return true;
    }
    if (strcmp(line, RSP_BUSY) == 0) {
        return true;
    }
    if (strcmp(line, RSP_NO_WIFI) == 0) {
        return true;
    }

    return false;
}

/**
 * @brief 取出一条已完成响应行（原子消费�?
 * @param line 输出缓冲�?
 * @param line_size 缓冲区大�?
 * @return true=取到一�? false=当前无完整行
 */
static bool take_completed_line(char *line, uint32_t line_size)
{
    bool has_line = false;

    if ((line == NULL) || (line_size == 0U)) {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_line_complete) {
        uint32_t len = strlen(g_completed_line_buf);
        if (len >= line_size) {
            len = line_size - 1U;
        }
        memcpy(line, g_completed_line_buf, len);
        line[len] = '\0';

        g_line_complete = false;
        g_completed_line_buf[0] = '\0';
        has_line = true;
    }
    taskEXIT_CRITICAL();

    return has_line;
}

/**
 * @brief 发送命令并等待响应
 * @param cmd 命令字符�?(不含\r\n)
 * @param response 响应缓冲�?
 * @param resp_size 缓冲区大�?
 * @param timeout_ms 超时时间
 * @return 响应长度, 负数=错误
 */
static int send_cmd_wait_response(const char *cmd, char *response,
                                   uint32_t resp_size, uint32_t timeout_ms)
{
    char cmd_buf[256];
    char line[RX_BUFFER_SIZE];
    TickType_t start_tick;
    uint32_t refresh_cnt = 0;

    /* 清空接收缓冲 */
    clear_rx_buffer();

    /* 构造命�?(添加\r\n) */
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);

    debug_print("[W800] TX: ");
    debug_println(cmd_buf);

    /* 发送命�?*/
    if (uart_send(cmd_buf) != 0) {
        debug_println("[W800] UART send failed!");
        return -1;
    }

    /* 等待并筛选有效响应行 */
    start_tick = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) {
        if (take_completed_line(line, sizeof(line))) {
            if (is_expected_response_line(line)) {
                uint32_t len = strlen(line);
                if (len >= resp_size) {
                    len = resp_size - 1U;
                }
                memcpy(response, line, len);
                response[len] = '\0';

                debug_print("[W800] RX: ");
                debug_println(response);

                return (int)len;
            }

            /* 丢弃 DBG/噪声等非协议行，继续等待有效应答 */
            debug_print("[W800] Skip non-protocol line: ");
            debug_println(line);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        if (++refresh_cnt >= 200U) {
            watchdog_refresh();
            refresh_cnt = 0U;
        }
    }

    debug_println("[W800] Response timeout!");
    return -2;
}

/* ========== 公共接口 ========== */

void w800_init(w800_data_callback_t on_data)
{
    g_data_callback = on_data;
    g_internal_state = W800_IDLE;
    g_rx_idx = 0;
    g_line_idx = 0;
    g_line_complete = false;
    g_line_is_json_cmd = false;
    g_uart_tx_complete = true;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_line_buffer, 0, sizeof(g_line_buffer));
    memset(g_completed_line_buf, 0, sizeof(g_completed_line_buf));
    json_cmd_queue_reset();

    if (g_uart_tx_mutex == NULL) {
        g_uart_tx_mutex = xSemaphoreCreateMutex();
    }

    debug_println("[W800] Driver initialized (Direct UART Protocol v2.0)");
}

/**
 * @brief 查询W800状�?
 * @return W800_STATUS_READY, W800_STATUS_BUSY, W800_STATUS_NO_WIFI, 或负数错�?
 */
/* 内部工具：支持自定义超时的 STATUS 查询 */
static w800_status_t w800_get_status_with_timeout(uint32_t timeout_ms)
{
    char response[64];

    int ret = send_cmd_wait_response(CMD_STATUS, response, sizeof(response),
                                     timeout_ms);
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

w800_status_t w800_get_status(void)
{
    return w800_get_status_with_timeout(W800_STATUS_TIMEOUT_MS);
}

/**
 * @brief 连接WiFi (通过W800固件，非阻塞)
 * @param ssid WiFi名称
 * @param password WiFi密码
 * @return 0=命令已接受并开始连接, 负数=错误
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

    /* 发送命令并等待首个响应，仅用于确认命令是否被接受 */
    int ret = send_cmd_wait_response(cmd, response, sizeof(response),
                                     W800_WIFI_TIMEOUT_MS);
    if (ret < 0) {
        return ret;
    }

    /* 成功响应: OK:WIFI_CONNECTING */
    if (strstr(response, RSP_OK) && strstr(response, RSP_WIFI_CONNECTING)) {
        debug_println("[W800] WiFi connecting (non-blocking)...");

        /* 清掉旧的 IP 标志，等待新一轮连接完成后由固件上报 */
        g_w800_got_ip = false;
        return 0;
    }

    /* 错误响应: ERR:<code> */
    if (strstr(response, RSP_ERR)) {
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

    /* 未知响应格式 */
    debug_println("[W800] Unknown WiFi connect response");
    return -1;
}

/**
 * @brief 调用云端LLM (通过W800固件直连)
 * @param query 用户查询
 * @param response 响应缓冲�?
 * @param resp_size 缓冲区大�?
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
 * @brief 调用LLM (带重�?
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

        /* 检查是否是"正在�?错误，等待后重试 */
        if (ret == -101) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    return -1;
}

/**
 * @brief 设置药柜状�?(v2.1)
 * @param state 药柜状态字符串 (�?cabinet_serialize_state 生成)
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

    /* 发送命�?*/
    int ret = send_cmd_wait_response(cmd, response, sizeof(response),
                                      W800_STATUS_TIMEOUT_MS);
    if (ret < 0) {
        debug_print("[W800] Set state failed: ");
        debug_print_int(ret);
        debug_println("");
        return ret;
    }

    /* 检查响�?*/
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
 * @param user_cmd 用户指令 (�?"把快过期的药放前�?)
 * @param response 响应缓冲�?
 * @param resp_size 缓冲区大�?
 * @return 响应长度, 负数=错误
 */
int w800_call_llm_organize(const char *state, const char *user_cmd,
                           char *response, uint32_t resp_size)
{
    if (!state || !user_cmd || !response) {
        return -1;
    }

    debug_println("[W800] ========== Smart Organize ==========");

    /* 1. 先发送药柜状�?*/
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
 * @brief 网络连通性测�?
 */
void w800_test_network(void)
{
    debug_println("[W800] Network test...");

    /* 使用较短超时，避免在主循环中长时间阻塞 */
    w800_status_t status = w800_get_status_with_timeout(200);
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
    bool line_is_json = g_line_is_json_cmd;
    bool line_done = false;

    /* 存入原始缓冲 */
    if (g_rx_idx < RX_BUFFER_SIZE - 1) {
        g_rx_buffer[g_rx_idx++] = byte;
        g_rx_buffer[g_rx_idx] = '\0';
    }

    /* 行缓冲处�?*/
    if (g_line_idx < RX_BUFFER_SIZE - 1) {
        if (g_line_idx == 0) {
            g_line_is_json_cmd = (byte == '{');
        }
        line_is_json = g_line_is_json_cmd;

        g_line_buffer[g_line_idx++] = byte;
        g_line_buffer[g_line_idx] = '\0';

        /* 检测行结束：兼�?CRLF �?LF */
        if (byte == '\n') {
            if (g_line_idx > 0) {
                /* 去除末尾 '\n' */
                g_line_idx--;
                g_line_buffer[g_line_idx] = '\0';
                /* 若前一字节�?'\r'，一并去�?*/
                if ((g_line_idx > 0) && (g_line_buffer[g_line_idx - 1] == '\r')) {
                    g_line_idx--;
                    g_line_buffer[g_line_idx] = '\0';
                }
            }
            line_done = true;
        }

        if (line_done) {
            /* 空行直接忽略 */
            if (g_line_idx == 0U) {
                g_line_buffer[0] = '\0';
                g_line_is_json_cmd = false;
                return;
            }

            /* JSON 命令分流:
             * 行首�?'{' 时按 TCP 透传命令处理�?
             * 其他行继续走原有 W800 响应流程 (OK/ERR/READY...)�?*/

            /* IP: 前缀 —�?W800 上报自身 IP，仅打印不进响应队列 */
            if (strncmp(g_line_buffer, "IP:", 3) == 0) {
                /* ISR-safe: 不在ISR中调用debug_print，改为设置标志由主循环打印 */
                strncpy(g_notify_ip_str, g_line_buffer + 3, sizeof(g_notify_ip_str) - 1);
                g_notify_ip_str[sizeof(g_notify_ip_str) - 1] = '\0';
                g_notify_got_ip = true;
                g_w800_got_ip = true;   /* WiFi已连接，IP已获�?*/
                g_line_idx = 0;
                g_line_buffer[0] = '\0';
                g_line_is_json_cmd = false;
                return;
            }

            /* MCDATA: 前缀 —�?W800 �?UDP 收到�?MC �?hex 编码后转发给 RA6M5 */
            if (strncmp(g_line_buffer, "MCDATA:", 7) == 0) {
                const char *hex = g_line_buffer + 7;
                while ((hex[0] != '\0') && (hex[1] != '\0')) {
                    int hi = (hex[0] >= 'A') ? (hex[0] - 'A' + 10) : (hex[0] - '0');
                    int lo = (hex[1] >= 'A') ? (hex[1] - 'A' + 10) : (hex[1] - '0');
                    uint8_t b = (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
                    maixcam_uart_rx_byte(b);
                    hex += 2;
                }
                g_line_idx = 0;
                g_line_buffer[0] = '\0';
                g_line_is_json_cmd = false;
                return;
            }

            if ((g_line_idx > 0) && (g_line_buffer[0] == '{')) {
                json_cmd_queue_push(g_line_buffer, g_line_idx);
                g_line_idx = 0;
                g_line_buffer[0] = '\0';
            } else {
                /* 将完成的行复制到专用缓冲，再重置行缓�?*/
                memcpy(g_completed_line_buf, g_line_buffer, g_line_idx + 1);
                g_completed_line_buf[g_line_idx] = '\0';
                g_line_complete = true;
                /* 重置行缓冲，使下一�?g_line_idx == 0，JSON检测正常工�?*/
                g_line_idx = 0;
                g_line_buffer[0] = '\0';
            }

            g_line_is_json_cmd = false;
        }
    }

    /* 数据回调 */
    if (g_data_callback && !line_is_json) {
        g_data_callback(&byte, 1);
    }
}

void w800_process_pending_cmd(void)
{
    char cmd_line[JSON_CMD_MAX_LEN];
    uint32_t dropped = json_cmd_take_drop_count();

    if (dropped > 0U) {
        debug_print("[W800] JSON cmd dropped: ");
        debug_print_int((int)dropped);
        debug_println("");
    }

    /* 每轮主循环只处理1条JSON命令，防止多条命令累积阻塞debug_has_line() */
    if (json_cmd_queue_pop(cmd_line, sizeof(cmd_line))) {
        watchdog_refresh();
        debug_print("[W800] JSON cmd dispatch: ");
        debug_println(cmd_line);
        w800_cmd_handle(cmd_line);
    }
}

void w800_poll_notifications(void)
{
    if (g_notify_got_ip) {
        g_notify_got_ip = false;
        debug_print("[W800] W800 IP: ");
        debug_println(g_notify_ip_str);
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
    json_cmd_queue_reset();
    debug_println("[W800] Reset");
}

/**
 * @brief 硬件复位W800 (通过P508引脚)
 * @note  P508连接到W800的RESET引脚，低电平有效
 */
void w800_hardware_reset(void)
{
    debug_println("[W800] Hardware reset via P508...");

    /* 配置P508为输出模�?*/
    R_IOPORT_PinCfg(&g_ioport_ctrl, W800_RESET_PIN,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH);

    /* 输出低电�?(复位W800) */
    R_IOPORT_PinWrite(&g_ioport_ctrl, W800_RESET_PIN, BSP_IO_LEVEL_LOW);
    vTaskDelay(pdMS_TO_TICKS(50));  /* 保持50ms */

    /* 输出高电�?(释放复位) */
    R_IOPORT_PinWrite(&g_ioport_ctrl, W800_RESET_PIN, BSP_IO_LEVEL_HIGH);

    debug_println("[W800] Hardware reset done, waiting for boot...");

    /* 等待W800启动 */
    watchdog_refresh();
    vTaskDelay(pdMS_TO_TICKS(1000));
    watchdog_refresh();
    vTaskDelay(pdMS_TO_TICKS(1000));
    watchdog_refresh();

    /* 清空接收缓冲 */
    g_internal_state = W800_IDLE;
    g_w800_got_ip = false;
    clear_rx_buffer();
    json_cmd_queue_reset();
}

/* ========== 兼容旧接�?(保留但不推荐使用) ========== */

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
    if ((data == NULL) || (len == 0U)) {
        return -1;
    }

    if (g_uart_tx_mutex != NULL) {
        if (xSemaphoreTake(g_uart_tx_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
            return -1;  /* 超时，避免死锁 */
        }
    }

    g_uart_tx_complete = false;
    int result = -1;
    if (R_SCI_UART_Write(&g_uart6_ctrl, (uint8_t *)data, len) == FSP_SUCCESS) {
        wait_tx_complete();
        result = (int)len;
    }

    if (g_uart_tx_mutex != NULL) {
        xSemaphoreGive(g_uart_tx_mutex);
    }
    return result;
}

/* ========== MaixCam2 WiFi 透传接口 ========== */

int w800_mc_send(const uint8_t *data, uint32_t len)
{
    /* UDP:<HEX>\r\n  最�?128字节 �?256个hex字符 + "UDP:\r\n" = 262 */
    char cmd_buf[262];
    uint32_t i;
    uint32_t n;

    if ((data == NULL) || (len == 0U) || (len > 128U)) {
        return -1;
    }

    n = (uint32_t)snprintf(cmd_buf, sizeof(cmd_buf), "UDP:");
    for (i = 0U; i < len; i++) {
        cmd_buf[n++] = "0123456789ABCDEF"[(data[i] >> 4) & 0xFU];
        cmd_buf[n++] = "0123456789ABCDEF"[data[i] & 0xFU];
    }
    cmd_buf[n++] = '\r';
    cmd_buf[n++] = '\n';
    cmd_buf[n]   = '\0';

    return uart_send(cmd_buf);
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


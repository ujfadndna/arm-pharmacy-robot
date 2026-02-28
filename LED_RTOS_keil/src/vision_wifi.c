/**
 ******************************************************************************
 * @file    vision_wifi.c
 * @brief   MaixCam2 WiFi UDP通信驱动实现
 ******************************************************************************
 */

#include "vision_wifi.h"
#include <string.h>
#include <stdio.h>

/* ========== 帧解析状态机 ========== */
typedef enum {
    PARSE_STATE_HEADER_0 = 0,
    PARSE_STATE_HEADER_1,
    PARSE_STATE_CMD,
    PARSE_STATE_LEN_L,
    PARSE_STATE_LEN_H,
    PARSE_STATE_DATA,
    PARSE_STATE_CHK
} parse_state_t;

/* ========== 内部状态 ========== */
static vision_wifi_config_t g_config = {0};
static vision_state_t g_state = VISION_STATE_DISCONNECTED;

/* 帧解析 */
static parse_state_t g_parse_state = PARSE_STATE_HEADER_0;
static uint8_t  g_frame_cmd = 0;
static uint16_t g_frame_len = 0;
static uint16_t g_frame_idx = 0;
static uint8_t  g_frame_data[VISION_FRAME_MAX_DATA];
static uint8_t  g_frame_chk = 0;

/* 最新检测结果 */
static vision_detect_result_t g_detect_result = {0};
static bool g_detect_new = false;

/* 心跳计时 */
static uint32_t g_last_heartbeat = 0;
static uint32_t g_tick_count = 0;

/* ========== 底层发送接口 ========== */
/* W800_UART_Send 在 w800_driver.c 中实现，声明在 w800_driver.h */
extern int W800_UART_Send(const uint8_t *data, uint32_t len);

/* ========== 内部函数 ========== */

static uint8_t calc_checksum(uint8_t cmd, uint16_t len, const uint8_t *data)
{
    uint8_t chk = cmd;
    chk ^= (uint8_t)(len & 0xFF);
    chk ^= (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}

static void parse_detect_result(const uint8_t *data, uint16_t len)
{
    if (len < 1) return;

    uint8_t count = data[0];
    if (count > VISION_MAX_OBJECTS) {
        count = VISION_MAX_OBJECTS;
    }

    uint16_t expected_len = (uint16_t)(1 + count * VISION_OBJ_SIZE);
    if (len < expected_len) return;

    g_detect_result.count = count;
    const uint8_t *ptr = &data[1];

    for (uint8_t i = 0; i < count; i++) {
        vision_object_t *obj = &g_detect_result.objects[i];
        /* Little-endian解析 */
        obj->x = (int16_t)(ptr[0] | (ptr[1] << 8));
        obj->y = (int16_t)(ptr[2] | (ptr[3] << 8));
        obj->w = (uint16_t)(ptr[4] | (ptr[5] << 8));
        obj->h = (uint16_t)(ptr[6] | (ptr[7] << 8));
        obj->class_id = (uint16_t)(ptr[8] | (ptr[9] << 8));
        /* float解析 (假设IEEE 754 little-endian) */
        uint32_t score_raw = (uint32_t)ptr[10] | ((uint32_t)ptr[11] << 8) |
                             ((uint32_t)ptr[12] << 16) | ((uint32_t)ptr[13] << 24);
        memcpy(&obj->score, &score_raw, sizeof(float));
        ptr += VISION_OBJ_SIZE;
    }

    g_detect_new = true;

    /* 回调通知 */
    if (g_config.on_detect) {
        g_config.on_detect(&g_detect_result);
    }
}

static void parse_qr_result(const uint8_t *data, uint16_t len)
{
    if (len < 1) return;

    vision_qr_result_t qr = {0};
    qr.len = data[0];
    if (qr.len > VISION_QR_MAX_LEN) {
        qr.len = VISION_QR_MAX_LEN;
    }
    if (len < 1 + qr.len) return;

    memcpy(qr.data, &data[1], qr.len);

    if (g_config.on_qr) {
        g_config.on_qr(&qr);
    }
}

static void process_frame(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    g_last_heartbeat = g_tick_count;

    switch (cmd) {
        case VISION_CMD_HEARTBEAT:
            /* 心跳响应，更新连接状态 */
            if (g_state != VISION_STATE_CONNECTED) {
                g_state = VISION_STATE_CONNECTED;
                if (g_config.on_state) {
                    g_config.on_state(g_state);
                }
            }
            break;

        case VISION_CMD_DETECT_RESULT:
            parse_detect_result(data, len);
            break;

        case VISION_CMD_QR_RESULT:
            parse_qr_result(data, len);
            break;

        case VISION_CMD_TASK_ACK:
            /* 任务确认 */
            break;

        default:
            break;
    }
}

/* ========== 公共接口实现 ========== */

int vision_wifi_init(const vision_wifi_config_t *config)
{
    if (!config) return -1;

    memcpy(&g_config, config, sizeof(vision_wifi_config_t));
    g_state = VISION_STATE_DISCONNECTED;
    g_parse_state = PARSE_STATE_HEADER_0;
    g_detect_new = false;
    g_tick_count = 0;
    g_last_heartbeat = 0;

    return 0;
}

int vision_wifi_connect(void)
{
    /*
     * TODO: 发送W800 AT指令连接WiFi
     * AT+CWJAP="ssid","password"
     * AT+CIPSTART="UDP","remote_ip",port
     */
    g_state = VISION_STATE_CONNECTING;
    if (g_config.on_state) {
        g_config.on_state(g_state);
    }
    return 0;
}

void vision_wifi_rx_handler(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (g_parse_state) {
            case PARSE_STATE_HEADER_0:
                if (byte == VISION_FRAME_HEADER_0) {
                    g_parse_state = PARSE_STATE_HEADER_1;
                }
                break;

            case PARSE_STATE_HEADER_1:
                if (byte == VISION_FRAME_HEADER_1) {
                    g_parse_state = PARSE_STATE_CMD;
                } else {
                    g_parse_state = PARSE_STATE_HEADER_0;
                }
                break;

            case PARSE_STATE_CMD:
                g_frame_cmd = byte;
                g_frame_chk = byte;
                g_parse_state = PARSE_STATE_LEN_L;
                break;

            case PARSE_STATE_LEN_L:
                g_frame_len = byte;
                g_frame_chk ^= byte;
                g_parse_state = PARSE_STATE_LEN_H;
                break;

            case PARSE_STATE_LEN_H:
                g_frame_len = (uint16_t)(g_frame_len | ((uint16_t)byte << 8));
                g_frame_chk ^= byte;
                g_frame_idx = 0;
                if (g_frame_len > 0 && g_frame_len <= VISION_FRAME_MAX_DATA) {
                    g_parse_state = PARSE_STATE_DATA;
                } else if (g_frame_len == 0) {
                    g_parse_state = PARSE_STATE_CHK;
                } else {
                    g_parse_state = PARSE_STATE_HEADER_0;
                }
                break;

            case PARSE_STATE_DATA:
                g_frame_data[g_frame_idx++] = byte;
                g_frame_chk ^= byte;
                if (g_frame_idx >= g_frame_len) {
                    g_parse_state = PARSE_STATE_CHK;
                }
                break;

            case PARSE_STATE_CHK:
                if (byte == g_frame_chk) {
                    process_frame(g_frame_cmd, g_frame_data, g_frame_len);
                }
                g_parse_state = PARSE_STATE_HEADER_0;
                break;

            default:
                g_parse_state = PARSE_STATE_HEADER_0;
                break;
        }
    }
}

static int send_frame(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t frame[VISION_FRAME_MAX_DATA + VISION_FRAME_MIN_SIZE];
    uint16_t idx = 0;

    frame[idx++] = VISION_FRAME_HEADER_0;
    frame[idx++] = VISION_FRAME_HEADER_1;
    frame[idx++] = cmd;
    frame[idx++] = (uint8_t)(len & 0xFF);
    frame[idx++] = (uint8_t)(len >> 8);

    if (data && len > 0) {
        memcpy(&frame[idx], data, len);
        idx += len;
    }

    frame[idx++] = calc_checksum(cmd, len, data);

    return W800_UART_Send(frame, idx);
}

int vision_wifi_send_task(const vision_task_request_t *task)
{
    if (!task) return -1;
    return send_frame(VISION_CMD_TASK_REQUEST,
                      (const uint8_t *)task, sizeof(vision_task_request_t));
}

void vision_wifi_send_heartbeat(void)
{
    send_frame(VISION_CMD_HEARTBEAT, NULL, 0);
}

vision_state_t vision_wifi_get_state(void)
{
    return g_state;
}

bool vision_wifi_get_detect_result(vision_detect_result_t *result)
{
    if (!g_detect_new || !result) return false;

    memcpy(result, &g_detect_result, sizeof(vision_detect_result_t));
    g_detect_new = false;
    return true;
}

void vision_wifi_tick(void)
{
    g_tick_count++;

    /* 检查超时 */
    if (g_state == VISION_STATE_CONNECTED) {
        if (g_tick_count - g_last_heartbeat > VISION_TIMEOUT_MS) {
            g_state = VISION_STATE_DISCONNECTED;
            if (g_config.on_state) {
                g_config.on_state(g_state);
            }
        }
    }
}

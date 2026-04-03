#include "maixcam_uart.h"
#include "r_sci_uart.h"
#include <string.h>

/* UART9 实例由 FSP 生成代码定义 (P601=RXD, P602=TXD) */
extern sci_uart_instance_ctrl_t g_uart9_ctrl;

/* ========== 帧解析状态机 ========== */
typedef enum {
    MC_PARSE_HEADER_0 = 0,
    MC_PARSE_HEADER_1,
    MC_PARSE_CMD,
    MC_PARSE_LEN_L,
    MC_PARSE_LEN_H,
    MC_PARSE_DATA,
    MC_PARSE_CHK
} mc_parse_state_t;

/* ========== 内部状态 ========== */
static maixcam_uart_callbacks_t g_mc_callbacks = {0};

static mc_parse_state_t mc_state = MC_PARSE_HEADER_0;
static uint8_t  mc_frame_cmd = 0;
static uint16_t mc_frame_len = 0;
static uint16_t mc_frame_idx = 0;
static uint8_t  mc_frame_data[VISION_FRAME_MAX_DATA];
static uint8_t  mc_frame_chk = 0;

/* ========== 内部函数 ========== */
static uint8_t mc_calc_checksum(uint8_t cmd, uint16_t len, const uint8_t *data)
{
    uint8_t chk = cmd;
    chk ^= (uint8_t)(len & 0xFF);
    chk ^= (uint8_t)(len >> 8);

    for (uint16_t i = 0; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}

static int mc_send_frame(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    if (len > VISION_FRAME_MAX_DATA) return -1;
    if ((len > 0) && (data == NULL)) return -1;

    uint8_t frame[VISION_FRAME_MAX_DATA + VISION_FRAME_MIN_SIZE];
    uint16_t idx = 0;

    frame[idx++] = VISION_FRAME_HEADER_0;
    frame[idx++] = VISION_FRAME_HEADER_1;
    frame[idx++] = cmd;
    frame[idx++] = (uint8_t)(len & 0xFF);
    frame[idx++] = (uint8_t)(len >> 8);

    if ((data != NULL) && (len > 0)) {
        memcpy(&frame[idx], data, len);
        idx += len;
    }

    frame[idx++] = mc_calc_checksum(cmd, len, data);

    if (R_SCI_UART_Write(&g_uart9_ctrl, frame, idx) != FSP_SUCCESS) {
        return -2;
    }
    return 0;
}

static void mc_process_frame(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    switch (cmd) {
        case MC_CMD_HEARTBEAT:
            break;

        case MC_CMD_TAG_SINGLE:
        case MC_CMD_SCAN_TAG:
            if ((len >= sizeof(mc_tag_result_t)) && g_mc_callbacks.on_tag) {
                mc_tag_result_t r;
                memcpy(&r, data, sizeof(r));
                g_mc_callbacks.on_tag(&r);
            }
            break;

        case MC_CMD_SCAN_START:
            break;

        case MC_CMD_SCAN_END:
            break;

        case MC_CMD_ALIGN_OK:
            if ((len >= sizeof(mc_align_ok_t)) && g_mc_callbacks.on_align_ok) {
                mc_align_ok_t r;
                memcpy(&r, data, sizeof(r));
                g_mc_callbacks.on_align_ok(&r);
            }
            break;

        case MC_CMD_ALIGN_FAIL:
            if ((len >= sizeof(mc_align_fail_t)) && g_mc_callbacks.on_align_fail) {
                mc_align_fail_t r;
                memcpy(&r, data, sizeof(r));
                g_mc_callbacks.on_align_fail(&r);
            }
            break;

        case MC_CMD_QR_RESULT:
            if ((len >= 1U) && g_mc_callbacks.on_qr) {
                vision_qr_result_t qr = {0};
                qr.len = data[0];
                if (qr.len > VISION_QR_MAX_LEN) {
                    qr.len = VISION_QR_MAX_LEN;
                }
                if (len >= (uint16_t)(1U + qr.len)) {
                    memcpy(qr.data, &data[1], qr.len);
                    g_mc_callbacks.on_qr(&qr);
                }
            }
            break;

        case MC_CMD_YOLO_RESULT:
            if ((len >= 1U) && g_mc_callbacks.on_yolo) {
                vision_detect_result_t result = {0};
                uint8_t count = data[0];
                if (count > VISION_MAX_OBJECTS) {
                    count = VISION_MAX_OBJECTS;
                }

                uint16_t expected_len = (uint16_t)(1U + count * VISION_OBJ_SIZE);
                if (len >= expected_len) {
                    result.count = count;

                    const uint8_t *ptr = &data[1];
                    for (uint8_t i = 0; i < count; i++) {
                        vision_object_t *obj = &result.objects[i];
                        obj->x = (int16_t)(ptr[0] | ((uint16_t)ptr[1] << 8));
                        obj->y = (int16_t)(ptr[2] | ((uint16_t)ptr[3] << 8));
                        obj->w = (uint16_t)(ptr[4] | ((uint16_t)ptr[5] << 8));
                        obj->h = (uint16_t)(ptr[6] | ((uint16_t)ptr[7] << 8));
                        obj->class_id = (uint16_t)(ptr[8] | ((uint16_t)ptr[9] << 8));

                        uint32_t score_raw = (uint32_t)ptr[10] |
                                             ((uint32_t)ptr[11] << 8) |
                                             ((uint32_t)ptr[12] << 16) |
                                             ((uint32_t)ptr[13] << 24);
                        memcpy(&obj->score, &score_raw, sizeof(float));
                        ptr += VISION_OBJ_SIZE;
                    }

                    g_mc_callbacks.on_yolo(&result);
                }
            }
            break;

        case MC_CMD_VOICE_ACTION:
            if ((len >= sizeof(mc_voice_action_t)) && g_mc_callbacks.on_voice_action) {
                mc_voice_action_t r;
                memcpy(&r, data, sizeof(r));
                g_mc_callbacks.on_voice_action(&r);
            }
            break;

        case MC_CMD_ACK:
            /* ACK 暂不处理 */
            break;

        default:
            break;
    }
}

/* ========== 公共接口实现 ========== */
int maixcam_uart_init(const maixcam_uart_callbacks_t *cbs)
{
    if (cbs) {
        memcpy(&g_mc_callbacks, cbs, sizeof(g_mc_callbacks));
    } else {
        memset(&g_mc_callbacks, 0, sizeof(g_mc_callbacks));
    }

    mc_state = MC_PARSE_HEADER_0;
    mc_frame_cmd = 0;
    mc_frame_len = 0;
    mc_frame_idx = 0;
    mc_frame_chk = 0;

    return 0;
}

void maixcam_uart_rx_byte(uint8_t byte)
{
    switch (mc_state) {
        case MC_PARSE_HEADER_0:
            if (byte == VISION_FRAME_HEADER_0) {
                mc_state = MC_PARSE_HEADER_1;
            }
            break;

        case MC_PARSE_HEADER_1:
            if (byte == VISION_FRAME_HEADER_1) {
                mc_state = MC_PARSE_CMD;
            } else {
                mc_state = MC_PARSE_HEADER_0;
            }
            break;

        case MC_PARSE_CMD:
            mc_frame_cmd = byte;
            mc_frame_chk = byte;
            mc_state = MC_PARSE_LEN_L;
            break;

        case MC_PARSE_LEN_L:
            mc_frame_len = byte;
            mc_frame_chk ^= byte;
            mc_state = MC_PARSE_LEN_H;
            break;

        case MC_PARSE_LEN_H:
            mc_frame_len = (uint16_t)(mc_frame_len | ((uint16_t)byte << 8));
            mc_frame_chk ^= byte;
            mc_frame_idx = 0;
            if ((mc_frame_len > 0U) && (mc_frame_len <= VISION_FRAME_MAX_DATA)) {
                mc_state = MC_PARSE_DATA;
            } else if (mc_frame_len == 0U) {
                mc_state = MC_PARSE_CHK;
            } else {
                mc_state = MC_PARSE_HEADER_0;
            }
            break;

        case MC_PARSE_DATA:
            mc_frame_data[mc_frame_idx++] = byte;
            mc_frame_chk ^= byte;
            if (mc_frame_idx >= mc_frame_len) {
                mc_state = MC_PARSE_CHK;
            }
            break;

        case MC_PARSE_CHK:
            if (byte == mc_frame_chk) {
                mc_process_frame(mc_frame_cmd, mc_frame_data, mc_frame_len);
            }
            mc_state = MC_PARSE_HEADER_0;
            break;

        default:
            mc_state = MC_PARSE_HEADER_0;
            break;
    }
}

int maixcam_uart_send_req_scan(uint8_t scan_type, uint8_t frames)
{
    mc_req_scan_t req = {
        .scan_type = scan_type,
        .frames_req = frames,
    };
    return mc_send_frame(MC_CMD_REQ_SCAN, (const uint8_t *)&req, 2);
}

int maixcam_uart_send_req_align(uint8_t target_id, uint16_t timeout_ms, uint8_t stable)
{
    mc_req_align_t req = {
        .target_id = target_id,
        .timeout_ms = timeout_ms,
        .stable_req = stable,
    };
    return mc_send_frame(MC_CMD_REQ_ALIGN, (const uint8_t *)&req, 4);
}

int maixcam_uart_send_req_qr(uint16_t timeout_ms)
{
    mc_req_qr_t req = {
        .timeout_ms = timeout_ms,
    };
    return mc_send_frame(MC_CMD_REQ_QR, (const uint8_t *)&req, 2);
}

int maixcam_uart_send_req_yolo(uint8_t class_filter, uint8_t conf_thresh, uint16_t timeout_ms)
{
    mc_req_yolo_t req = {
        .class_filter = class_filter,
        .conf_thresh = conf_thresh,
        .timeout_ms = timeout_ms,
    };
    return mc_send_frame(MC_CMD_REQ_YOLO, (const uint8_t *)&req, 4);
}

int maixcam_uart_send_speak(const char *text, uint8_t len)
{
    uint8_t payload[64];
    uint8_t text_len = len;

    if ((text == NULL) && (len > 0U)) {
        return -1;
    }

    if (text_len > 63U) {
        text_len = 63U;
    }

    payload[0] = text_len;
    if (text_len > 0U) {
        memcpy(&payload[1], text, text_len);
    }

    return mc_send_frame(MC_CMD_SPEAK, payload, (uint16_t)(1U + text_len));
}

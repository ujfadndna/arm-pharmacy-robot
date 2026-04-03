#pragma once

#ifndef MAIXCAM_UART_H_
#define MAIXCAM_UART_H_

#include "vision_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== MaixCam2 -> RA6M5 命令 ========== */
#define MC_CMD_HEARTBEAT     0x01
#define MC_CMD_TAG_SINGLE    0x02  /* uint8 tag_id, float32 x,y,z, uint8 conf = 14B */
#define MC_CMD_SCAN_START    0x03
#define MC_CMD_SCAN_TAG      0x04  /* 同0x02 14B */
#define MC_CMD_SCAN_END      0x05  /* uint8 total_count = 1B */
#define MC_CMD_ALIGN_OK      0x06  /* uint8 target_id, float32 dx,dy, uint8 stable = 10B */
#define MC_CMD_ALIGN_FAIL    0x07  /* uint8 target_id, uint8 reason = 2B */
#define MC_CMD_QR_RESULT     0x08  /* 复用 vision_qr_result_t */
#define MC_CMD_YOLO_RESULT   0x09  /* 复用 vision_detect_result_t */
#define MC_CMD_VOICE_ACTION  0x0A  /* uint8 action_type, char slot_from[3], slot_to[3], uint8 rsv = 8B */
#define MC_CMD_ACK           0x0B  /* uint8 ack_cmd, uint8 status = 2B */

/* ========== RA6M5 -> MaixCam2 命令 ========== */
#define MC_CMD_REQ_SCAN      0x20  /* uint8 scan_type, uint8 frames_req = 2B */
#define MC_CMD_REQ_ALIGN     0x21  /* uint8 target_id, uint16 timeout_ms, uint8 stable_req = 4B */
#define MC_CMD_REQ_QR        0x22  /* uint16 timeout_ms = 2B */
#define MC_CMD_REQ_YOLO      0x23  /* uint8 class_filter, uint8 conf_thresh, uint16 timeout_ms = 4B */
#define MC_CMD_SPEAK         0x24  /* uint8 len, char text[N] N<=63 */
#define MC_CMD_SET_MODE      0x25  /* uint8 mode, uint8 param = 2B */

/* ========== 数据结构 ========== */
typedef struct __attribute__((packed)) {
    uint8_t tag_id;
    float x;
    float y;
    float z;
    uint8_t conf;
} mc_tag_result_t;   /* 14B */

typedef struct __attribute__((packed)) {
    uint8_t target_id;
    float dx;
    float dy;
    uint8_t stable;
} mc_align_ok_t;     /* 10B */

typedef struct __attribute__((packed)) {
    uint8_t target_id;
    uint8_t reason;
} mc_align_fail_t;   /* 2B */

typedef struct __attribute__((packed)) {
    uint8_t action_type;
    char slot_from[3];
    char slot_to[3];
    uint8_t rsv;
} mc_voice_action_t; /* 8B */

typedef struct __attribute__((packed)) {
    uint8_t scan_type;
    uint8_t frames_req;
} mc_req_scan_t;

typedef struct __attribute__((packed)) {
    uint8_t target_id;
    uint16_t timeout_ms;
    uint8_t stable_req;
} mc_req_align_t;

typedef struct __attribute__((packed)) {
    uint16_t timeout_ms;
} mc_req_qr_t;

typedef struct __attribute__((packed)) {
    uint8_t class_filter;
    uint8_t conf_thresh;
    uint16_t timeout_ms;
} mc_req_yolo_t;

/* ========== 回调类型 ========== */
typedef void (*mc_tag_cb_t)(const mc_tag_result_t *r);
typedef void (*mc_align_ok_cb_t)(const mc_align_ok_t *r);
typedef void (*mc_align_fail_cb_t)(const mc_align_fail_t *r);
typedef void (*mc_qr_cb_t)(const vision_qr_result_t *r);
typedef void (*mc_yolo_cb_t)(const vision_detect_result_t *r);
typedef void (*mc_voice_action_cb_t)(const mc_voice_action_t *r);

typedef struct {
    mc_tag_cb_t          on_tag;
    mc_align_ok_cb_t     on_align_ok;
    mc_align_fail_cb_t   on_align_fail;
    mc_qr_cb_t           on_qr;
    mc_yolo_cb_t         on_yolo;
    mc_voice_action_cb_t on_voice_action;
} maixcam_uart_callbacks_t;

/* ========== 公共 API ========== */
int  maixcam_uart_init(const maixcam_uart_callbacks_t *cbs);
void maixcam_uart_rx_byte(uint8_t byte);

int  maixcam_uart_send_req_scan(uint8_t scan_type, uint8_t frames);
int  maixcam_uart_send_req_align(uint8_t target_id, uint16_t timeout_ms, uint8_t stable);
int  maixcam_uart_send_req_qr(uint16_t timeout_ms);
int  maixcam_uart_send_req_yolo(uint8_t class_filter, uint8_t conf_thresh, uint16_t timeout_ms);
int  maixcam_uart_send_speak(const char *text, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* MAIXCAM_UART_H_ */

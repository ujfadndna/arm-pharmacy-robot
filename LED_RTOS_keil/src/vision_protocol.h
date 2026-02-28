/**
 ******************************************************************************
 * @file    vision_protocol.h
 * @brief   MaixCam2 <-> RA6M5 WiFi UDP通信协议定义
 * @note    两端共用协议格式，MaixCam2用Python实现，RA6M5用C实现
 ******************************************************************************
 */

#ifndef VISION_PROTOCOL_H_
#define VISION_PROTOCOL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 帧格式定义 ========== */
/*
 * | 帧头      | CMD  | LEN    | DATA    | CHK   |
 * | 0xAA 0x55 | 1B   | 2B(LE) | N字节   | 1B    |
 *
 * CHK = XOR(CMD, LEN_L, LEN_H, DATA[0..N-1])
 */

#define VISION_FRAME_HEADER_0       0xAA
#define VISION_FRAME_HEADER_1       0x55
#define VISION_FRAME_HEADER_SIZE    2
#define VISION_FRAME_MIN_SIZE       6   /* 帧头(2) + CMD(1) + LEN(2) + CHK(1) */
#define VISION_FRAME_MAX_DATA       256

/* ========== 命令定义 ========== */
#define VISION_CMD_HEARTBEAT        0x01    /* 心跳 */
#define VISION_CMD_DETECT_RESULT    0x02    /* 检测结果 (MaixCam2 -> RA6M5) */
#define VISION_CMD_QR_RESULT        0x03    /* QR码结果 (MaixCam2 -> RA6M5) */
#define VISION_CMD_TASK_REQUEST     0x10    /* 任务指令 (RA6M5 -> MaixCam2) */
#define VISION_CMD_TASK_ACK         0x11    /* 任务确认 (MaixCam2 -> RA6M5) */
#define VISION_CMD_STATUS           0x20    /* 状态查询 */

/* ========== 检测结果数据格式 (CMD=0x02) ========== */
/*
 * | count | obj[0] | obj[1] | ... |
 * | 1B    | 14B    | 14B    | ... |
 *
 * 每个obj:
 * | x     | y     | w     | h     | class_id | score      |
 * | 2B LE | 2B LE | 2B LE | 2B LE | 2B LE    | 4B float LE|
 */

#define VISION_OBJ_SIZE             14  /* 每个检测对象的字节数 */
#define VISION_MAX_OBJECTS          16  /* 最大检测对象数 */

typedef struct __attribute__((packed)) {
    int16_t  x;         /* 边界框左上角X (像素) */
    int16_t  y;         /* 边界框左上角Y (像素) */
    uint16_t w;         /* 边界框宽度 */
    uint16_t h;         /* 边界框高度 */
    uint16_t class_id;  /* 类别ID */
    float    score;     /* 置信度 0.0~1.0 */
} vision_object_t;

typedef struct {
    uint8_t count;                              /* 检测到的对象数量 */
    vision_object_t objects[VISION_MAX_OBJECTS];
} vision_detect_result_t;

/* ========== QR码结果数据格式 (CMD=0x03) ========== */
/*
 * | len | string |
 * | 1B  | N字节  |
 */

#define VISION_QR_MAX_LEN           128

typedef struct {
    uint8_t len;
    char    data[VISION_QR_MAX_LEN];
} vision_qr_result_t;

/* ========== 任务指令数据格式 (CMD=0x10) ========== */

#define VISION_TASK_SCAN_QR         0x01    /* 扫描QR码 */
#define VISION_TASK_DETECT_DRUG     0x02    /* 检测药品 */
#define VISION_TASK_CALIBRATE       0x03    /* 手眼标定 */

typedef struct {
    uint8_t task_type;
    uint8_t params[8];  /* 任务参数 */
} vision_task_request_t;

/* ========== 网络配置 ========== */
#define VISION_UDP_PORT             8888    /* UDP端口 */
#define VISION_HEARTBEAT_INTERVAL   1000    /* 心跳间隔(ms) */
#define VISION_TIMEOUT_MS           3000    /* 超时时间(ms) */

/* ========== AprilTag消息格式 (文本协议) ========== */
/*
 * MaixCam2 → RA6M5 (UART文本协议):
 *
 * 单个Tag检测结果:
 *   TAG:<id>,<x>,<y>,<z>\n
 *   例: TAG:0,150.5,350.2,-50.0\n
 *
 * 批量扫描结果:
 *   SCAN_START\n
 *   TAG:0,150.5,350.2,-50.0\n
 *   TAG:1,230.0,350.5,-50.0\n
 *   TAG:2,150.3,350.1,30.0\n
 *   SCAN_END\n
 *
 * 字段说明:
 *   id  - AprilTag ID (0-255)
 *   x,y,z - 基座坐标系下的位置 (mm)，已经过手眼标定转换
 */

#define TAG_MSG_PREFIX          "TAG:"
#define TAG_MSG_SCAN_START      "SCAN_START"
#define TAG_MSG_SCAN_END        "SCAN_END"

/* AprilTag检测结果结构 */
typedef struct {
    uint8_t tag_id;             /* AprilTag ID */
    float x, y, z;              /* 基座坐标系位置 (mm) */
    bool valid;                 /* 数据是否有效 */
} apriltag_result_t;

#define APRILTAG_MAX_RESULTS    16  /* 单次扫描最大Tag数量 */

typedef struct {
    apriltag_result_t tags[APRILTAG_MAX_RESULTS];
    uint8_t count;              /* 检测到的Tag数量 */
} apriltag_scan_result_t;

#ifdef __cplusplus
}
#endif

#endif /* VISION_PROTOCOL_H_ */

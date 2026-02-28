/**
 ******************************************************************************
 * @file    vision_wifi.h
 * @brief   MaixCam2 WiFi UDP通信驱动接口
 * @note    通过W800模块实现UDP通信，接收视觉识别结果
 ******************************************************************************
 */

#ifndef VISION_WIFI_H_
#define VISION_WIFI_H_

#include "vision_protocol.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 连接状态 ========== */
typedef enum {
    VISION_STATE_DISCONNECTED = 0,
    VISION_STATE_CONNECTING,
    VISION_STATE_CONNECTED,
    VISION_STATE_ERROR
} vision_state_t;

/* ========== 回调函数类型 ========== */
typedef void (*vision_detect_callback_t)(const vision_detect_result_t *result);
typedef void (*vision_qr_callback_t)(const vision_qr_result_t *result);
typedef void (*vision_state_callback_t)(vision_state_t state);

/* ========== 配置结构 ========== */
typedef struct {
    const char *ssid;           /* WiFi SSID */
    const char *password;       /* WiFi密码 */
    const char *remote_ip;      /* MaixCam2 IP地址 */
    uint16_t   remote_port;     /* UDP端口 (默认8888) */
    vision_detect_callback_t on_detect;     /* 检测结果回调 */
    vision_qr_callback_t     on_qr;         /* QR码回调 */
    vision_state_callback_t  on_state;      /* 状态变化回调 */
} vision_wifi_config_t;

/* ========== 公共接口 ========== */

/**
 * @brief 初始化WiFi通信模块
 * @param config 配置参数
 * @return 0=成功, -1=失败
 */
int vision_wifi_init(const vision_wifi_config_t *config);

/**
 * @brief 连接WiFi并建立UDP通信
 * @return 0=成功, -1=失败
 * @note 阻塞函数，超时返回失败
 */
int vision_wifi_connect(void);

/**
 * @brief 处理接收数据 (在UART接收中断或轮询中调用)
 * @param data 接收到的数据
 * @param len 数据长度
 */
void vision_wifi_rx_handler(const uint8_t *data, uint32_t len);

/**
 * @brief 发送任务指令给MaixCam2
 * @param task 任务请求
 * @return 0=成功, -1=失败
 */
int vision_wifi_send_task(const vision_task_request_t *task);

/**
 * @brief 发送心跳包
 */
void vision_wifi_send_heartbeat(void);

/**
 * @brief 获取当前连接状态
 */
vision_state_t vision_wifi_get_state(void);

/**
 * @brief 获取最近一次检测结果
 * @param result 输出结果
 * @return true=有新数据, false=无新数据
 */
bool vision_wifi_get_detect_result(vision_detect_result_t *result);

/**
 * @brief 周期性处理 (在主循环或定时器中调用)
 * @note 处理超时、重连等逻辑
 */
void vision_wifi_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* VISION_WIFI_H_ */

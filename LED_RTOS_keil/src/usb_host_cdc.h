/**
 ******************************************************************************
 * @file    usb_host_cdc.h
 * @brief   USB Host CDC驱动接口 (连接STM32 Dummy机械臂)
 * @note    通过USB Full-Speed Host连接CDC复合设备
 *          VID: 0x1209, PID: 0x0D32
 *          使用Endpoint 1 (0x01/0x81) ASCII协议通道
 ******************************************************************************
 */

#ifndef USB_HOST_CDC_H_
#define USB_HOST_CDC_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 ========== */
#define USB_CDC_RX_BUF_SIZE     512     /* 接收缓冲区大小 */
#define USB_CDC_TX_BUF_SIZE     256     /* 发送缓冲区大小 */
#define USB_CDC_LINE_BUF_SIZE   256     /* 行缓冲区大小 */
#define USB_CDC_DEFAULT_TIMEOUT 1000    /* 默认超时 (ms) */

/* ========== 连接状态 ========== */
typedef enum {
    USB_CDC_STATE_DISCONNECTED = 0,     /* 未连接 */
    USB_CDC_STATE_ATTACHED,             /* 已插入，枚举中 */
    USB_CDC_STATE_CONFIGURED,           /* 已配置，SET_LINE_CODING中 */
    USB_CDC_STATE_READY                 /* 就绪，可收发数据 */
} usb_cdc_state_t;

/* ========== 公共接口 ========== */

/**
 * @brief 初始化USB Host CDC驱动
 * @note  内部创建FreeRTOS任务处理USB事件
 *        必须在RASC生成USB模块后调用
 */
void usb_cdc_init(void);

/**
 * @brief 获取连接状态
 * @return 当前USB CDC连接状态
 */
usb_cdc_state_t usb_cdc_get_state(void);

/**
 * @brief 检查设备是否就绪
 * @return true=可收发数据, false=未就绪
 */
bool usb_cdc_is_ready(void);

/**
 * @brief 发送数据
 * @param data 数据指针
 * @param len  数据长度 (最大64字节/包)
 * @param timeout_ms 超时时间
 * @return 实际发送字节数, -1=失败
 */
int usb_cdc_write(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief 发送字符串
 * @param str 以\0结尾的字符串
 * @param timeout_ms 超时时间
 * @return 实际发送字节数, -1=失败
 */
int usb_cdc_write_string(const char *str, uint32_t timeout_ms);

/**
 * @brief 接收数据
 * @param buf 接收缓冲区
 * @param max_len 缓冲区大小
 * @param timeout_ms 超时时间
 * @return 实际接收字节数, -1=失败, 0=超时无数据
 */
int usb_cdc_read(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms);

/**
 * @brief 读取一行响应 (以\r\n结尾)
 * @param buf 接收缓冲区
 * @param max_len 缓冲区大小
 * @param timeout_ms 超时时间
 * @return 行长度(不含\r\n), -1=失败/超时
 */
int usb_cdc_read_line(char *buf, uint16_t max_len, uint32_t timeout_ms);

/**
 * @brief 获取USB设备地址 (调试用)
 * @return 设备地址, 0=未连接
 */
uint8_t usb_cdc_get_device_address(void);

/**
 * @brief 清空接收缓冲区和信号量
 * @note  在需要丢弃残留响应时调用
 */
void usb_cdc_flush(void);

/**
 * @brief 发送命令并等待响应
 * @param cmd 命令字符串 (不含\r\n，内部自动添加)
 * @param resp 响应缓冲区 (可为NULL)
 * @param resp_max 响应缓冲区大小
 * @param timeout_ms 超时时间
 * @return 响应长度, -1=失败/超时
 */
int usb_cdc_command(const char *cmd, char *resp, uint16_t resp_max, uint32_t timeout_ms);

/**
 * @brief 发送阻塞命令并等待响应 (支持长超时+看门狗刷新)
 * @note  用于HOME/RESET等需要物理运动完成的命令，最长可等30s
 * @param cmd 命令字符串 (不含\r\n)
 * @param resp 响应缓冲区 (可为NULL)
 * @param resp_max 响应缓冲区大小
 * @param timeout_ms 总超时时间 (可达30000ms)
 * @return 响应长度, -1=失败/超时
 */
int usb_cdc_command_blocking(const char *cmd, char *resp, uint16_t resp_max,
                             uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* USB_HOST_CDC_H_ */

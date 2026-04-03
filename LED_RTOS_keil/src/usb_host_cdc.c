/**
 ******************************************************************************
 * @file    usb_host_cdc.c
 * @brief   USB Host CDC驱动实现
 * @note    使用FSP r_usb_basic + r_usb_hcdc 驱动
 *          连接STM32 Dummy机械臂 (VID:0x1209 PID:0x0D32)
 *
 * 前置条件 (RASC配置):
 *   1. UCLK = 48MHz (PLL2 / 5)
 *   2. r_usb_basic: Host, Hi-Speed, USB_IP1, callback=usb_host_callback
 *   3. r_usb_hcdc: 挂在r_usb_basic之上
 *   4. USBHS引脚: USBHS_DP, USBHS_DM, USBHS_VBUSEN=PB00, USBHS_OVRCURA=P707
 ******************************************************************************
 */

#include "usb_host_cdc.h"
#include "new_thread0.h"
#include "debug_uart.h"
#include "log.h"
#include "watchdog.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#define TAG "USB_CDC"

/* ========== USB CDC Line Coding ========== */
/* 使用FSP原生类型 usb_hcdc_linecoding_t (定义在 r_usb_hcdc_api.h) */

/* ========== 内部状态 ========== */
static volatile usb_cdc_state_t g_state = USB_CDC_STATE_DISCONNECTED;
static uint8_t g_device_address = 0;

/* USB传输缓冲区 (4字节对齐，USB DMA要求) */
static uint8_t g_rx_buf[USB_CDC_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t g_tx_buf[USB_CDC_TX_BUF_SIZE] __attribute__((aligned(4)));

/* 行接收缓冲区 (环形) */
static uint8_t  g_line_ring[USB_CDC_RX_BUF_SIZE];
static volatile uint16_t g_ring_head = 0;
static volatile uint16_t g_ring_tail = 0;

/* 同步原语 */
static SemaphoreHandle_t g_tx_done_sem = NULL;
static SemaphoreHandle_t g_rx_done_sem = NULL;
static SemaphoreHandle_t g_ctrl_done_sem = NULL;
static StaticSemaphore_t g_tx_done_buf;
static StaticSemaphore_t g_rx_done_buf;
static StaticSemaphore_t g_ctrl_done_buf;

/* USB实例 (RASC生成在 new_thread0.h 中，已通过include引入) */

/* 最后一次读取的字节数 */
static volatile uint16_t g_last_rx_size = 0;

/* Line Coding配置 */
static usb_hcdc_linecoding_t g_line_coding = {
    .dwdte_rate   = USB_HCDC_SPEED_115200,
    .bchar_format = USB_HCDC_STOP_BIT_1,
    .bparity_type = USB_HCDC_PARITY_BIT_NONE,
    .bdata_bits   = USB_HCDC_DATA_BIT_8,
    .rsv          = 0
};

/* ========== 环形缓冲区操作 ========== */

static inline uint16_t ring_count(void)
{
    return (uint16_t)((g_ring_head - g_ring_tail) % USB_CDC_RX_BUF_SIZE);
}

static void ring_push(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        g_line_ring[g_ring_head] = data[i];
        g_ring_head = (g_ring_head + 1) % USB_CDC_RX_BUF_SIZE;
        /* 溢出时丢弃最旧数据 */
        if (g_ring_head == g_ring_tail) {
            g_ring_tail = (g_ring_tail + 1) % USB_CDC_RX_BUF_SIZE;
        }
    }
}

static int ring_pop(uint8_t *out)
{
    if (g_ring_head == g_ring_tail) return -1;
    *out = g_line_ring[g_ring_tail];
    g_ring_tail = (g_ring_tail + 1) % USB_CDC_RX_BUF_SIZE;
    return 0;
}

/* ========== 内部函数声明 ========== */
static void usb_send_set_line_coding(void);
static void usb_start_bulk_read(void);

/* ========== USB Host 回调 ========== */

/**
 * @brief USB事件回调 (RASC配置的callback名)
 * @note  在USB中断上下文或USB任务上下文中调用
 */
void usb_host_callback(usb_event_info_t *p_event, usb_hdl_t handle, usb_onoff_t state)
{
    (void)handle;
    (void)state;
    BaseType_t xHigher = pdFALSE;

    switch (p_event->event) {

    case USB_STATUS_CONFIGURED:
        /* 设备枚举完成 */
        g_device_address = p_event->device_address;
        g_state = USB_CDC_STATE_CONFIGURED;
        LOG_I(TAG, "Device configured, addr=%d", g_device_address);
        /* 发送 SET_LINE_CODING */
        usb_send_set_line_coding();
        break;

    case USB_STATUS_REQUEST_COMPLETE:
        /* 控制传输完成 (SET_LINE_CODING等) */
        if (g_state == USB_CDC_STATE_CONFIGURED) {
            g_state = USB_CDC_STATE_READY;
            LOG_I(TAG, "Line coding set, ready");
            /* 启动持续Bulk IN读取 */
            usb_start_bulk_read();
        }
        if (g_ctrl_done_sem) {
            xSemaphoreGiveFromISR(g_ctrl_done_sem, &xHigher);
        }
        break;

    case USB_STATUS_READ_COMPLETE:
        /* Bulk IN 数据到达 */
        g_last_rx_size = p_event->data_size;
        if (g_last_rx_size > 0) {
            ring_push(g_rx_buf, g_last_rx_size);
        }
        if (g_rx_done_sem) {
            xSemaphoreGiveFromISR(g_rx_done_sem, &xHigher);
        }
        /* 继续读取 */
        if (g_state == USB_CDC_STATE_READY) {
            usb_start_bulk_read();
        }
        break;

    case USB_STATUS_WRITE_COMPLETE:
        /* Bulk OUT 发送完成 */
        LOG_D(TAG, "WRITE_COMPLETE");
        if (g_tx_done_sem) {
            xSemaphoreGiveFromISR(g_tx_done_sem, &xHigher);
        }
        break;

    case USB_STATUS_DETACH:
        /* 设备拔出 */
        g_state = USB_CDC_STATE_DISCONNECTED;
        g_device_address = 0;
        g_ring_head = 0;
        g_ring_tail = 0;
        LOG_W(TAG, "Device detached");
        break;

    default:
        /* 记录未处理的USB事件，帮助诊断连接问题 */
        LOG_W(TAG, "Unhandled USB event: %d", (int)p_event->event);
        break;
    }

    portYIELD_FROM_ISR(xHigher);
}

/* ========== 内部函数 ========== */

/**
 * @brief 发送SET_LINE_CODING类请求
 */
static void usb_send_set_line_coding(void)
{
    usb_setup_t setup;
    /* FSP request_type = bmRequestType(低字节) | bRequest(高字节)
     * bmRequestType = USB_HOST_TO_DEV | USB_CLASS | USB_INTERFACE = 0x0021
     * USB_CDC_SET_LINE_CODING = 0x2000 (bRequest=0x20 << 8)
     * 合并: 0x2000 | 0x0021 = 0x2021 */
    setup.request_type  = USB_CDC_SET_LINE_CODING | USB_HOST_TO_DEV | USB_CLASS | USB_INTERFACE;
    setup.request_value = 0;
    setup.request_index = 0;    /* Interface 0 (第一个CDC ACM) */
    setup.request_length = (uint16_t)sizeof(usb_hcdc_linecoding_t);

    fsp_err_t err = R_USB_HostControlTransfer(
        &g_basic0_ctrl,
        &setup,
        (uint8_t *)&g_line_coding,
        g_device_address
    );

    if (FSP_SUCCESS != err) {
        LOG_E(TAG, "SET_LINE_CODING failed: %d", (int)err);
        /* 即使失败也尝试进入READY，有些CDC设备不需要Line Coding */
        g_state = USB_CDC_STATE_READY;
        usb_start_bulk_read();
    }
}

/**
 * @brief 启动Bulk IN持续读取
 */
static void usb_start_bulk_read(void)
{
    fsp_err_t err = R_USB_Read(
        &g_basic0_ctrl,
        g_rx_buf,
        64,     /* CDC Full-Speed最大包长 */
        g_device_address
    );

    if (FSP_SUCCESS != err) {
        LOG_E(TAG, "Bulk IN start failed: %d", (int)err);
    }
}

/* ========== 公共接口实现 ========== */

void usb_cdc_init(void)
{
    /* 创建同步信号量 */
    g_tx_done_sem  = xSemaphoreCreateBinaryStatic(&g_tx_done_buf);
    g_rx_done_sem  = xSemaphoreCreateBinaryStatic(&g_rx_done_buf);
    g_ctrl_done_sem = xSemaphoreCreateBinaryStatic(&g_ctrl_done_buf);

    /* 清空环形缓冲区 */
    g_ring_head = 0;
    g_ring_tail = 0;
    g_state = USB_CDC_STATE_DISCONNECTED;

    /* 打开USB Host */
    fsp_err_t err = R_USB_Open(&g_basic0_ctrl, &g_basic0_cfg);
    if (FSP_SUCCESS != err) {
        LOG_E(TAG, "R_USB_Open FAILED: err=%d (already open?)", (int)err);
        /* 如果是重复调用导致的失败，尝试先关再开 */
        R_USB_Close(&g_basic0_ctrl);
        err = R_USB_Open(&g_basic0_ctrl, &g_basic0_cfg);
        if (FSP_SUCCESS != err) {
            LOG_E(TAG, "R_USB_Open retry FAILED: %d", (int)err);
            return;
        }
        LOG_I(TAG, "R_USB_Open retry OK (after close)");
    }

    LOG_I(TAG, "USB Host CDC initialized, waiting for device...");
}

usb_cdc_state_t usb_cdc_get_state(void)
{
    return g_state;
}

uint8_t usb_cdc_get_device_address(void)
{
    return g_device_address;
}

bool usb_cdc_is_ready(void)
{
    return (g_state == USB_CDC_STATE_READY);
}

int usb_cdc_write(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!usb_cdc_is_ready() || data == NULL || len == 0) return -1;
    if (len > USB_CDC_TX_BUF_SIZE) len = USB_CDC_TX_BUF_SIZE;

    memcpy(g_tx_buf, data, len);

    /* 预清空残留的 WRITE_COMPLETE 信号，防止假成功 */
    xSemaphoreTake(g_tx_done_sem, 0);

    fsp_err_t err = R_USB_Write(
        &g_basic0_ctrl,
        g_tx_buf,
        len,
        g_device_address
    );

    if (FSP_SUCCESS != err) {
        LOG_E(TAG, "R_USB_Write failed: %d", (int)err);
        return -1;
    }

    /* 等待发送完成 */
    if (xSemaphoreTake(g_tx_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        LOG_W(TAG, "TX timeout, state=%d addr=%d", (int)g_state, g_device_address);
        /* 尝试 Clear Endpoint Halt 恢复 pipe，并等待控制传输完成 */
        usb_setup_t setup;
        setup.request_type  = USB_HOST_TO_DEV | USB_STANDARD | USB_ENDPOINT;
        setup.request_value = 0;    /* CLEAR_FEATURE(ENDPOINT_HALT) */
        setup.request_index = 0x01; /* EP1 OUT */
        setup.request_length = 0;
        xSemaphoreTake(g_ctrl_done_sem, 0);  /* 预清空控制传输信号 */
        R_USB_HostControlTransfer(&g_basic0_ctrl, &setup, NULL, g_device_address);
        xSemaphoreTake(g_ctrl_done_sem, pdMS_TO_TICKS(500));  /* 等待清除完成 */
        usb_start_bulk_read();  /* 重启 Bulk IN */
        return -1;
    }

    return (int)len;
}

int usb_cdc_write_string(const char *str, uint32_t timeout_ms)
{
    if (str == NULL) return -1;
    return usb_cdc_write((const uint8_t *)str, (uint16_t)strlen(str), timeout_ms);
}

int usb_cdc_read(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms)
{
    if (!usb_cdc_is_ready() || buf == NULL || max_len == 0) return -1;

    /* 先检查环形缓冲区是否有数据 */
    uint16_t avail = ring_count();
    if (avail == 0) {
        /* 等待新数据到达 */
        if (xSemaphoreTake(g_rx_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            return 0;  /* 超时 */
        }
        avail = ring_count();
    }

    /* 从环形缓冲区取数据 */
    uint16_t to_read = (avail < max_len) ? avail : max_len;
    for (uint16_t i = 0; i < to_read; i++) {
        if (ring_pop(&buf[i]) != 0) break;
    }

    return (int)to_read;
}

int usb_cdc_read_line(char *buf, uint16_t max_len, uint32_t timeout_ms)
{
    if (!usb_cdc_is_ready() || buf == NULL || max_len < 2) return -1;

    uint16_t pos = 0;
    uint32_t start = xTaskGetTickCount();

    while (pos < max_len - 1) {
        uint32_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
        if (elapsed >= timeout_ms) return -1;  /* 超时 */

        uint8_t ch;
        if (ring_pop(&ch) == 0) {
            if (ch == '\n') {
                /* 去掉末尾\r */
                if (pos > 0 && buf[pos - 1] == '\r') pos--;
                buf[pos] = '\0';
                return (int)pos;
            }
            buf[pos++] = (char)ch;
        } else {
            /* 缓冲区空，等待新数据 */
            uint32_t remain = timeout_ms - elapsed;
            xSemaphoreTake(g_rx_done_sem, pdMS_TO_TICKS(remain > 50 ? 50 : remain));
        }
    }

    buf[pos] = '\0';
    return (int)pos;
}

void usb_cdc_flush(void)
{
    /* 持续排空直到ring buffer安静20ms，确保所有残留数据被丢弃
     * max_total_ms 防止设备持续发数据时进入无限循环 */
    const uint32_t quiet_ms     = 20;
    const uint32_t max_total_ms = 200;  /* 最多等200ms，超时强制退出 */
    TickType_t total_start = xTaskGetTickCount();
    TickType_t quiet_start = xTaskGetTickCount();

    while (1) {
        uint32_t total_elapsed = (xTaskGetTickCount() - total_start) * portTICK_PERIOD_MS;
        if (total_elapsed >= max_total_ms) break;  /* 超过上限，强制退出 */

        uint32_t quiet_elapsed = (xTaskGetTickCount() - quiet_start) * portTICK_PERIOD_MS;
        if (quiet_elapsed >= quiet_ms) break;      /* 安静20ms，正常退出 */

        /* 检查是否有新数据到达 */
        if (ring_count() > 0) {
            g_ring_head = 0;
            g_ring_tail = 0;
            quiet_start = xTaskGetTickCount();  /* 重置安静计时器 */
        }

        /* 短等待，让USB中断有机会推数据 */
        if (g_rx_done_sem) {
            xSemaphoreTake(g_rx_done_sem, pdMS_TO_TICKS(5));
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    /* 最终清空 */
    g_ring_head = 0;
    g_ring_tail = 0;
    if (g_rx_done_sem) {
        while (xSemaphoreTake(g_rx_done_sem, 0) == pdTRUE) {}
    }
}

int usb_cdc_command(const char *cmd, char *resp, uint16_t resp_max, uint32_t timeout_ms)
{
    if (cmd == NULL) return -1;

    if (!usb_cdc_is_ready()) {
        debug_print("[CDC TX] Not ready, state=");
        debug_print_int((int)g_state);
        debug_println("");
        return -1;
    }

    /* 清空残留数据，确保读到的是本次命令的响应 */
    usb_cdc_flush();

    /* 构造命令: cmd + \r\n */
    uint16_t cmd_len = (uint16_t)strlen(cmd);
    if (cmd_len + 2 > USB_CDC_TX_BUF_SIZE) return -1;

    memcpy(g_tx_buf, cmd, cmd_len);
    g_tx_buf[cmd_len]     = '\r';
    g_tx_buf[cmd_len + 1] = '\n';

    /* 调试日志: 显示发送的命令和USB状态 */
    debug_print("[CDC TX] ");
    debug_print(cmd);
    debug_print(" (addr=");
    debug_print_int(g_device_address);
    debug_println(")");

    /* 预清空残留的 WRITE_COMPLETE 信号，防止假成功 */
    xSemaphoreTake(g_tx_done_sem, 0);

    /* 发送 */
    fsp_err_t err = R_USB_Write(
        &g_basic0_ctrl,
        g_tx_buf,
        cmd_len + 2,
        g_device_address
    );

    if (FSP_SUCCESS != err) {
        debug_print("[CDC TX] R_USB_Write err=");
        debug_print_int((int)err);
        debug_println("");
        return -1;
    }

    if (xSemaphoreTake(g_tx_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        debug_print("[CDC TX] TX timeout, state=");
        debug_print_int((int)g_state);
        debug_println("");
        /* Pipe 恢复: Clear Endpoint Halt，等待控制传输完成再重启 Bulk IN */
        usb_setup_t setup;
        setup.request_type  = USB_HOST_TO_DEV | USB_STANDARD | USB_ENDPOINT;
        setup.request_value = 0;    /* CLEAR_FEATURE(ENDPOINT_HALT) */
        setup.request_index = 0x01; /* EP1 OUT */
        setup.request_length = 0;
        xSemaphoreTake(g_ctrl_done_sem, 0);  /* 预清空控制传输信号 */
        R_USB_HostControlTransfer(&g_basic0_ctrl, &setup, NULL, g_device_address);
        xSemaphoreTake(g_ctrl_done_sem, pdMS_TO_TICKS(500));  /* 等待清除完成 */
        usb_start_bulk_read();  /* 重启 Bulk IN */
        return -1;
    }

    /* 读取响应 */
    if (resp != NULL && resp_max > 0) {
        int r = usb_cdc_read_line(resp, resp_max, timeout_ms);
        debug_print("[CDC RX] ");
        debug_println(r >= 0 ? resp : "(timeout)");
        return r;
    }

    return 0;
}

int usb_cdc_command_blocking(const char *cmd, char *resp, uint16_t resp_max,
                             uint32_t timeout_ms)
{
    if (cmd == NULL) return -1;

    if (!usb_cdc_is_ready()) {
        debug_print("[CDC TX] Not ready, state=");
        debug_print_int((int)g_state);
        debug_println("");
        return -1;
    }

    /* 清空残留数据 */
    usb_cdc_flush();

    /* 构造命令: cmd + \r\n */
    uint16_t cmd_len = (uint16_t)strlen(cmd);
    if (cmd_len + 2 > USB_CDC_TX_BUF_SIZE) return -1;

    memcpy(g_tx_buf, cmd, cmd_len);
    g_tx_buf[cmd_len]     = '\r';
    g_tx_buf[cmd_len + 1] = '\n';

    debug_print("[CDC TX] ");
    debug_print(cmd);
    debug_print(" (addr=");
    debug_print_int(g_device_address);
    debug_print(", blocking ");
    debug_print_int((int)(timeout_ms / 1000));
    debug_println("s)");

    /* 预清空残留的 WRITE_COMPLETE 信号 */
    xSemaphoreTake(g_tx_done_sem, 0);

    /* 发送 (短超时1s) */
    fsp_err_t err = R_USB_Write(&g_basic0_ctrl, g_tx_buf, cmd_len + 2, g_device_address);
    if (FSP_SUCCESS != err) {
        debug_print("[CDC TX] R_USB_Write err=");
        debug_print_int((int)err);
        debug_println("");
        return -1;
    }

    if (xSemaphoreTake(g_tx_done_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
        debug_println("[CDC TX] TX timeout (blocking cmd)");
        /* Pipe 恢复，等待控制传输完成 */
        usb_setup_t setup;
        setup.request_type  = USB_HOST_TO_DEV | USB_STANDARD | USB_ENDPOINT;
        setup.request_value = 0;
        setup.request_index = 0x01;
        setup.request_length = 0;
        xSemaphoreTake(g_ctrl_done_sem, 0);
        R_USB_HostControlTransfer(&g_basic0_ctrl, &setup, NULL, g_device_address);
        xSemaphoreTake(g_ctrl_done_sem, pdMS_TO_TICKS(500));
        usb_start_bulk_read();
        return -1;
    }

    /* RX: 分段等待，每轮500ms + watchdog_refresh，总超时 timeout_ms */
    if (resp == NULL || resp_max == 0) return 0;

    uint32_t start = xTaskGetTickCount();
    const uint32_t chunk_ms = 500;

    while (1) {
        uint32_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
        if (elapsed >= timeout_ms) {
            debug_println("[CDC RX] (blocking timeout)");
            return -1;
        }

        uint32_t remain = timeout_ms - elapsed;
        uint32_t wait = (remain > chunk_ms) ? chunk_ms : remain;

        int r = usb_cdc_read_line(resp, resp_max, wait);
        if (r >= 0) {
            debug_print("[CDC RX] ");
            debug_println(resp);
            return r;
        }

        watchdog_refresh();
    }
}
/**
 * @file    llm_test.c
 * @brief   LLM调用测试代码 (RA6M5 + W800 + Render.com)
 * @note    在main.c或某个线程中调用测试
 */

#include "w800_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* WiFi配置 */
#define WIFI_SSID       "yamaRedmi K70"
#define WIFI_PASSWORD   "12345678"

/* LLM响应缓冲 */
static char g_llm_response[512];

/* LLM测试任务静态分配 */
static StaticTask_t g_llm_task_tcb;
static StackType_t g_llm_task_stack[2048];

/**
 * @brief 测试LLM调用
 * @note  在FreeRTOS任务中调用
 */
void test_llm_call(void)
{
    int ret;

    printf("=== LLM调用测试 ===\r\n");

    /* 1. 初始化W800 */
    w800_init(NULL);
    printf("[1] W800初始化完成\r\n");

    /* 2. 连接WiFi */
    printf("[2] 连接WiFi: %s\r\n", WIFI_SSID);
    ret = w800_connect_wifi_only(WIFI_SSID, WIFI_PASSWORD);
    if (ret != 0) {
        printf("WiFi连接失败: %d\r\n", ret);
        return;
    }
    printf("WiFi连接成功!\r\n");

    /* 3. 获取IP */
    char ip[16];
    if (w800_get_local_ip(ip, sizeof(ip)) == 0) {
        printf("本地IP: %s\r\n", ip);
    }

    /* 4. 调用LLM */
    printf("[3] 调用LLM: 帮我取阿莫西林\r\n");

    ret = w800_call_llm("帮我取阿莫西林", g_llm_response, sizeof(g_llm_response));

    if (ret > 0) {
        printf("LLM响应 (%d bytes):\r\n%s\r\n", ret, g_llm_response);
    } else {
        printf("LLM调用失败: %d\r\n", ret);
    }

    /* 5. 再测试一个英文指令 */
    printf("[4] 调用LLM: get ibuprofen\r\n");

    ret = w800_call_llm("get ibuprofen", g_llm_response, sizeof(g_llm_response));

    if (ret > 0) {
        printf("LLM响应 (%d bytes):\r\n%s\r\n", ret, g_llm_response);
    } else {
        printf("LLM调用失败: %d\r\n", ret);
    }

    printf("=== 测试完成 ===\r\n");
}

/**
 * @brief LLM测试任务 (FreeRTOS)
 */
void llm_test_task(void *pvParameters)
{
    (void)pvParameters;

    /* 等待系统稳定 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    test_llm_call();

    /* 任务完成后删除自己 */
    vTaskDelete(NULL);
}

/**
 * @brief 创建LLM测试任务
 * @note  在main()中调用
 */
void create_llm_test_task(void)
{
    xTaskCreateStatic(llm_test_task, "LLM_Test", 2048, NULL, 2,
                      g_llm_task_stack, &g_llm_task_tcb);
}

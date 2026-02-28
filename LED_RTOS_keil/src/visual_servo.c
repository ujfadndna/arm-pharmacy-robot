/**
 * @file    visual_servo.c
 * @brief   视觉伺服模块实现
 */

#include "visual_servo.h"
#include "maixcam_driver.h"
#include "motion_controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************
 * 全局变量
 ***************************************************************/
static vs_state_t g_vs_state = VS_IDLE;
static align_result_t g_align_result;
static SemaphoreHandle_t g_align_sem = NULL;

/***************************************************************
 * 内部函数
 ***************************************************************/

/**
 * @brief 等待运动完成（简化版）
 */
static int wait_motion_done_simple(uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount();

    while (1) {
        motion_state_t state = motion_get_state();

        if (state == MOTION_DONE || state == MOTION_IDLE) {
            return 0;
        }
        if (state == MOTION_ERROR) {
            return -1;
        }

        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > timeout_ms) {
            return -2;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/***************************************************************
 * API实现
 ***************************************************************/

void visual_servo_init(void)
{
    g_vs_state = VS_IDLE;
    g_align_result.success = false;
    g_align_result.dx = 0;
    g_align_result.dy = 0;

    if (g_align_sem == NULL) {
        g_align_sem = xSemaphoreCreateBinary();
    }
}

int visual_servo_request_align(int target_id)
{
    char cmd[32];

    /* 清空之前的结果 */
    g_align_result.success = false;
    g_vs_state = VS_WAITING_ALIGN;

    /* 清空信号量 */
    xSemaphoreTake(g_align_sem, 0);

    /* 发送精定位请求 */
    snprintf(cmd, sizeof(cmd), "ALIGN:%d\n", target_id);
    maixcam_send_string(cmd);

    return 0;
}

int visual_servo_wait_align(align_result_t *result, uint32_t timeout_ms)
{
    if (!result) return -1;

    /* 等待信号量 */
    if (xSemaphoreTake(g_align_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        *result = g_align_result;
        g_vs_state = g_align_result.success ? VS_DONE : VS_ERROR;
        return g_align_result.success ? 0 : -1;
    }

    /* 超时 */
    g_vs_state = VS_ERROR;
    result->success = false;
    return -2;
}

void visual_servo_handle_message(const char *msg)
{
    if (!msg) return;

    /* 解析 ALIGN_OK:dx,dy */
    if (strncmp(msg, "ALIGN_OK:", 9) == 0) {
        float dx, dy;
        if (sscanf(msg + 9, "%f,%f", &dx, &dy) == 2) {
            g_align_result.success = true;
            g_align_result.dx = dx;
            g_align_result.dy = dy;
            xSemaphoreGive(g_align_sem);
        }
    }
    /* 解析 ALIGN_FAIL */
    else if (strncmp(msg, "ALIGN_FAIL", 10) == 0) {
        g_align_result.success = false;
        g_align_result.dx = 0;
        g_align_result.dy = 0;
        xSemaphoreGive(g_align_sem);
    }
}

int visual_servo_align(int target_id, float *current_x, float *current_y)
{
    if (!current_x || !current_y) return -1;

    align_result_t result;
    int iteration;

    for (iteration = 0; iteration < VS_MAX_ITERATIONS; iteration++) {
        /* 1. 等待机械臂稳定 */
        vTaskDelay(pdMS_TO_TICKS(150));

        /* 2. 请求精定位 */
        visual_servo_request_align(target_id);

        /* 3. 等待结果 */
        int ret = visual_servo_wait_align(&result, VS_ALIGN_TIMEOUT_MS);
        if (ret < 0) {
            printf("[VS] Align failed or timeout\n");
            return ret;
        }

        printf("[VS] Iter %d: dx=%.1f, dy=%.1f\n",
               iteration + 1, result.dx, result.dy);

        /* 4. 检查是否已对准 */
        if (fabsf(result.dx) < VS_POSITION_THRESHOLD &&
            fabsf(result.dy) < VS_POSITION_THRESHOLD) {
            printf("[VS] Aligned! Final error: (%.1f, %.1f)\n",
                   result.dx, result.dy);
            return 0;
        }

        /* 5. 计算修正量（乘以增益避免过冲） */
        float correct_x = result.dx * VS_CORRECTION_GAIN;
        float correct_y = result.dy * VS_CORRECTION_GAIN;

        /* 6. 更新目标位置 */
        *current_x += correct_x;
        *current_y += correct_y;

        printf("[VS] Correcting: +%.1f, +%.1f -> (%.1f, %.1f)\n",
               correct_x, correct_y, *current_x, *current_y);

        /* 7. 移动到新位置 */
        ret = motion_move_relative(correct_x, correct_y, 0);
        if (ret < 0) {
            printf("[VS] Motion failed\n");
            return ret;
        }

        /* 8. 等待运动完成 */
        ret = wait_motion_done_simple(5000);
        if (ret < 0) {
            printf("[VS] Motion timeout\n");
            return ret;
        }
    }

    /* 达到最大迭代次数，使用当前位置 */
    printf("[VS] Max iterations reached, using current position\n");
    return 0;
}

vs_state_t visual_servo_get_state(void)
{
    return g_vs_state;
}

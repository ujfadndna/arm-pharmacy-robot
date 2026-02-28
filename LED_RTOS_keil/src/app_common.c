/***************************************************************
 * app_common.c - 全局变量定义和初始化
 ***************************************************************/

#include "app_common.h"

/***************************************************************
 * 全局变量定义
 ***************************************************************/
volatile MotorState_t g_motor_state = {0};
volatile SystemState_t g_system_state = STATE_IDLE;
volatile int g_wifi_error_code = 0;             /* WiFi错误码: 0=正常, -1~-5=错误 */

/* 静态分配存储 */
static StaticEventGroup_t s_event_group_buffer;
static StaticQueue_t s_vision_queue_buffer;
static StaticQueue_t s_voice_queue_buffer;
static StaticQueue_t s_target_queue_buffer;
static StaticSemaphore_t s_motor_mutex_buffer;

static uint8_t s_vision_queue_storage[4 * sizeof(vision_data_t)];
static uint8_t s_voice_queue_storage[2 * sizeof(VoiceCommand_t)];
static uint8_t s_target_queue_storage[1 * sizeof(TaskTarget_t)];

EventGroupHandle_t g_event_group = NULL;
QueueHandle_t g_vision_queue = NULL;
QueueHandle_t g_voice_queue = NULL;
QueueHandle_t g_target_queue = NULL;
SemaphoreHandle_t g_motor_mutex = NULL;

/***************************************************************
 * 初始化函数 - 在hal_entry()中调用
 ***************************************************************/
void app_common_init(void)
{
    /* 创建事件组 (静态分配) */
    g_event_group = xEventGroupCreateStatic(&s_event_group_buffer);
    configASSERT(g_event_group != NULL);

    /* 创建队列 (静态分配) */
    g_vision_queue = xQueueCreateStatic(4, sizeof(vision_data_t),
                                        s_vision_queue_storage,
                                        &s_vision_queue_buffer);
    g_voice_queue = xQueueCreateStatic(2, sizeof(VoiceCommand_t),
                                       s_voice_queue_storage,
                                       &s_voice_queue_buffer);
    g_target_queue = xQueueCreateStatic(1, sizeof(TaskTarget_t),
                                        s_target_queue_storage,
                                        &s_target_queue_buffer);
    configASSERT(g_vision_queue != NULL);
    configASSERT(g_voice_queue != NULL);
    configASSERT(g_target_queue != NULL);

    /* 创建互斥量 (静态分配) */
    g_motor_mutex = xSemaphoreCreateMutexStatic(&s_motor_mutex_buffer);
    configASSERT(g_motor_mutex != NULL);

    /* 初始化电机状态 */
    for (int i = 0; i < 6; i++)
    {
        g_motor_state.q_current[i] = 0.0f;
        g_motor_state.q_target[i] = 0.0f;
        g_motor_state.q_velocity[i] = 0.0f;
        g_motor_state.Iq_measured[i] = 0.0f;
    }

    /* 初始化药柜配置 */
    cabinet_config_init();
}

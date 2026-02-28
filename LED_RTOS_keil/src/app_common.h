/***************************************************************
 * app_common.h - 公共定义头文件
 *
 * 包含:
 * - 共享数据结构
 * - 事件组位定义
 * - 全局变量声明
 ***************************************************************/

#ifndef APP_COMMON_H_
#define APP_COMMON_H_

#include "hal_data.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"

/***************************************************************
 * 事件组位定义
 ***************************************************************/
#define BIT_MOTOR_FEEDBACK_READY    (1 << 0)    // 电机反馈就绪
#define BIT_TRAJECTORY_DONE         (1 << 1)    // 轨迹规划完成
#define BIT_COLLISION_DETECTED      (1 << 2)    // 碰撞检测
#define BIT_CONTACT_DETECTED        (1 << 3)    // 接触检测（放置）
#define BIT_UART_DATA_READY         (1 << 4)    // UART数据就绪

/***************************************************************
 * 电机状态结构体
 ***************************************************************/
typedef struct {
    float q_current[6];     // 当前关节角度 (rad)
    float q_target[6];      // 目标关节角度 (rad)
    float q_velocity[6];    // 当前关节速度 (rad/s)
    float Iq_measured[6];   // 电机电流 (A)
} MotorState_t;

/***************************************************************
 * 目标位姿结构体
 ***************************************************************/
typedef struct {
    float x, y, z;              // 末端位置 (mm)
    float roll, pitch, yaw;     // 末端姿态 (rad)
    float duration_ms;          // 运动时间 (ms)
} TaskTarget_t;

/***************************************************************
 * AprilTag位姿结构体
 ***************************************************************/
typedef struct {
    float x, y, z;              // 相机坐标系位置 (mm)
    float qx, qy, qz, qw;       // 四元数姿态
} AprilTagPose_t;

/***************************************************************
 * 视觉检测数据结构体
 ***************************************************************/
typedef struct {
    int16_t target_x;           // 目标中心X (像素)
    int16_t target_y;           // 目标中心Y (像素)
    float   confidence;         // 置信度 0.0~1.0
    uint16_t class_id;          // 类别ID
} vision_data_t;

/***************************************************************
 * 系统状态枚举
 ***************************************************************/
typedef enum {
    STATE_IDLE = 0,         // 待命
    STATE_LISTENING,        // 监听语音
    STATE_PLANNING,         // 轨迹规划中
    STATE_EXECUTING,        // 执行中
    STATE_GRASPING,         // 抓取中
    STATE_PLACING,          // 放置中
    STATE_ERROR,            // 错误态
} SystemState_t;

/***************************************************************
 * 语音指令枚举
 ***************************************************************/
typedef enum {
    VOICE_CMD_NONE = 0,
    VOICE_CMD_PICK,         // 取药
    VOICE_CMD_PLACE,        // 放药
    VOICE_CMD_STOP,         // 停止
} VoiceCommand_t;

/***************************************************************
 * 全局变量声明 (在app_common.c中定义)
 ***************************************************************/
extern volatile MotorState_t g_motor_state;
extern volatile SystemState_t g_system_state;
extern volatile int g_wifi_error_code;          /* WiFi错误码: -1~-5 */
extern EventGroupHandle_t g_event_group;
extern QueueHandle_t g_vision_queue;
extern QueueHandle_t g_voice_queue;
extern QueueHandle_t g_target_queue;
extern SemaphoreHandle_t g_motor_mutex;

/* 队列别名 (兼容旧代码) */
#define xVisionQueue    g_vision_queue
#define xVoiceQueue     g_voice_queue
#define xTargetQueue    g_target_queue

/***************************************************************
 * 全局初始化函数
 ***************************************************************/
void app_common_init(void);

/***************************************************************
 * 药柜配置 (详见 cabinet_config.h)
 ***************************************************************/
#include "cabinet_config.h"

#endif /* APP_COMMON_H_ */

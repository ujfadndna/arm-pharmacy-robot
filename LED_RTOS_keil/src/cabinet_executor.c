/**
 * @file    cabinet_executor.c
 * @brief   药柜动作执行器实现
 *
 * 把LLM返回的动作序列转成实际的机械臂运动
 */

#include "cabinet_executor.h"
#include "cabinet_state.h"
#include "cabinet_config.h"
#include "motion_controller.h"
#include "gripper.h"
#include "visual_servo.h"
#include "medicine_db.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

/***************************************************************
 * 配置参数
 ***************************************************************/
#define EXEC_SAFE_HEIGHT_Z      50.0f   /* 安全高度 (mm) */
#define EXEC_APPROACH_OFFSET    60.0f   /* 接近偏移 - 视觉伺服悬停高度 (mm) */
#define EXEC_MOTION_TIMEOUT_MS  10000   /* 运动超时 (ms) */
#define EXEC_GRIPPER_DELAY_MS   500     /* 夹爪动作延时 (ms) */
#define EXEC_ENABLE_VISUAL_SERVO 1      /* 启用视觉伺服 */

/***************************************************************
 * 全局变量
 ***************************************************************/
static exec_state_t g_exec_state = EXEC_IDLE;
static exec_speak_callback_t g_speak_callback = NULL;
static bool g_stop_requested = false;
static bool g_pause_requested = false;

/***************************************************************
 * 内部函数
 ***************************************************************/

/**
 * @brief 等待运动完成
 */
static int wait_motion_done(void)
{
    uint32_t start_tick = xTaskGetTickCount();

    while (1) {
        motion_state_t state = motion_get_state();

        if (state == MOTION_DONE || state == MOTION_IDLE) {
            return 0;
        }
        if (state == MOTION_ERROR) {
            return -1;
        }
        if (g_stop_requested) {
            motion_stop();
            return -2;
        }

        /* 超时检查 */
        if ((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS > EXEC_MOTION_TIMEOUT_MS) {
            motion_stop();
            return -3;
        }

        /* 暂停处理 */
        while (g_pause_requested && !g_stop_requested) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 移动到安全高度
 */
static int move_to_safe_height(float x, float y)
{
    int ret = motion_move_to_xyz(x, y, EXEC_SAFE_HEIGHT_Z);
    if (ret < 0) return ret;
    return wait_motion_done();
}

/**
 * @brief 执行抓取动作（带视觉伺服）
 * @param x, y, z 目标坐标
 * @param tag_id AprilTag ID（用于视觉伺服，-1表示不使用）
 */
static int do_grab_at_with_servo(float x, float y, float z, int tag_id)
{
    int ret;
    float servo_x = x;
    float servo_y = y;

    /* 1. 移动到目标上方（视觉伺服悬停高度） */
    ret = motion_move_to_xyz(x, y, z + EXEC_APPROACH_OFFSET);
    if (ret < 0) return ret;
    ret = wait_motion_done();
    if (ret < 0) return ret;

    /* 2. 视觉伺服微调 */
#if EXEC_ENABLE_VISUAL_SERVO
    if (tag_id >= 0) {
        printf("[EXEC] Visual servo for tag %d\n", tag_id);
        ret = visual_servo_align(tag_id, &servo_x, &servo_y);
        if (ret < 0) {
            printf("[EXEC] Visual servo failed, using original coords\n");
            /* 失败时继续使用原坐标 */
            servo_x = x;
            servo_y = y;
        }
    }
#endif

    /* 3. 张开夹爪 */
    gripper_open();
    vTaskDelay(pdMS_TO_TICKS(EXEC_GRIPPER_DELAY_MS));

    /* 4. 下降到抓取位置 */
    ret = motion_move_to_xyz(servo_x, servo_y, z);
    if (ret < 0) return ret;
    ret = wait_motion_done();
    if (ret < 0) return ret;

    /* 5. 闭合夹爪 */
    gripper_close();
    vTaskDelay(pdMS_TO_TICKS(EXEC_GRIPPER_DELAY_MS));

    /* 6. 抬起 */
    ret = motion_move_to_xyz(servo_x, servo_y, z + EXEC_APPROACH_OFFSET);
    if (ret < 0) return ret;
    ret = wait_motion_done();
    if (ret < 0) return ret;

    return 0;
}

/**
 * @brief 执行抓取动作（原版，无视觉伺服）
 */
static int do_grab_at(float x, float y, float z)
{
    return do_grab_at_with_servo(x, y, z, -1);
}

/**
 * @brief 执行放置动作
 */
static int do_place_at(float x, float y, float z)
{
    int ret;

    /* 1. 移动到目标上方 */
    ret = motion_move_to_xyz(x, y, z + EXEC_APPROACH_OFFSET);
    if (ret < 0) return ret;
    ret = wait_motion_done();
    if (ret < 0) return ret;

    /* 2. 下降到放置位置 */
    ret = motion_move_to_xyz(x, y, z);
    if (ret < 0) return ret;
    ret = wait_motion_done();
    if (ret < 0) return ret;

    /* 3. 张开夹爪 */
    gripper_open();
    vTaskDelay(pdMS_TO_TICKS(EXEC_GRIPPER_DELAY_MS));

    /* 4. 抬起 */
    ret = motion_move_to_xyz(x, y, z + EXEC_APPROACH_OFFSET);
    if (ret < 0) return ret;
    ret = wait_motion_done();
    if (ret < 0) return ret;

    return 0;
}

/***************************************************************
 * 动作执行函数
 ***************************************************************/

/**
 * @brief 执行 MOVE_SLOT 动作（带视觉伺服）
 */
static int exec_move_slot(const char *from, const char *to)
{
    int ret;
    slot_pos_t from_pos, to_pos;
    float from_x, from_y, from_z;
    float to_x, to_y, to_z;

    /* 解析槽位 */
    if (!cabinet_parse_slot(from, &from_pos)) {
        return -1;
    }
    if (!cabinet_parse_slot(to, &to_pos)) {
        return -1;
    }

    /* 获取坐标 */
    if (!cabinet_get_slot_coords(from_pos.row, from_pos.col, &from_x, &from_y, &from_z)) {
        return -2;
    }
    if (!cabinet_get_slot_coords(to_pos.row, to_pos.col, &to_x, &to_y, &to_z)) {
        return -2;
    }

    /* 检查源槽位是否有药品 */
    if (cabinet_is_slot_empty(from_pos.row, from_pos.col)) {
        return -3;  /* 源槽位为空 */
    }

    /* 检查目标槽位是否为空 */
    if (!cabinet_is_slot_empty(to_pos.row, to_pos.col)) {
        return -4;  /* 目标槽位非空 */
    }

    /* 获取药品的AprilTag ID（用于视觉伺服） */
    int tag_id = -1;
#if EXEC_ENABLE_VISUAL_SERVO
    cabinet_slot_t *slot = cabinet_get_slot(from_pos.row, from_pos.col);
    if (slot && slot->name[0] != '\0') {
        tag_id = medicine_db_find_id_by_name(slot->name);
    }
#endif

    /* 执行移动 */
    /* 1. 移动到安全高度 */
    ret = move_to_safe_height(from_x, from_y);
    if (ret < 0) return ret;

    /* 2. 抓取（带视觉伺服） */
    ret = do_grab_at_with_servo(from_x, from_y, from_z, tag_id);
    if (ret < 0) return ret;

    /* 3. 移动到目标上方 */
    ret = move_to_safe_height(to_x, to_y);
    if (ret < 0) return ret;

    /* 4. 放置（目标位置不需要视觉伺服，因为是空位） */
    ret = do_place_at(to_x, to_y, to_z);
    if (ret < 0) return ret;

    /* 5. 更新药柜状态 */
    cabinet_move_medicine(from_pos.row, from_pos.col, to_pos.row, to_pos.col);

    return 0;
}

/**
 * @brief 执行 FETCH 动作（带视觉伺服）
 */
static int exec_fetch(const char *slot)
{
    int ret;
    slot_pos_t pos;
    float x, y, z;

    /* 解析槽位 */
    if (!cabinet_parse_slot(slot, &pos)) {
        return -1;
    }

    /* 获取坐标 */
    if (!cabinet_get_slot_coords(pos.row, pos.col, &x, &y, &z)) {
        return -2;
    }

    /* 检查槽位是否有药品 */
    if (cabinet_is_slot_empty(pos.row, pos.col)) {
        return -3;
    }

    /* 获取药品的AprilTag ID（用于视觉伺服） */
    int tag_id = -1;
#if EXEC_ENABLE_VISUAL_SERVO
    cabinet_slot_t *slot_info = cabinet_get_slot(pos.row, pos.col);
    if (slot_info && slot_info->name[0] != '\0') {
        tag_id = medicine_db_find_id_by_name(slot_info->name);
    }
#endif

    /* 执行取药 */
    ret = move_to_safe_height(x, y);
    if (ret < 0) return ret;

    ret = do_grab_at_with_servo(x, y, z, tag_id);
    if (ret < 0) return ret;

    /* 移动到交付位置 (这里简化为原点上方) */
    ret = motion_move_to_xyz(0, 0, EXEC_SAFE_HEIGHT_Z);
    if (ret < 0) return ret;
    ret = wait_motion_done();
    if (ret < 0) return ret;

    /* 清空槽位状态 */
    cabinet_clear_slot(pos.row, pos.col);

    return 0;
}

/**
 * @brief 执行 SPEAK 动作
 */
static int exec_speak(const char *text)
{
    if (g_speak_callback) {
        g_speak_callback(text);
    }
    return 0;
}

/***************************************************************
 * 公共API
 ***************************************************************/

void cabinet_executor_init(void)
{
    g_exec_state = EXEC_IDLE;
    g_speak_callback = NULL;
    g_stop_requested = false;
    g_pause_requested = false;
}

void cabinet_executor_set_speak_callback(exec_speak_callback_t cb)
{
    g_speak_callback = cb;
}

int cabinet_executor_step(llm_action_t *action)
{
    if (!action) return -1;

    switch (action->type) {
    case ACTION_MOVE_SLOT:
        return exec_move_slot(action->params.move_slot.from,
                              action->params.move_slot.to);

    case ACTION_FETCH:
        return exec_fetch(action->params.fetch.slot);

    case ACTION_SPEAK:
        return exec_speak(action->params.speak.text);

    case ACTION_GRAB:
        gripper_close();
        vTaskDelay(pdMS_TO_TICKS(EXEC_GRIPPER_DELAY_MS));
        return 0;

    case ACTION_RELEASE:
        gripper_open();
        vTaskDelay(pdMS_TO_TICKS(EXEC_GRIPPER_DELAY_MS));
        return 0;

    default:
        return -1;
    }
}

int cabinet_executor_run(llm_action_sequence_t *seq)
{
    if (!seq || seq->count == 0) return -1;

    g_exec_state = EXEC_RUNNING;
    g_stop_requested = false;
    g_pause_requested = false;

    for (int i = 0; i < seq->count; i++) {
        if (g_stop_requested) {
            g_exec_state = EXEC_IDLE;
            return -2;
        }

        seq->current = i;
        int ret = cabinet_executor_step(&seq->actions[i]);

        if (ret < 0) {
            g_exec_state = EXEC_ERROR;
            return ret;
        }
    }

    g_exec_state = EXEC_DONE;
    return 0;
}

exec_state_t cabinet_executor_get_state(void)
{
    return g_exec_state;
}

void cabinet_executor_stop(void)
{
    g_stop_requested = true;
    motion_stop();
}

void cabinet_executor_pause(void)
{
    g_pause_requested = true;
}

void cabinet_executor_resume(void)
{
    g_pause_requested = false;
}

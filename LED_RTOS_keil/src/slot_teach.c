/**
 * @file    slot_teach.c
 * @brief   药柜格子示教再现系统实现
 */

#include "slot_teach.h"
#include "flash_storage.h"
#include "dummy_arm_cmd.h"
#include "motion_controller.h"
#include "gripper.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#define TAG "SLOT"

/* ========== 数据 ========== */

typedef struct {
    float joints[6];   /* 关节角(度): j0..j5 */
    bool  taught;
} SlotPose_t;

static SlotPose_t g_slots[SLOT_NUM];

/* ========== API 实现 ========== */

void slot_teach_init(void)
{
    memset(g_slots, 0, sizeof(g_slots));

    /* 尝试从 Flash 恢复上次示教数据 */
    if (flash_storage_load(g_slots, sizeof(g_slots))) {
        /* 统计已示教格子数 */
        int count = 0;
        for (int i = 0; i < SLOT_NUM; i++) {
            if (g_slots[i].taught) count++;
        }
        char buf[64];
        snprintf(buf, sizeof(buf), "[SLOT] Loaded %d slots from Flash", count);
        debug_println(buf);
    } else {
        debug_println("[SLOT] No saved data, starting fresh");
    }
}

bool slot_teach(uint8_t id)
{
    if (id >= SLOT_NUM) {
        debug_println("[SLOT] ID out of range (0-15)");
        return false;
    }

    /* 查询实际关节角（非缓存） */
    dummy_joint_pos_t jp;
    if (dummy_arm_get_joint_pos(&jp) != 0) {
        debug_println("[SLOT] Query failed (USB disconnected?)");
        return false;
    }

    for (int i = 0; i < 6; i++) {
        g_slots[id].joints[i] = jp.j[i];
    }
    g_slots[id].taught = true;

    /* 打印确认 */
    char buf[128];
    snprintf(buf, sizeof(buf),
             "[SLOT] Slot %d taught: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f",
             id,
             jp.j[0], jp.j[1], jp.j[2],
             jp.j[3], jp.j[4], jp.j[5]);
    debug_println(buf);

    /* 持久化到 Flash */
    if (flash_storage_save(g_slots, sizeof(g_slots))) {
        debug_println("[SLOT] Saved to Flash");
    } else {
        debug_println("[SLOT] Flash save failed!");
    }

    return true;
}

int slot_goto(uint8_t id)
{
    if (id >= SLOT_NUM) {
        debug_println("[SLOT] ID out of range (0-15)");
        return -1;
    }
    if (!g_slots[id].taught) {
        char buf[64];
        snprintf(buf, sizeof(buf), "[SLOT] Slot %d not taught yet", id);
        debug_println(buf);
        return -2;
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "[SLOT] Going to slot %d...", id);
    debug_println(buf);

    int ret = motion_move_to_joints(g_slots[id].joints);
    if (ret != 0) {
        snprintf(buf, sizeof(buf), "[SLOT] Motion failed: %d", ret);
        debug_println(buf);
    } else {
        debug_println("[SLOT] Arrived.");
    }
    return ret;
}

int slot_grab(uint8_t id)
{
    /* 1. 移动到格子 */
    int ret = slot_goto(id);
    if (ret != 0) {
        return ret;
    }

    /* 2. 开真空泵，等待吸附 */
    debug_println("[SLOT] Vacuum ON...");
    vacuum_on();
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 3. 向后退出50mm (Y方向) */
    debug_println("[SLOT] Retracting...");
    ret = motion_move_by_xyz(-50.0f, 0.0f, 0.0f);
    if (ret != 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "[SLOT] Retract failed: %d", ret);
        debug_println(buf);
    } else {
        debug_println("[SLOT] Grab done. Vacuum still ON.");
    }

    return ret;
}

int slot_fetch(uint8_t id)
{
    char buf[64];

    /* 1. 移动到药盒位置 */
    snprintf(buf, sizeof(buf), "[FETCH] Step 1/6: Going to slot %d...", id);
    debug_println(buf);
    int ret = slot_goto(id);
    if (ret != 0) return ret;

    /* 2. 开真空泵，等待吸附 */
    debug_println("[FETCH] Step 2/6: Vacuum ON...");
    vacuum_on();
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 3. 向后退出90mm，把药拉出药柜 */
    debug_println("[FETCH] Step 3/6: Retracting 90mm...");
    ret = motion_move_by_xyz(-SLOT_RETRACT_MM, 0.0f, 0.0f);
    if (ret != 0) {
        debug_println("[FETCH] Retract failed, stopping");
        vacuum_off();
        return ret;
    }

    /* 4. 移动到off点（放药位姿） */
    debug_println("[FETCH] Step 4/6: Going to OFF point...");
    ret = slot_goto(SLOT_OFF_ID);
    if (ret != 0) {
        debug_println("[FETCH] Go to OFF failed, stopping");
        vacuum_off();
        return ret;
    }

    /* 5. 关真空泵，释放药品 */
    debug_println("[FETCH] Step 5/6: Vacuum OFF, releasing...");
    vacuum_off();
    vTaskDelay(pdMS_TO_TICKS(300));

    /* 6. 回Home */
    debug_println("[FETCH] Step 6/6: Homing...");
    ret = dummy_arm_home();
    if (ret != 0) {
        debug_println("[FETCH] Home failed");
        return ret;
    }

    debug_println("[FETCH] Done!");
    return 0;
}

void slot_list(void)
{
    debug_println("[SLOT] Taught slots:");
    bool any = false;
    for (int i = 0; i < SLOT_NUM; i++) {
        if (g_slots[i].taught) {
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "  [%2d] %.1f, %.1f, %.1f, %.1f, %.1f, %.1f",
                     i,
                     g_slots[i].joints[0], g_slots[i].joints[1],
                     g_slots[i].joints[2], g_slots[i].joints[3],
                     g_slots[i].joints[4], g_slots[i].joints[5]);
            debug_println(buf);
            any = true;
        }
    }
    if (!any) {
        debug_println("  (none)");
    }
}

bool slot_is_taught(uint8_t id)
{
    if (id >= SLOT_NUM) return false;
    return g_slots[id].taught;
}

bool slot_get_joints(uint8_t id, float joints_out[6])
{
    if (id >= SLOT_NUM || !g_slots[id].taught) return false;
    for (int i = 0; i < 6; i++) {
        joints_out[i] = g_slots[id].joints[i];
    }
    return true;
}

void slot_clear(uint8_t id)
{
    if (id >= SLOT_NUM) {
        debug_println("[SLOT] ID out of range (0-15)");
        return;
    }
    g_slots[id].taught = false;
    memset(g_slots[id].joints, 0, sizeof(g_slots[id].joints));

    char buf[48];
    snprintf(buf, sizeof(buf), "[SLOT] Slot %d cleared", id);
    debug_println(buf);

    /* 持久化到 Flash */
    flash_storage_save(g_slots, sizeof(g_slots));
}

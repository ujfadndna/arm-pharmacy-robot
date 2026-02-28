/**
 * @file    cabinet_demo.c
 * @brief   智能药柜整理功能演示
 *
 * 演示流程:
 * 1. 初始化药品数据库和药柜状态
 * 2. 模拟接收MaixCam2的TAG消息
 * 3. 生成药柜状态字符串
 * 4. 调用W800发送给LLM
 * 5. 解析LLM返回的动作序列
 * 6. 执行动作
 */

#include "cabinet_state.h"
#include "cabinet_config.h"
#include "medicine_db.h"
#include "llm_action.h"
#include "w800_driver.h"
#include <stdio.h>
#include <string.h>

/* 演示用的TAG消息 (模拟MaixCam2发送) */
static const char *demo_tag_messages[] = {
    "TAG:0,150.5,350.2,-50.0",   /* 阿莫西林 */
    "TAG:1,230.0,350.5,-50.0",   /* 布洛芬 */
    "TAG:2,150.3,350.1,30.0",    /* 感冒灵 */
    "TAG:3,230.5,350.0,30.0",    /* 板蓝根 */
    NULL
};

/**
 * @brief 演示: 处理TAG消息更新药柜状态
 */
static void demo_process_tags(void)
{
    printf("\n=== 处理TAG消息 ===\n");

    /* 清空药柜状态 */
    cabinet_clear_all();

    /* 处理每条TAG消息 */
    for (int i = 0; demo_tag_messages[i] != NULL; i++) {
        printf("处理: %s\n", demo_tag_messages[i]);
        if (cabinet_parse_tag_message(demo_tag_messages[i])) {
            printf("  -> 成功\n");
        } else {
            printf("  -> 失败\n");
        }
    }
}

/**
 * @brief 演示: 生成药柜状态字符串
 */
static void demo_serialize_state(void)
{
    printf("\n=== 药柜状态 ===\n");

    char state_buf[512];
    int len = cabinet_serialize_state(state_buf, sizeof(state_buf));
    if (len > 0) {
        printf("%s\n", state_buf);
    }

    printf("\n=== 精简状态 ===\n");
    len = cabinet_serialize_state_compact(state_buf, sizeof(state_buf));
    if (len > 0) {
        printf("%s\n", state_buf);
    }
}

/**
 * @brief 演示: 调用LLM进行智能整理
 */
static void demo_llm_organize(void)
{
    printf("\n=== 调用LLM整理 ===\n");

    /* 生成状态字符串 */
    char state_buf[512];
    cabinet_serialize_state(state_buf, sizeof(state_buf));

    /* 用户指令 */
    const char *user_cmd = "把快过期的药放前面";
    printf("用户指令: %s\n", user_cmd);

    /* 调用W800 LLM */
    char response[512];
    int ret = w800_call_llm_organize(state_buf, user_cmd, response, sizeof(response));

    if (ret > 0) {
        printf("LLM响应: %s\n", response);

        /* 解析动作序列 */
        llm_action_sequence_t seq;
        if (llm_parse_json(response, &seq) == 0) {
            printf("\n解析到 %d 个动作:\n", seq.count);
            for (int i = 0; i < seq.count; i++) {
                llm_action_t *act = &seq.actions[i];
                printf("  [%d] %s", i, llm_action_type_str(act->type));

                switch (act->type) {
                case ACTION_MOVE_SLOT:
                    printf(" from=%s to=%s",
                           act->params.move_slot.from,
                           act->params.move_slot.to);
                    break;
                case ACTION_FETCH:
                    printf(" slot=%s", act->params.fetch.slot);
                    break;
                case ACTION_SPEAK:
                    printf(" text=\"%s\"", act->params.speak.text);
                    break;
                default:
                    break;
                }
                printf("\n");
            }
        }
    } else {
        printf("LLM调用失败: %d\n", ret);
    }
}

/**
 * @brief 演示: 执行动作序列
 */
static void demo_execute_actions(llm_action_sequence_t *seq)
{
    printf("\n=== 执行动作 ===\n");

    for (int i = 0; i < seq->count; i++) {
        llm_action_t *act = &seq->actions[i];
        printf("[%d] 执行 %s...\n", i, llm_action_type_str(act->type));

        switch (act->type) {
        case ACTION_MOVE_SLOT: {
            /* 移动药品 */
            slot_pos_t from_pos, to_pos;
            if (cabinet_parse_slot(act->params.move_slot.from, &from_pos) &&
                cabinet_parse_slot(act->params.move_slot.to, &to_pos)) {

                /* 获取源位置坐标 */
                float from_x, from_y, from_z;
                cabinet_get_slot_coords(from_pos.row, from_pos.col,
                                        &from_x, &from_y, &from_z);

                /* 获取目标位置坐标 */
                float to_x, to_y, to_z;
                cabinet_get_slot_coords(to_pos.row, to_pos.col,
                                        &to_x, &to_y, &to_z);

                printf("  移动: %s(%.1f,%.1f,%.1f) -> %s(%.1f,%.1f,%.1f)\n",
                       act->params.move_slot.from, from_x, from_y, from_z,
                       act->params.move_slot.to, to_x, to_y, to_z);

                /* TODO: 调用运动控制执行实际移动 */
                /* motion_move_to(from_x, from_y, from_z); */
                /* gripper_grab(); */
                /* motion_move_to(to_x, to_y, to_z); */
                /* gripper_release(); */

                /* 更新药柜状态 */
                cabinet_move_medicine(from_pos.row, from_pos.col,
                                      to_pos.row, to_pos.col);
            }
            break;
        }

        case ACTION_FETCH: {
            /* 取出药品 */
            slot_pos_t pos;
            if (cabinet_parse_slot(act->params.fetch.slot, &pos)) {
                float x, y, z;
                cabinet_get_slot_coords(pos.row, pos.col, &x, &y, &z);
                printf("  取出: %s(%.1f,%.1f,%.1f)\n",
                       act->params.fetch.slot, x, y, z);

                /* TODO: 执行取药动作 */
            }
            break;
        }

        case ACTION_SPEAK:
            printf("  播报: \"%s\"\n", act->params.speak.text);
            /* TODO: 调用TTS播报 */
            break;

        default:
            printf("  未知动作类型\n");
            break;
        }
    }
}

/**
 * @brief 智能药柜整理演示主函数
 */
void cabinet_demo_run(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  智能药柜整理功能演示\n");
    printf("========================================\n");

    /* 1. 初始化 */
    printf("\n[1] 初始化...\n");
    cabinet_config_init();
    cabinet_state_init();
    medicine_db_init();
    medicine_db_load_demo();
    printf("  药品数据库: %d 条\n", medicine_db_count());

    /* 2. 处理TAG消息 */
    demo_process_tags();

    /* 3. 显示药柜状态 */
    demo_serialize_state();

    /* 4. 调用LLM整理 (需要WiFi连接) */
    /* demo_llm_organize(); */

    /* 5. 模拟LLM响应并执行 */
    printf("\n[模拟] LLM响应解析...\n");
    const char *mock_response =
        "[{\"action\":\"move\",\"from\":\"A2\",\"to\":\"A3\"},"
        "{\"action\":\"move\",\"from\":\"A1\",\"to\":\"A2\"},"
        "{\"action\":\"move\",\"from\":\"B1\",\"to\":\"A1\"},"
        "{\"action\":\"speak\",\"text\":\"整理完成\"}]";

    llm_action_sequence_t seq;
    if (llm_parse_json(mock_response, &seq) == 0) {
        demo_execute_actions(&seq);
    }

    /* 6. 显示整理后的状态 */
    printf("\n[整理后] 药柜状态:\n");
    demo_serialize_state();

    printf("\n========================================\n");
    printf("  演示结束\n");
    printf("========================================\n");
}

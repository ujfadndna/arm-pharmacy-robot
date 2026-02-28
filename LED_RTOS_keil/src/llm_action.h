/**
 * @file    llm_action.h
 * @brief   LLM动作解析模块 - 数据结构定义
 */

#ifndef LLM_ACTION_H_
#define LLM_ACTION_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 动作类型 ========== */
typedef enum {
    ACTION_NONE = 0,
    ACTION_MOVE,        /* 移动到坐标 (target=药品名 或 x/y/z) */
    ACTION_MOVE_SLOT,   /* 移动药品 (from=Slot, to=Slot) - 智能整理用 */
    ACTION_FETCH,       /* 取出药品交给用户 (slot=Slot) */
    ACTION_GRAB,        /* 夹爪抓取 */
    ACTION_RELEASE,     /* 夹爪释放 */
    ACTION_SPEAK,       /* 语音播报 */
    ACTION_SCAN_QR      /* 扫描二维码 */
} action_type_t;

/* ========== 单个动作 ========== */
typedef struct {
    action_type_t type;
    union {
        struct {
            float x, y, z;      /* 目标位置 (mm) */
        } move;
        struct {
            char from[4];       /* 源Slot名称，如 "A1" */
            char to[4];         /* 目标Slot名称，如 "B2" */
        } move_slot;
        struct {
            char slot[4];       /* Slot名称，如 "A1" */
        } fetch;
        struct {
            char text[64];      /* 播报文本 */
        } speak;
    } params;
} llm_action_t;

/* ========== 动作序列 ========== */
#define MAX_ACTIONS 8

typedef struct {
    llm_action_t actions[MAX_ACTIONS];
    uint8_t count;              /* 动作总数 */
    uint8_t current;            /* 当前执行到第几个 */
} llm_action_sequence_t;

/* ========== API ========== */

/**
 * @brief 解析LLM返回的JSON动作序列
 * @param json JSON字符串
 * @param seq 输出的动作序列
 * @return 0=成功, -1=失败
 */
int llm_parse_json(const char *json, llm_action_sequence_t *seq);

/**
 * @brief 获取动作类型的字符串表示
 */
const char* llm_action_type_str(action_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* LLM_ACTION_H_ */

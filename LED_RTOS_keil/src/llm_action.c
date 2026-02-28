/**
 * @file    llm_action.c
 * @brief   LLM动作解析模块 - JSON解析实现
 */

#include "llm_action.h"
#include <string.h>
#include <stdlib.h>

/* ========== 药品位置表 ========== */
/* 在这里修改坐标，不需要重新编译W800 */
typedef struct {
    const char *name;       /* 药品名称 */
    float x, y, z;          /* 坐标 */
} medicine_location_t;

static const medicine_location_t g_medicine_table[] = {
    /* 根据scan结果，可达范围: x=-50~0, y=-50~50, z=-50 */
    {"阿莫西林",  -30.0f, -40.0f, -50.0f},
    {"布洛芬",    -30.0f,   0.0f, -50.0f},
    {"感冒灵",    -30.0f,  40.0f, -50.0f},
    /* 可以继续添加更多药品 */
    {NULL, 0, 0, 0}  /* 结束标记 */
};

/**
 * @brief 根据药品名称查找坐标
 * @return 0=找到, -1=未找到
 */
static int lookup_medicine(const char *name, float *x, float *y, float *z)
{
    for (int i = 0; g_medicine_table[i].name != NULL; i++) {
        if (strstr(name, g_medicine_table[i].name) != NULL) {
            *x = g_medicine_table[i].x;
            *y = g_medicine_table[i].y;
            *z = g_medicine_table[i].z;
            return 0;
        }
    }
    return -1;
}

/* ========== 内部辅助函数 ========== */

/**
 * @brief 从字符串中提取指定key后的浮点数
 * 例如: "\"x\": 120.5" 中提取 120.5
 */
static float parse_float_after(const char *str, const char *key)
{
    const char *p = strstr(str, key);
    if (!p) return 0.0f;

    p += strlen(key);

    /* 跳过 ": 和空格 */
    while (*p && (*p == '"' || *p == ':' || *p == ' ' || *p == '\t')) {
        p++;
    }

    return (float)atof(p);
}

/**
 * @brief 从字符串中提取指定key后的字符串值
 * 例如: "\"text\": \"hello\"" 中提取 hello
 */
static int parse_string_after(const char *str, const char *key,
                              char *out, int max_len)
{
    const char *p = strstr(str, key);
    if (!p) return -1;

    p += strlen(key);

    /* 找到冒号后的开始引号 */
    while (*p && *p != '"') p++;
    if (!*p) return -1;
    p++;  /* 跳过开始引号 */

    /* 复制到结束引号 */
    int i = 0;
    while (*p && *p != '"' && i < max_len - 1) {
        out[i++] = *p++;
    }
    out[i] = '\0';

    return i;
}

/**
 * @brief 解析单个动作对象 {...}
 */
static int parse_action_object(const char *obj_start, const char *obj_end,
                               llm_action_t *action)
{
    int len = (int)(obj_end - obj_start);
    if (len <= 0 || len > 256) return -1;

    /* 创建临时缓冲区 */
    char buf[257];
    memcpy(buf, obj_start, (size_t)len);
    buf[len] = '\0';

    /* 初始化 */
    memset(action, 0, sizeof(*action));

    /* 检测动作类型 */
    if (strstr(buf, "\"move\"")) {
        /* 检查是否有 from/to 字段（智能整理模式） */
        char from_slot[8] = {0};
        char to_slot[8] = {0};

        if (parse_string_after(buf, "\"from\"", from_slot, sizeof(from_slot)) > 0 &&
            parse_string_after(buf, "\"to\"", to_slot, sizeof(to_slot)) > 0) {
            /* move(from, to) 格式 - 智能整理 */
            action->type = ACTION_MOVE_SLOT;
            strncpy(action->params.move_slot.from, from_slot, 3);
            action->params.move_slot.from[3] = '\0';
            strncpy(action->params.move_slot.to, to_slot, 3);
            action->params.move_slot.to[3] = '\0';
        } else {
            /* 传统 move 格式 */
            action->type = ACTION_MOVE;

            /* 优先检查target字段（药品名称） */
            char target_name[32] = {0};
            if (parse_string_after(buf, "\"target\"", target_name, sizeof(target_name)) > 0) {
                /* 有target字段，查表获取坐标 */
                if (lookup_medicine(target_name,
                                   &action->params.move.x,
                                   &action->params.move.y,
                                   &action->params.move.z) != 0) {
                    /* 未找到药品，使用默认坐标 */
                    action->params.move.x = 0;
                    action->params.move.y = 0;
                    action->params.move.z = -50;
                }
            } else {
                /* 没有target字段，使用x/y/z坐标（兼容旧格式） */
                action->params.move.x = parse_float_after(buf, "\"x\"");
                action->params.move.y = parse_float_after(buf, "\"y\"");
                action->params.move.z = parse_float_after(buf, "\"z\"");
            }
        }
    }
    else if (strstr(buf, "\"fetch\"")) {
        /* fetch(slot) - 取出药品交给用户 */
        action->type = ACTION_FETCH;
        char slot_name[8] = {0};
        if (parse_string_after(buf, "\"slot\"", slot_name, sizeof(slot_name)) > 0) {
            strncpy(action->params.fetch.slot, slot_name, 3);
            action->params.fetch.slot[3] = '\0';
        }
    }
    else if (strstr(buf, "\"grab\"")) {
        action->type = ACTION_GRAB;
    }
    else if (strstr(buf, "\"release\"")) {
        action->type = ACTION_RELEASE;
    }
    else if (strstr(buf, "\"speak\"")) {
        action->type = ACTION_SPEAK;
        parse_string_after(buf, "\"text\"",
                          action->params.speak.text,
                          sizeof(action->params.speak.text));
    }
    else if (strstr(buf, "\"scan_qr\"")) {
        action->type = ACTION_SCAN_QR;
    }
    else {
        action->type = ACTION_NONE;
        return -1;
    }

    return 0;
}

/* ========== 公共API ========== */

int llm_parse_json(const char *json, llm_action_sequence_t *seq)
{
    if (!json || !seq) return -1;

    /* 初始化输出结构 */
    memset(seq, 0, sizeof(*seq));

    const char *p = json;

    /* 遍历查找所有 {...} 对象 */
    while (*p && seq->count < MAX_ACTIONS) {
        /* 找到对象开始 '{' */
        const char *obj_start = strchr(p, '{');
        if (!obj_start) break;

        /* 找到对象结束 '}' */
        const char *obj_end = strchr(obj_start, '}');
        if (!obj_end) break;

        /* 解析这个动作对象 */
        if (parse_action_object(obj_start, obj_end + 1,
                               &seq->actions[seq->count]) == 0) {
            seq->count++;
        }

        /* 继续下一个 */
        p = obj_end + 1;
    }

    return seq->count > 0 ? 0 : -1;
}

const char* llm_action_type_str(action_type_t type)
{
    switch (type) {
        case ACTION_MOVE:      return "MOVE";
        case ACTION_MOVE_SLOT: return "MOVE_SLOT";
        case ACTION_FETCH:     return "FETCH";
        case ACTION_GRAB:      return "GRAB";
        case ACTION_RELEASE:   return "RELEASE";
        case ACTION_SPEAK:     return "SPEAK";
        case ACTION_SCAN_QR:   return "SCAN_QR";
        default:               return "UNKNOWN";
    }
}

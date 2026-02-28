/***************************************************************
 * cabinet_state.c - 药柜状态管理实现
 ***************************************************************/

#include "cabinet_state.h"
#include "cabinet_config.h"
#include "medicine_db.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/***************************************************************
 * 全局变量
 ***************************************************************/
static cabinet_slot_t g_cabinet[CABINET_STATE_ROWS][CABINET_STATE_COLS];

/***************************************************************
 * 内部函数
 ***************************************************************/

/**
 * @brief 验证行列索引是否有效
 */
static bool is_valid_pos(uint8_t row, uint8_t col)
{
    return (row < CABINET_STATE_ROWS && col < CABINET_STATE_COLS);
}

/***************************************************************
 * API 实现
 ***************************************************************/

void cabinet_state_init(void)
{
    /* 清空所有Slot */
    memset(g_cabinet, 0, sizeof(g_cabinet));

    /* 计算所有Slot的物理坐标 */
    cabinet_calculate_all_coords();
}

bool cabinet_parse_slot(const char *slot_name, slot_pos_t *pos)
{
    if (!slot_name || !pos) return false;

    /* 跳过可能的空格 */
    while (*slot_name == ' ') slot_name++;

    /* 解析行: A-D -> 0-3 */
    char row_char = slot_name[0];
    if (row_char >= 'A' && row_char <= 'D') {
        pos->row = (uint8_t)(row_char - 'A');
    } else if (row_char >= 'a' && row_char <= 'd') {
        pos->row = (uint8_t)(row_char - 'a');
    } else {
        return false;
    }

    /* 解析列: 1-4 -> 0-3 */
    char col_char = slot_name[1];
    if (col_char >= '1' && col_char <= '4') {
        pos->col = (uint8_t)(col_char - '1');
    } else {
        return false;
    }

    return true;
}

cabinet_slot_t* cabinet_get_slot(uint8_t row, uint8_t col)
{
    if (!is_valid_pos(row, col)) return NULL;
    return &g_cabinet[row][col];
}

cabinet_slot_t* cabinet_get_slot_by_name(const char *slot_name)
{
    slot_pos_t pos;
    if (!cabinet_parse_slot(slot_name, &pos)) return NULL;
    return cabinet_get_slot(pos.row, pos.col);
}

bool cabinet_is_slot_empty(uint8_t row, uint8_t col)
{
    if (!is_valid_pos(row, col)) return true;
    return (g_cabinet[row][col].name[0] == '\0');
}

bool cabinet_set_slot(uint8_t row, uint8_t col, const char *name, const char *expiry)
{
    if (!is_valid_pos(row, col)) return false;

    cabinet_slot_t *slot = &g_cabinet[row][col];

    /* 设置药品名称 */
    if (name && name[0] != '\0') {
        strncpy(slot->name, name, MEDICINE_NAME_MAX - 1);
        slot->name[MEDICINE_NAME_MAX - 1] = '\0';
    } else {
        slot->name[0] = '\0';
    }

    /* 设置有效期 */
    if (expiry && expiry[0] != '\0') {
        strncpy(slot->expiry, expiry, EXPIRY_DATE_MAX - 1);
        slot->expiry[EXPIRY_DATE_MAX - 1] = '\0';
    } else {
        slot->expiry[0] = '\0';
    }

    return true;
}

bool cabinet_clear_slot(uint8_t row, uint8_t col)
{
    if (!is_valid_pos(row, col)) return false;

    cabinet_slot_t *slot = &g_cabinet[row][col];
    slot->name[0] = '\0';
    slot->expiry[0] = '\0';
    /* 保留坐标不变 */

    return true;
}

bool cabinet_move_medicine(uint8_t from_row, uint8_t from_col,
                           uint8_t to_row, uint8_t to_col)
{
    if (!is_valid_pos(from_row, from_col) || !is_valid_pos(to_row, to_col)) {
        return false;
    }

    /* 检查源Slot是否有药品 */
    if (cabinet_is_slot_empty(from_row, from_col)) {
        return false;
    }

    /* 检查目标Slot是否为空 */
    if (!cabinet_is_slot_empty(to_row, to_col)) {
        return false;
    }

    cabinet_slot_t *from = &g_cabinet[from_row][from_col];
    cabinet_slot_t *to = &g_cabinet[to_row][to_col];

    /* 复制药品信息（不复制坐标） */
    strncpy(to->name, from->name, MEDICINE_NAME_MAX);
    strncpy(to->expiry, from->expiry, EXPIRY_DATE_MAX);

    /* 清空源Slot */
    from->name[0] = '\0';
    from->expiry[0] = '\0';

    return true;
}

int cabinet_serialize_state(char *buf, int buf_size)
{
    if (!buf || buf_size < 64) return -1;

    int offset = 0;
    const char row_names[] = "ABCD";

    for (uint8_t row = 0; row < CABINET_STATE_ROWS; row++) {
        for (uint8_t col = 0; col < CABINET_STATE_COLS; col++) {
            cabinet_slot_t *slot = &g_cabinet[row][col];

            int written;
            if (slot->name[0] != '\0') {
                /* 有药品 */
                if (slot->expiry[0] != '\0') {
                    written = snprintf(buf + offset, buf_size - offset,
                                       "%c%d:%s(%s) ",
                                       row_names[row], col + 1,
                                       slot->name, slot->expiry);
                } else {
                    written = snprintf(buf + offset, buf_size - offset,
                                       "%c%d:%s ",
                                       row_names[row], col + 1, slot->name);
                }
            } else {
                /* 空位 */
                written = snprintf(buf + offset, buf_size - offset,
                                   "%c%d:空 ",
                                   row_names[row], col + 1);
            }

            if (written < 0 || offset + written >= buf_size) {
                return -1;
            }
            offset += written;
        }

        /* 每行结束换行 */
        if (offset < buf_size - 1) {
            buf[offset++] = '\n';
        }
    }

    /* 去掉最后的换行，添加结束符 */
    if (offset > 0 && buf[offset - 1] == '\n') {
        offset--;
    }
    buf[offset] = '\0';

    return offset;
}

int cabinet_serialize_state_compact(char *buf, int buf_size)
{
    if (!buf || buf_size < 32) return -1;

    int offset = 0;
    const char row_names[] = "ABCD";

    for (uint8_t row = 0; row < CABINET_STATE_ROWS; row++) {
        for (uint8_t col = 0; col < CABINET_STATE_COLS; col++) {
            cabinet_slot_t *slot = &g_cabinet[row][col];

            /* 只输出非空Slot */
            if (slot->name[0] != '\0') {
                int written;
                if (slot->expiry[0] != '\0') {
                    written = snprintf(buf + offset, buf_size - offset,
                                       "%c%d:%s(%s) ",
                                       row_names[row], col + 1,
                                       slot->name, slot->expiry);
                } else {
                    written = snprintf(buf + offset, buf_size - offset,
                                       "%c%d:%s ",
                                       row_names[row], col + 1, slot->name);
                }

                if (written < 0 || offset + written >= buf_size) {
                    return -1;
                }
                offset += written;
            }
        }
    }

    /* 去掉最后的空格 */
    if (offset > 0 && buf[offset - 1] == ' ') {
        offset--;
    }
    buf[offset] = '\0';

    return offset;
}

bool cabinet_get_slot_coords(uint8_t row, uint8_t col, float *x, float *y, float *z)
{
    if (!is_valid_pos(row, col) || !x || !y || !z) return false;

    cabinet_slot_t *slot = &g_cabinet[row][col];
    *x = slot->x;
    *y = slot->y;
    *z = slot->z;

    return true;
}

bool cabinet_set_slot_coords(uint8_t row, uint8_t col, float x, float y, float z)
{
    if (!is_valid_pos(row, col)) return false;

    cabinet_slot_t *slot = &g_cabinet[row][col];
    slot->x = x;
    slot->y = y;
    slot->z = z;

    return true;
}

void cabinet_calculate_all_coords(void)
{
    /* 使用 cabinet_config 中的配置计算坐标 */
    for (uint8_t row = 0; row < CABINET_STATE_ROWS; row++) {
        for (uint8_t col = 0; col < CABINET_STATE_COLS; col++) {
            cabinet_slot_t *slot = &g_cabinet[row][col];

            /* 基于原点和格子尺寸计算 */
            slot->x = g_cabinet_config.origin_x + col * g_cabinet_config.cell_width;
            slot->y = g_cabinet_config.origin_y - g_cabinet_config.cell_depth / 2;
            slot->z = g_cabinet_config.origin_z - row * g_cabinet_config.cell_height;
        }
    }
}

void cabinet_state_print(void)
{
    /* 需要串口打印支持，这里只是占位 */
    /* TODO: 实现串口打印
     *
     * printf("药柜状态 (4x4):\n");
     * for (int row = 0; row < 4; row++) {
     *     printf("  %c: ", 'A' + row);
     *     for (int col = 0; col < 4; col++) {
     *         cabinet_slot_t *slot = &g_cabinet[row][col];
     *         if (slot->name[0]) {
     *             printf("[%s] ", slot->name);
     *         } else {
     *             printf("[空] ");
     *         }
     *     }
     *     printf("\n");
     * }
     */
}

void cabinet_load_demo_data(void)
{
    /* 加载演示数据 */
    cabinet_set_slot(0, 0, "阿莫西林", "2026-02-15");   /* A1 */
    cabinet_set_slot(0, 1, "布洛芬", "2027-01-01");     /* A2 */
    cabinet_set_slot(1, 0, "感冒灵", "2026-03-01");     /* B1 */
    cabinet_set_slot(1, 1, "板蓝根", "2026-12-01");     /* B2 */
    /* 其他Slot保持为空 */
}

/***************************************************************
 * AprilTag消息处理
 ***************************************************************/

bool cabinet_find_slot_by_coord(float x, float y, float z,
                                 uint8_t *row, uint8_t *col)
{
    if (!row || !col) return false;

    /* 计算最近的槽位 */
    float min_dist = 1e9f;
    int best_row = -1, best_col = -1;

    for (uint8_t r = 0; r < CABINET_STATE_ROWS; r++) {
        for (uint8_t c = 0; c < CABINET_STATE_COLS; c++) {
            cabinet_slot_t *slot = &g_cabinet[r][c];

            /* 计算距离 */
            float dx = x - slot->x;
            float dy = y - slot->y;
            float dz = z - slot->z;
            float dist = sqrtf(dx*dx + dy*dy + dz*dz);

            if (dist < min_dist) {
                min_dist = dist;
                best_row = r;
                best_col = c;
            }
        }
    }

    /* 检查距离是否在合理范围内 (半个格子尺寸) */
    float threshold = g_cabinet_config.cell_width / 2.0f + 10.0f;  /* 加10mm容差 */
    if (min_dist > threshold) {
        return false;  /* 坐标超出药柜范围 */
    }

    *row = (uint8_t)best_row;
    *col = (uint8_t)best_col;
    return true;
}

bool cabinet_parse_tag_message(const char *msg)
{
    if (!msg) return false;

    /* 检查前缀 "TAG:" */
    if (strncmp(msg, "TAG:", 4) != 0) return false;

    /* 解析: TAG:<id>,<x>,<y>,<z> */
    int id;
    float x, y, z;
    if (sscanf(msg + 4, "%d,%f,%f,%f", &id, &x, &y, &z) != 4) {
        return false;
    }

    /* 查询药品数据库 */
    const medicine_info_t *med = medicine_db_lookup((uint8_t)id);
    if (!med) {
        /* 未知药品ID，跳过 */
        return false;
    }

    /* 根据坐标确定槽位 */
    uint8_t row, col;
    if (!cabinet_find_slot_by_coord(x, y, z, &row, &col)) {
        /* 坐标超出范围 */
        return false;
    }

    /* 更新药柜状态 */
    cabinet_slot_t *slot = &g_cabinet[row][col];
    strncpy(slot->name, med->name, MEDICINE_NAME_MAX - 1);
    slot->name[MEDICINE_NAME_MAX - 1] = '\0';
    strncpy(slot->expiry, med->expiry, EXPIRY_DATE_MAX - 1);
    slot->expiry[EXPIRY_DATE_MAX - 1] = '\0';

    /* 更新实际检测到的坐标 */
    slot->x = x;
    slot->y = y;
    slot->z = z;

    return true;
}

void cabinet_clear_all(void)
{
    for (uint8_t row = 0; row < CABINET_STATE_ROWS; row++) {
        for (uint8_t col = 0; col < CABINET_STATE_COLS; col++) {
            g_cabinet[row][col].name[0] = '\0';
            g_cabinet[row][col].expiry[0] = '\0';
            /* 保留坐标不变 */
        }
    }
}

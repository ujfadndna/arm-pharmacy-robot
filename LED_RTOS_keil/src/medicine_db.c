/**
 * @file    medicine_db.c
 * @brief   药品数据库实现
 */

#include "medicine_db.h"
#include <string.h>
#include <stdio.h>

/***************************************************************
 * 全局变量
 ***************************************************************/
static medicine_info_t g_medicine_db[MEDICINE_DB_MAX_ENTRIES];
static int g_db_count = 0;

/***************************************************************
 * 内部函数
 ***************************************************************/

/**
 * @brief 查找条目索引
 * @return 索引，-1表示未找到
 */
static int find_entry_index(uint8_t tag_id)
{
    for (int i = 0; i < MEDICINE_DB_MAX_ENTRIES; i++) {
        if (g_medicine_db[i].valid && g_medicine_db[i].tag_id == tag_id) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 查找空闲槽位
 * @return 索引，-1表示已满
 */
static int find_free_slot(void)
{
    for (int i = 0; i < MEDICINE_DB_MAX_ENTRIES; i++) {
        if (!g_medicine_db[i].valid) {
            return i;
        }
    }
    return -1;
}

/***************************************************************
 * API 实现
 ***************************************************************/

void medicine_db_init(void)
{
    memset(g_medicine_db, 0, sizeof(g_medicine_db));
    g_db_count = 0;
}

const medicine_info_t* medicine_db_lookup(uint8_t tag_id)
{
    int idx = find_entry_index(tag_id);
    if (idx >= 0) {
        return &g_medicine_db[idx];
    }
    return NULL;
}

bool medicine_db_add(uint8_t tag_id, const char *name, const char *expiry)
{
    if (!name || name[0] == '\0') return false;

    /* 先查找是否已存在 */
    int idx = find_entry_index(tag_id);
    if (idx < 0) {
        /* 不存在，找空闲槽位 */
        idx = find_free_slot();
        if (idx < 0) return false;  /* 数据库已满 */
        g_db_count++;
    }

    medicine_info_t *entry = &g_medicine_db[idx];
    entry->tag_id = tag_id;
    entry->valid = true;

    strncpy(entry->name, name, MEDICINE_NAME_LEN - 1);
    entry->name[MEDICINE_NAME_LEN - 1] = '\0';

    if (expiry && expiry[0] != '\0') {
        strncpy(entry->expiry, expiry, MEDICINE_EXPIRY_LEN - 1);
        entry->expiry[MEDICINE_EXPIRY_LEN - 1] = '\0';
    } else {
        entry->expiry[0] = '\0';
    }

    return true;
}

bool medicine_db_remove(uint8_t tag_id)
{
    int idx = find_entry_index(tag_id);
    if (idx < 0) return false;

    g_medicine_db[idx].valid = false;
    g_db_count--;
    return true;
}

int medicine_db_find_id_by_name(const char *name)
{
    if (!name || name[0] == '\0') return -1;

    for (int i = 0; i < MEDICINE_DB_MAX_ENTRIES; i++) {
        if (g_medicine_db[i].valid &&
            strcmp(g_medicine_db[i].name, name) == 0) {
            return g_medicine_db[i].tag_id;
        }
    }
    return -1;
}

int medicine_db_count(void)
{
    return g_db_count;
}

void medicine_db_clear(void)
{
    memset(g_medicine_db, 0, sizeof(g_medicine_db));
    g_db_count = 0;
}

void medicine_db_load_demo(void)
{
    /* 清空现有数据 */
    medicine_db_clear();

    /* 加载演示数据 - AprilTag ID对应药品 */
    /* ID分配方案: ID = 药品编号 */
    medicine_db_add(0, "阿莫西林", "2026-02-15");
    medicine_db_add(1, "布洛芬", "2027-01-01");
    medicine_db_add(2, "感冒灵", "2026-03-01");
    medicine_db_add(3, "板蓝根", "2026-12-01");
    medicine_db_add(4, "维生素C", "2027-06-15");
    medicine_db_add(5, "头孢克肟", "2026-08-20");
    medicine_db_add(6, "藿香正气", "2026-05-10");
    medicine_db_add(7, "连花清瘟", "2026-09-30");
}

void medicine_db_print(void)
{
    /* 需要串口打印支持 */
    /* TODO: 实现串口打印
     *
     * printf("药品数据库 (%d条):\n", g_db_count);
     * for (int i = 0; i < MEDICINE_DB_MAX_ENTRIES; i++) {
     *     if (g_medicine_db[i].valid) {
     *         printf("  ID=%d: %s (%s)\n",
     *                g_medicine_db[i].tag_id,
     *                g_medicine_db[i].name,
     *                g_medicine_db[i].expiry);
     *     }
     * }
     */
}

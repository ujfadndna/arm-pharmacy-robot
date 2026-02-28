/**
 * @file    medicine_db.h
 * @brief   药品数据库 - AprilTag ID → 药品信息映射
 *
 * 设计说明:
 *   - AprilTag贴在药盒侧面，ID对应药品编号
 *   - 药品信息存在RA6M5数据库里，不存在AprilTag中
 *   - 支持运行时添加/更新药品信息
 */

#ifndef MEDICINE_DB_H_
#define MEDICINE_DB_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************
 * 常量定义
 ***************************************************************/
#define MEDICINE_DB_MAX_ENTRIES     32      /* 最大药品数量 */
#define MEDICINE_NAME_LEN           16      /* 药品名称最大长度 */
#define MEDICINE_EXPIRY_LEN         12      /* 有效期字符串长度 YYYY-MM-DD */

/***************************************************************
 * 药品信息结构
 ***************************************************************/
typedef struct {
    uint8_t tag_id;                         /* AprilTag ID (0-255) */
    char name[MEDICINE_NAME_LEN];           /* 药品名称 */
    char expiry[MEDICINE_EXPIRY_LEN];       /* 有效期 YYYY-MM-DD */
    bool valid;                             /* 条目是否有效 */
} medicine_info_t;

/***************************************************************
 * API 函数
 ***************************************************************/

/**
 * @brief 初始化药品数据库（加载默认数据）
 */
void medicine_db_init(void);

/**
 * @brief 根据AprilTag ID查找药品信息
 * @param tag_id AprilTag ID
 * @return 药品信息指针，NULL表示未找到
 */
const medicine_info_t* medicine_db_lookup(uint8_t tag_id);

/**
 * @brief 添加或更新药品信息
 * @param tag_id AprilTag ID
 * @param name 药品名称
 * @param expiry 有效期 (可为NULL)
 * @return true=成功, false=数据库已满
 */
bool medicine_db_add(uint8_t tag_id, const char *name, const char *expiry);

/**
 * @brief 删除药品信息
 * @param tag_id AprilTag ID
 * @return true=成功, false=未找到
 */
bool medicine_db_remove(uint8_t tag_id);

/**
 * @brief 根据药品名称查找AprilTag ID
 * @param name 药品名称
 * @return AprilTag ID，-1表示未找到
 */
int medicine_db_find_id_by_name(const char *name);

/**
 * @brief 获取数据库中的药品数量
 * @return 有效条目数量
 */
int medicine_db_count(void);

/**
 * @brief 清空数据库
 */
void medicine_db_clear(void);

/**
 * @brief 加载演示数据
 */
void medicine_db_load_demo(void);

/**
 * @brief 打印数据库内容（调试用）
 */
void medicine_db_print(void);

#ifdef __cplusplus
}
#endif

#endif /* MEDICINE_DB_H_ */

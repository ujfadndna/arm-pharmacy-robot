/***************************************************************
 * cabinet_state.h - 药柜状态管理（4x4格）
 *
 * 药柜布局 (4列 x 4行 = 16格):
 *
 *     列1   列2   列3   列4
 *    +----+----+----+----+
 * A  | A1 | A2 | A3 | A4 |  <- 顶层
 *    +----+----+----+----+
 * B  | B1 | B2 | B3 | B4 |
 *    +----+----+----+----+
 * C  | C1 | C2 | C3 | C4 |
 *    +----+----+----+----+
 * D  | D1 | D2 | D3 | D4 |  <- 底层
 *    +----+----+----+----+
 *
 * Slot命名: 行(A-D) + 列(1-4)
 * 内部索引: [row][col] = [0-3][0-3]
 ***************************************************************/

#ifndef CABINET_STATE_H_
#define CABINET_STATE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************
 * 常量定义
 ***************************************************************/
#define CABINET_STATE_ROWS      4       /* 行数 A-D */
#define CABINET_STATE_COLS      4       /* 列数 1-4 */
#define CABINET_STATE_TOTAL     (CABINET_STATE_ROWS * CABINET_STATE_COLS)  /* 16格 */

#define MEDICINE_NAME_MAX       16      /* 药品名称最大长度 */
#define EXPIRY_DATE_MAX         12      /* 有效期字符串长度 YYYY-MM-DD */

/***************************************************************
 * 单个药柜格子状态
 ***************************************************************/
typedef struct {
    char name[MEDICINE_NAME_MAX];       /* 药品名称，空字符串表示空位 */
    char expiry[EXPIRY_DATE_MAX];       /* 有效期 YYYY-MM-DD */
    float x, y, z;                      /* 物理坐标 (mm) */
} cabinet_slot_t;

/***************************************************************
 * Slot位置结构
 ***************************************************************/
typedef struct {
    uint8_t row;    /* 0-3 对应 A-D */
    uint8_t col;    /* 0-3 对应 1-4 */
} slot_pos_t;

/***************************************************************
 * API 函数
 ***************************************************************/

/**
 * @brief 初始化药柜状态（加载默认值或从Flash恢复）
 */
void cabinet_state_init(void);

/**
 * @brief 解析Slot名称（如"A1"）为行列索引
 * @param slot_name Slot名称，如 "A1", "B2", "D4"
 * @param pos 输出: 行列位置
 * @return true=成功, false=无效名称
 */
bool cabinet_parse_slot(const char *slot_name, slot_pos_t *pos);

/**
 * @brief 获取指定Slot的状态
 * @param row 行 (0-3)
 * @param col 列 (0-3)
 * @return Slot指针，NULL表示无效
 */
cabinet_slot_t* cabinet_get_slot(uint8_t row, uint8_t col);

/**
 * @brief 根据Slot名称获取状态
 * @param slot_name Slot名称，如 "A1"
 * @return Slot指针，NULL表示无效
 */
cabinet_slot_t* cabinet_get_slot_by_name(const char *slot_name);

/**
 * @brief 检查Slot是否为空
 * @param row 行 (0-3)
 * @param col 列 (0-3)
 * @return true=空位
 */
bool cabinet_is_slot_empty(uint8_t row, uint8_t col);

/**
 * @brief 设置Slot内容
 * @param row 行 (0-3)
 * @param col 列 (0-3)
 * @param name 药品名称（NULL或空字符串表示清空）
 * @param expiry 有效期（可为NULL）
 * @return true=成功
 */
bool cabinet_set_slot(uint8_t row, uint8_t col, const char *name, const char *expiry);

/**
 * @brief 清空指定Slot
 * @param row 行 (0-3)
 * @param col 列 (0-3)
 * @return true=成功
 */
bool cabinet_clear_slot(uint8_t row, uint8_t col);

/**
 * @brief 移动药品（从一个Slot到另一个Slot）
 * @param from_row 源行
 * @param from_col 源列
 * @param to_row 目标行
 * @param to_col 目标列
 * @return true=成功, false=源为空或目标非空
 *
 * 注意: 此函数只更新状态，不执行物理移动
 */
bool cabinet_move_medicine(uint8_t from_row, uint8_t from_col,
                           uint8_t to_row, uint8_t to_col);

/**
 * @brief 生成药柜状态字符串（用于发送给LLM）
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @return 实际写入长度，-1表示失败
 *
 * 输出格式示例:
 * A1:阿莫西林(2026-02-15) A2:布洛芬(2027-01-01) A3:空 A4:空
 * B1:感冒灵(2026-03-01) B2:板蓝根(2026-12-01) B3:空 B4:空
 * C1:空 C2:空 C3:空 C4:空
 * D1:空 D2:空 D3:空 D4:空
 */
int cabinet_serialize_state(char *buf, int buf_size);

/**
 * @brief 生成精简状态字符串（只包含非空Slot）
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @return 实际写入长度，-1表示失败
 *
 * 输出格式示例:
 * A1:阿莫西林(2026-02-15) A2:布洛芬(2027-01-01) B1:感冒灵(2026-03-01) B2:板蓝根(2026-12-01)
 */
int cabinet_serialize_state_compact(char *buf, int buf_size);

/**
 * @brief 获取Slot的物理坐标
 * @param row 行 (0-3)
 * @param col 列 (0-3)
 * @param x, y, z 输出: 坐标
 * @return true=成功
 */
bool cabinet_get_slot_coords(uint8_t row, uint8_t col, float *x, float *y, float *z);

/**
 * @brief 设置Slot的物理坐标（标定用）
 * @param row 行 (0-3)
 * @param col 列 (0-3)
 * @param x, y, z 坐标
 * @return true=成功
 */
bool cabinet_set_slot_coords(uint8_t row, uint8_t col, float x, float y, float z);

/**
 * @brief 根据药柜配置自动计算所有Slot坐标
 *
 * 使用 cabinet_config.h 中的原点和格子尺寸计算
 */
void cabinet_calculate_all_coords(void);

/**
 * @brief 打印药柜状态（调试用）
 */
void cabinet_state_print(void);

/**
 * @brief 加载测试数据（演示用）
 */
void cabinet_load_demo_data(void);

/***************************************************************
 * AprilTag消息处理 (与MaixCam2通信)
 ***************************************************************/

/**
 * @brief 解析单条TAG消息并更新药柜状态
 * @param msg TAG消息，格式: "TAG:<id>,<x>,<y>,<z>"
 * @return true=成功, false=解析失败
 *
 * 处理流程:
 * 1. 解析TAG消息获取 id, x, y, z
 * 2. 根据id查询药品数据库获取名称和有效期
 * 3. 根据坐标确定所在槽位
 * 4. 更新药柜状态
 */
bool cabinet_parse_tag_message(const char *msg);

/**
 * @brief 根据坐标查找最近的槽位
 * @param x, y, z 坐标 (mm)
 * @param row, col 输出: 槽位行列
 * @return true=找到, false=坐标超出范围
 */
bool cabinet_find_slot_by_coord(float x, float y, float z,
                                 uint8_t *row, uint8_t *col);

/**
 * @brief 清空所有槽位（准备重新扫描）
 */
void cabinet_clear_all(void);

#ifdef __cplusplus
}
#endif

#endif /* CABINET_STATE_H_ */

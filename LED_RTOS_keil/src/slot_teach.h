/**
 * @file    slot_teach.h
 * @brief   药柜格子示教再现系统
 *
 * 使用方法:
 *   示教: slot teach <id>   —— 记录当前关节角为格子id的位置
 *   移动: slot goto  <id>   —— 回放到格子id (不操作夹具)
 *   取药: slot grab  <id>   —— 回放 → 开真空泵 → 退出
 *   查看: slot list         —— 打印所有已示教格子
 *   清除: slot clear <id>   —— 清除格子id
 */

#ifndef SLOT_TEACH_H_
#define SLOT_TEACH_H_

#include <stdint.h>
#include <stdbool.h>

#define SLOT_NUM         16      /* 最多支持16个格子 */
#define SLOT_OFF_ID      6       /* 放药点(off点)的槽位ID，用 slot teach 6 示教 */
#define SLOT_RETRACT_MM  90.0f   /* 取药后向后退出距离(mm) */

/**
 * @brief 初始化（清空所有格子）
 */
void slot_teach_init(void);

/**
 * @brief 示教: 查询当前实际关节角并存入格子id
 * @param id 格子编号 (0~SLOT_NUM-1)
 * @return true=成功, false=USB未连接或id越界
 */
bool slot_teach(uint8_t id);

/**
 * @brief 移动到格子id（不操作夹具）
 * @return 0=成功, <0=失败
 */
int slot_goto(uint8_t id);

/**
 * @brief 取药: 移动到格子id → 开真空泵 → 退出50mm
 * @return 0=成功, <0=失败
 */
int slot_grab(uint8_t id);

/**
 * @brief 打印所有已示教格子的关节角
 */
void slot_list(void);

/**
 * @brief 清除格子id的示教数据
 */
void slot_clear(uint8_t id);

/**
 * @brief 完整取药: goto→吸取→退出90mm→去off点→释放→回Home
 * @return 0=成功, <0=失败(失败时自动关真空泵)
 */
int slot_fetch(uint8_t id);

/**
 * @brief 查询格子id是否已示教
 * @return true=已示教, false=未示教或id越界
 */
bool slot_is_taught(uint8_t id);

/**
 * @brief 获取格子id的关节角（供JSON接口等外部模块使用）
 * @param id 格子编号 (0~SLOT_NUM-1)
 * @param joints_out 输出6个关节角(度), 调用者保证非NULL且至少6个float
 * @return true=成功(已示教), false=未示教或id越界
 */
bool slot_get_joints(uint8_t id, float joints_out[6]);

#endif /* SLOT_TEACH_H_ */

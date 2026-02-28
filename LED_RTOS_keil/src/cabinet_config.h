/***************************************************************
 * cabinet_config.h - 药柜配置与坐标管理
 *
 * 药柜布局 (4列 × 4行 = 16格):
 *
 *     列0   列1   列2   列3
 *    ┌────┬────┬────┬────┐
 * 行0│ 00 │ 01 │ 02 │ 03 │  ← 顶层 (A)
 *    ├────┼────┼────┼────┤
 * 行1│ 04 │ 05 │ 06 │ 07 │  ← (B)
 *    ├────┼────┼────┼────┤
 * 行2│ 08 │ 09 │ 10 │ 11 │  ← (C)
 *    ├────┼────┼────┼────┤
 * 行3│ 12 │ 13 │ 14 │ 15 │  ← 底层 (D)
 *    └────┴────┴────┴────┘
 *
 * 坐标系: 基座坐标系 (X右, Y前, Z上)
 ***************************************************************/

#ifndef CABINET_CONFIG_H_
#define CABINET_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

/***************************************************************
 * 药柜尺寸定义
 ***************************************************************/
#define CABINET_ROWS        4       /* 行数 (A-D) */
#define CABINET_COLS        4       /* 列数 (1-4) */
#define CABINET_TOTAL_CELLS (CABINET_ROWS * CABINET_COLS)  /* 16格 */

/***************************************************************
 * 药柜配置结构体 (存入Flash)
 ***************************************************************/
typedef struct {
    /* 药柜原点 (左上角格子中心，基座坐标系) */
    float origin_x;         /* mm */
    float origin_y;         /* mm (到药柜的距离) */
    float origin_z;         /* mm */

    /* 格子尺寸 */
    float cell_width;       /* 格子宽度 mm (X方向间距) */
    float cell_height;      /* 格子高度 mm (Z方向间距) */
    float cell_depth;       /* 格子深度 mm (Y方向，取药深度) */

    /* 抓取姿态 (欧拉角) */
    float grasp_roll;       /* rad */
    float grasp_pitch;      /* rad */
    float grasp_yaw;        /* rad */

    /* 标定状态 */
    bool is_calibrated;     /* 是否已标定 */
    uint32_t calibration_crc;  /* 校验码 */

} CabinetConfig_t;

/***************************************************************
 * 单个格子的微调偏移 (可选，用于补偿机械误差)
 ***************************************************************/
typedef struct {
    float offset_x;         /* mm */
    float offset_y;         /* mm */
    float offset_z;         /* mm */
} CellOffset_t;

/***************************************************************
 * 目标位置结构体
 ***************************************************************/
typedef struct {
    float x, y, z;          /* 位置 mm */
    float roll, pitch, yaw; /* 姿态 rad */
} GraspPose_t;

/***************************************************************
 * 全局变量声明
 ***************************************************************/
extern CabinetConfig_t g_cabinet_config;
extern CellOffset_t g_cell_offsets[CABINET_TOTAL_CELLS];

/***************************************************************
 * API 函数
 ***************************************************************/

/**
 * @brief 初始化药柜配置 (从Flash加载或使用默认值)
 */
void cabinet_config_init(void);

/**
 * @brief 根据格子编号获取抓取位置
 * @param cell_id 格子编号 (0-11)
 * @param pose 输出: 抓取位姿
 * @return true=成功, false=编号无效
 */
bool cabinet_get_grasp_pose(uint8_t cell_id, GraspPose_t *pose);

/**
 * @brief 根据行列获取抓取位置
 * @param row 行 (0-2)
 * @param col 列 (0-3)
 * @param pose 输出: 抓取位姿
 * @return true=成功, false=坐标无效
 */
bool cabinet_get_grasp_pose_rc(uint8_t row, uint8_t col, GraspPose_t *pose);

/**
 * @brief 示教模式: 记录当前位置为药柜原点
 * @param x, y, z 当前末端位置
 */
void cabinet_teach_origin(float x, float y, float z);

/**
 * @brief 示教模式: 记录格子尺寸 (通过两点计算)
 * @param x2, z2 第二个参考点位置 (右下角)
 */
void cabinet_teach_size(float x2, float z2);

/**
 * @brief 保存配置到Flash
 * @return true=成功
 */
bool cabinet_config_save(void);

/**
 * @brief 从Flash加载配置
 * @return true=成功
 */
bool cabinet_config_load(void);

/**
 * @brief 打印当前配置 (调试用)
 */
void cabinet_config_print(void);

#endif /* CABINET_CONFIG_H_ */

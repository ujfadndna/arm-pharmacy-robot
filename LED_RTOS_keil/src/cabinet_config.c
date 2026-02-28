/***************************************************************
 * cabinet_config.c - 药柜配置与坐标管理实现
 ***************************************************************/

#include "cabinet_config.h"
#include "app_common.h"
#include <string.h>

/***************************************************************
 * 默认配置 (需要根据实际药柜尺寸调整)
 ***************************************************************/
#define DEFAULT_ORIGIN_X    150.0f      /* 药柜左边缘到基座的X距离 mm */
#define DEFAULT_ORIGIN_Y    300.0f      /* 药柜到基座的Y距离 mm */
#define DEFAULT_ORIGIN_Z    250.0f      /* 药柜顶层高度 mm */
#define DEFAULT_CELL_WIDTH   80.0f      /* 格子宽度 mm */
#define DEFAULT_CELL_HEIGHT  70.0f      /* 格子高度 mm */
#define DEFAULT_CELL_DEPTH   60.0f      /* 格子深度 mm */

/***************************************************************
 * 全局变量定义
 ***************************************************************/
CabinetConfig_t g_cabinet_config = {0};
CellOffset_t g_cell_offsets[CABINET_TOTAL_CELLS] = {0};

/***************************************************************
 * 内部函数
 ***************************************************************/
static uint32_t calculate_crc(CabinetConfig_t *config)
{
    /* 简易CRC: 累加所有字节 */
    uint8_t *p = (uint8_t *)config;
    uint32_t crc = 0;
    for (size_t i = 0; i < sizeof(CabinetConfig_t) - sizeof(uint32_t); i++) {
        crc += p[i];
    }
    return crc ^ 0xCAFE1234;
}

static void load_default_config(void)
{
    g_cabinet_config.origin_x = DEFAULT_ORIGIN_X;
    g_cabinet_config.origin_y = DEFAULT_ORIGIN_Y;
    g_cabinet_config.origin_z = DEFAULT_ORIGIN_Z;

    g_cabinet_config.cell_width = DEFAULT_CELL_WIDTH;
    g_cabinet_config.cell_height = DEFAULT_CELL_HEIGHT;
    g_cabinet_config.cell_depth = DEFAULT_CELL_DEPTH;

    /* 默认抓取姿态: 水平向前 */
    g_cabinet_config.grasp_roll = 0.0f;
    g_cabinet_config.grasp_pitch = 0.0f;
    g_cabinet_config.grasp_yaw = 0.0f;

    g_cabinet_config.is_calibrated = false;

    /* 清空微调偏移 */
    memset(g_cell_offsets, 0, sizeof(g_cell_offsets));
}

/***************************************************************
 * API 实现
 ***************************************************************/

void cabinet_config_init(void)
{
    /* 尝试从Flash加载 */
    if (!cabinet_config_load()) {
        /* 加载失败，使用默认值 */
        load_default_config();
    }
}

bool cabinet_get_grasp_pose(uint8_t cell_id, GraspPose_t *pose)
{
    if (cell_id >= CABINET_TOTAL_CELLS || pose == NULL) {
        return false;
    }

    uint8_t row = cell_id / CABINET_COLS;  /* 0-2 */
    uint8_t col = cell_id % CABINET_COLS;  /* 0-3 */

    return cabinet_get_grasp_pose_rc(row, col, pose);
}

bool cabinet_get_grasp_pose_rc(uint8_t row, uint8_t col, GraspPose_t *pose)
{
    if (row >= CABINET_ROWS || col >= CABINET_COLS || pose == NULL) {
        return false;
    }

    uint8_t cell_id = (uint8_t)(row * CABINET_COLS + col);

    /* 计算基础位置 */
    pose->x = g_cabinet_config.origin_x + col * g_cabinet_config.cell_width;
    pose->y = g_cabinet_config.origin_y - g_cabinet_config.cell_depth / 2;  /* 进入格子一半深度 */
    pose->z = g_cabinet_config.origin_z - row * g_cabinet_config.cell_height;

    /* 应用微调偏移 */
    pose->x += g_cell_offsets[cell_id].offset_x;
    pose->y += g_cell_offsets[cell_id].offset_y;
    pose->z += g_cell_offsets[cell_id].offset_z;

    /* 抓取姿态 */
    pose->roll = g_cabinet_config.grasp_roll;
    pose->pitch = g_cabinet_config.grasp_pitch;
    pose->yaw = g_cabinet_config.grasp_yaw;

    return true;
}

void cabinet_teach_origin(float x, float y, float z)
{
    g_cabinet_config.origin_x = x;
    g_cabinet_config.origin_y = y;
    g_cabinet_config.origin_z = z;
}

void cabinet_teach_size(float x2, float z2)
{
    /* 根据原点和第二点计算格子尺寸 */
    /* 假设第二点是右下角 (第3列第2行，即编号11) */
    float total_width = x2 - g_cabinet_config.origin_x;
    float total_height = g_cabinet_config.origin_z - z2;

    g_cabinet_config.cell_width = total_width / (CABINET_COLS - 1);
    g_cabinet_config.cell_height = total_height / (CABINET_ROWS - 1);

    g_cabinet_config.is_calibrated = true;
    g_cabinet_config.calibration_crc = calculate_crc(&g_cabinet_config);
}

bool cabinet_config_save(void)
{
    /* TODO: 写入Data Flash
     * 使用FSP的 R_FLASH_LP_Write() 或 R_FLASH_HP_Write()
     *
     * 示例伪代码:
     * R_FLASH_HP_Open(&g_flash_ctrl, &g_flash_cfg);
     * R_FLASH_HP_Erase(&g_flash_ctrl, FLASH_DATA_BLOCK_ADDR, 1);
     * R_FLASH_HP_Write(&g_flash_ctrl, (uint32_t)&g_cabinet_config,
     *                  FLASH_DATA_BLOCK_ADDR, sizeof(CabinetConfig_t));
     * R_FLASH_HP_Close(&g_flash_ctrl);
     */

    g_cabinet_config.calibration_crc = calculate_crc(&g_cabinet_config);

    /* 暂时返回true，实际需要实现Flash写入 */
    return true;
}

bool cabinet_config_load(void)
{
    /* TODO: 从Data Flash读取
     *
     * 示例伪代码:
     * memcpy(&g_cabinet_config, (void*)FLASH_DATA_BLOCK_ADDR, sizeof(CabinetConfig_t));
     */

    /* 验证CRC */
    if (g_cabinet_config.is_calibrated) {
        uint32_t crc = calculate_crc(&g_cabinet_config);
        if (crc == g_cabinet_config.calibration_crc) {
            return true;
        }
    }

    return false;  /* 未标定或CRC错误 */
}

void cabinet_config_print(void)
{
    /* TODO: 通过串口打印配置信息
     *
     * printf("药柜配置:\n");
     * printf("  原点: (%.1f, %.1f, %.1f) mm\n",
     *        g_cabinet_config.origin_x,
     *        g_cabinet_config.origin_y,
     *        g_cabinet_config.origin_z);
     * printf("  格子: %.1f x %.1f x %.1f mm\n",
     *        g_cabinet_config.cell_width,
     *        g_cabinet_config.cell_height,
     *        g_cabinet_config.cell_depth);
     * printf("  已标定: %s\n", g_cabinet_config.is_calibrated ? "是" : "否");
     */
}

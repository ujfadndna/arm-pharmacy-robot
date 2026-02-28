/**
 ******************************************************************************
 * @file    fast_math.h
 * @brief   快速数学函数 (查表+线性插值)
 * @note    用于替代标准math.h中的三角函数，提升IK计算速度
 *          精度: ±0.001 rad, 速度: ~10x 比标准库快
 ******************************************************************************
 */

#ifndef FAST_MATH_H_
#define FAST_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置 ========== */
#define FAST_MATH_TABLE_SIZE    256     /* 查找表大小 (越大越精确) */
#define FAST_MATH_USE_INTERP    1       /* 1=线性插值, 0=直接查表 */

/* ========== 常量 ========== */
#define FM_PI           3.14159265358979f
#define FM_TWO_PI       6.28318530717959f
#define FM_HALF_PI      1.57079632679490f
#define FM_INV_TWO_PI   0.15915494309190f

/* ========== 快速三角函数 ========== */

/**
 * @brief 快速正弦 (查表+插值)
 * @param x 角度 (弧度)
 * @return sin(x)
 */
float fast_sinf(float x);

/**
 * @brief 快速余弦 (查表+插值)
 * @param x 角度 (弧度)
 * @return cos(x)
 */
float fast_cosf(float x);

/**
 * @brief 同时计算sin和cos (更高效)
 * @param x 角度 (弧度)
 * @param s 输出sin(x)
 * @param c 输出cos(x)
 */
void fast_sincosf(float x, float *s, float *c);

/**
 * @brief 快速atan2 (CORDIC算法)
 * @param y Y坐标
 * @param x X坐标
 * @return atan2(y, x) 范围 [-PI, PI]
 */
float fast_atan2f(float y, float x);

/**
 * @brief 快速平方根 (牛顿迭代)
 * @param x 输入值 (x >= 0)
 * @return sqrt(x)
 */
float fast_sqrtf(float x);

/**
 * @brief 快速反平方根 (Quake III算法)
 * @param x 输入值 (x > 0)
 * @return 1/sqrt(x)
 */
float fast_invsqrtf(float x);

/**
 * @brief 初始化查找表 (可选，首次调用时自动初始化)
 */
void fast_math_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FAST_MATH_H_ */

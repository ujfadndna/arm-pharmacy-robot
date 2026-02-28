/**
 ******************************************************************************
 * @file    fast_math.c
 * @brief   快速数学函数实现
 * @note    查表 + 线性插值 + CORDIC算法
 ******************************************************************************
 */

#include "fast_math.h"
#include <math.h>

/* ========== 查找表 ========== */
static float sin_table[FAST_MATH_TABLE_SIZE + 1];
static int table_initialized = 0;

/* ========== 初始化 ========== */

void fast_math_init(void)
{
    if (table_initialized) return;

    /* 生成 [0, PI/2] 范围的sin表 */
    for (int i = 0; i <= FAST_MATH_TABLE_SIZE; i++) {
        float angle = (float)i * FM_HALF_PI / FAST_MATH_TABLE_SIZE;
        sin_table[i] = sinf(angle);  /* 使用标准库初始化 */
    }

    table_initialized = 1;
}

/* ========== 快速正弦 ========== */

float fast_sinf(float x)
{
    if (!table_initialized) fast_math_init();

    /* 归一化到 [0, 2*PI) */
    x = x - FM_TWO_PI * (int)(x * FM_INV_TWO_PI);
    if (x < 0) x += FM_TWO_PI;

    /* 利用对称性映射到 [0, PI/2] */
    int quadrant = (int)(x * (2.0f / FM_PI));
    float angle;
    int sign = 1;

    switch (quadrant & 3) {
        case 0: angle = x; break;
        case 1: angle = FM_PI - x; break;
        case 2: angle = x - FM_PI; sign = -1; break;
        case 3: angle = FM_TWO_PI - x; sign = -1; break;
        default: angle = x; break;
    }

    /* 查表 + 插值 */
    float index_f = angle * (FAST_MATH_TABLE_SIZE / FM_HALF_PI);
    int index = (int)index_f;

    if (index >= FAST_MATH_TABLE_SIZE) {
        return sign * sin_table[FAST_MATH_TABLE_SIZE];
    }

#if FAST_MATH_USE_INTERP
    float frac = index_f - index;
    float result = sin_table[index] + frac * (sin_table[index + 1] - sin_table[index]);
#else
    float result = sin_table[index];
#endif

    return sign * result;
}

/* ========== 快速余弦 ========== */

float fast_cosf(float x)
{
    return fast_sinf(x + FM_HALF_PI);
}

/* ========== 同时计算sin和cos ========== */

void fast_sincosf(float x, float *s, float *c)
{
    if (!table_initialized) fast_math_init();

    /* 归一化到 [0, 2*PI) */
    x = x - FM_TWO_PI * (int)(x * FM_INV_TWO_PI);
    if (x < 0) x += FM_TWO_PI;

    /* 利用对称性 */
    int quadrant = (int)(x * (2.0f / FM_PI));
    float angle;
    int sin_sign = 1, cos_sign = 1;

    switch (quadrant & 3) {
        case 0: angle = x; break;
        case 1: angle = FM_PI - x; cos_sign = -1; break;
        case 2: angle = x - FM_PI; sin_sign = -1; cos_sign = -1; break;
        case 3: angle = FM_TWO_PI - x; sin_sign = -1; break;
        default: angle = x; break;
    }

    /* 查表 + 插值 */
    float index_f = angle * (FAST_MATH_TABLE_SIZE / FM_HALF_PI);
    int index = (int)index_f;

    if (index >= FAST_MATH_TABLE_SIZE) index = FAST_MATH_TABLE_SIZE;

#if FAST_MATH_USE_INTERP
    float frac = index_f - index;
    if (index < FAST_MATH_TABLE_SIZE) {
        *s = sin_sign * (sin_table[index] + frac * (sin_table[index + 1] - sin_table[index]));
        /* cos = sin(PI/2 - angle) */
        int cos_index = FAST_MATH_TABLE_SIZE - index;
        *c = cos_sign * (sin_table[cos_index] - frac * (sin_table[cos_index] - sin_table[cos_index - 1]));
    } else {
        *s = sin_sign * sin_table[index];
        *c = cos_sign * sin_table[0];
    }
#else
    *s = sin_sign * sin_table[index];
    *c = cos_sign * sin_table[FAST_MATH_TABLE_SIZE - index];
#endif
}

/* ========== 快速atan2 (多项式近似) ========== */

float fast_atan2f(float y, float x)
{
    /* 处理特殊情况 */
    if (x == 0.0f) {
        if (y > 0.0f) return FM_HALF_PI;
        if (y < 0.0f) return -FM_HALF_PI;
        return 0.0f;
    }

    float abs_y = (y >= 0) ? y : -y;
    float abs_x = (x >= 0) ? x : -x;
    float a, angle;

    /* 使用 |y|/|x| <= 1 的范围 */
    if (abs_x >= abs_y) {
        a = abs_y / abs_x;
        /* 多项式近似: atan(a) ≈ a - a³/3 + a⁵/5 (简化版) */
        /* 更精确的近似: */
        angle = a * (0.9998660f + a * a * (-0.3302995f + a * a * 0.1801410f));
    } else {
        a = abs_x / abs_y;
        angle = FM_HALF_PI - a * (0.9998660f + a * a * (-0.3302995f + a * a * 0.1801410f));
    }

    /* 根据象限调整 */
    if (x < 0.0f) angle = FM_PI - angle;
    if (y < 0.0f) angle = -angle;

    return angle;
}

/* ========== 快速平方根 (牛顿迭代) ========== */

float fast_sqrtf(float x)
{
    if (x <= 0.0f) return 0.0f;

    /* 使用快速反平方根 + 乘法 */
    return x * fast_invsqrtf(x);
}

/* ========== 快速反平方根 (Quake III 算法) ========== */

float fast_invsqrtf(float x)
{
    union {
        float f;
        uint32_t i;
    } conv;

    conv.f = x;
    conv.i = 0x5F3759DF - (conv.i >> 1);  /* 魔数 */

    /* 一次牛顿迭代提高精度 */
    float xhalf = 0.5f * x;
    conv.f = conv.f * (1.5f - xhalf * conv.f * conv.f);

    /* 可选: 第二次迭代获得更高精度 */
    /* conv.f = conv.f * (1.5f - xhalf * conv.f * conv.f); */

    return conv.f;
}

/**
 ******************************************************************************
 * @file    kin_math.h
 * @brief   运动学数学函数配置
 * @note    通过宏切换标准库/快速库
 ******************************************************************************
 */

#ifndef KIN_MATH_H_
#define KIN_MATH_H_

#include <math.h>  /* 始终需要，用于atanf, fabsf等 */

/* 选择数学库: 0=标准库, 1=快速库 */
#define KIN_USE_FAST_MATH   1

#if KIN_USE_FAST_MATH
    #include "fast_math.h"

    /* 替换标准函数 */
    #define kin_sinf(x)             fast_sinf(x)
    #define kin_cosf(x)             fast_cosf(x)
    #define kin_sincosf(x, s, c)    fast_sincosf(x, s, c)
    #define kin_atan2f(y, x)        fast_atan2f(y, x)
    #define kin_sqrtf(x)            fast_sqrtf(x)
    #define kin_atanf(x)            atanf(x)  /* 暂用标准库 */
    #define kin_fabsf(x)            fabsf(x)

#else
    /* 使用标准函数 */
    #define kin_sinf(x)             sinf(x)
    #define kin_cosf(x)             cosf(x)
    #define kin_atan2f(y, x)        atan2f(y, x)
    #define kin_sqrtf(x)            sqrtf(x)
    #define kin_atanf(x)            atanf(x)
    #define kin_fabsf(x)            fabsf(x)

    /* 标准库没有sincosf，需要调用两次 */
    static inline void kin_sincosf(float x, float *s, float *c) {
        *s = sinf(x);
        *c = cosf(x);
    }
#endif

#endif /* KIN_MATH_H_ */

/**
 ******************************************************************************
 * @file    zdt_test.h
 * @brief   ZDT闭环步进电机测试 - Emm_V5固件协议
 * @note    用于验证CAN通信和电机是否正常工作
 *          已从X_V2协议修改为Emm协议
 ******************************************************************************
 */

#ifndef ZDT_TEST_H_
#define ZDT_TEST_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 宏定义 (来自ZDT官方) ========== */
#define ABS(x)      ((x) > 0 ? (x) : -(x))

/* Emm固件默认16细分: 3200脉冲 = 1圈 = 360° */
#define EMM_PULSES_PER_REV  3200
#define EMM_DEG_TO_PULSES(deg)  ((uint32_t)((float)ABS(deg) * EMM_PULSES_PER_REV / 360.0f))

/* ========== 系统参数枚举 (来自ZDT官方) ========== */
typedef enum {
    S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
    S_RL    = 1,      /* 读取读取相电阻和相电感 */
    S_PID   = 2,      /* 读取PID参数 */
    S_ORG   = 3,      /* 读取回零参数 */
    S_VBUS  = 4,      /* 读取总线电压 */
    S_CBUS  = 5,      /* 读取总线电流 */
    S_CPHA  = 6,      /* 读取相电流 */
    S_ENC   = 7,      /* 读取编码器原始值 */
    S_CPUL  = 8,      /* 读取实时脉冲数（即实时位置计算得到的脉冲数） */
    S_ENCL  = 9,      /* 读取经过线性化校准后的编码器值 */
    S_TPUL  = 10,     /* 读取输入脉冲数 */
    S_TPOS  = 11,     /* 读取电机目标位置 */
    S_OPOS  = 12,     /* 读取电机实时设定的目标位置（速度模式的实时位置） */
    S_VEL   = 13,     /* 读取电机实时转速 */
    S_CPOS  = 14,     /* 读取电机实时位置（基于角度变化量累加的电机实时位置） */
    S_PERR  = 15,     /* 读取电机位置误差 */
    S_TEMP  = 16,     /* 读取电机实时温度 */
    S_SFLAG = 17,     /* 读取状态标志位 */
    S_OFLAG = 18,     /* 读取回零状态标志位 */
    S_Conf  = 19,     /* 读取驱动器配置 */
    S_State = 20,     /* 读取系统状态参数 */
} SysParams_t;

/* ========== ZDT官方API (直接复制) ========== */

/**
 * @brief 初始化ZDT测试模块
 */
void zdt_test_init(void);

/**
 * @brief 将当前位置清零
 * @param addr 电机地址
 */
void ZDT_Reset_CurPos_To_Zero(uint8_t addr);

/**
 * @brief 复位堵转保护
 * @param addr 电机地址
 */
void ZDT_Reset_Clog_Pro(uint8_t addr);

/**
 * @brief 读取系统参数
 * @param addr 电机地址
 * @param s    系统参数类型
 */
void ZDT_Read_Sys_Params(uint8_t addr, SysParams_t s);

/**
 * @brief 使能信号控制
 * @param addr  电机地址
 * @param state 使能状态 (true=使能, false=关闭)
 * @param snF   多机同步标志 (0=不启用)
 */
void ZDT_En_Control(uint8_t addr, bool state, uint8_t snF);

/**
 * @brief 速度模式 (Emm协议)
 * @param addr     电机地址
 * @param dir      方向 (0=CW, 非0=CCW)
 * @param vel      速度 (0-5000 RPM)
 * @param acc      加速度 (0-255, 0=直接启动)
 * @param snF      多机同步标志
 */
void ZDT_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF);

/**
 * @brief 位置模式 (Emm协议, 0xFD)
 * @param addr     电机地址
 * @param dir      方向 (0=CW, 非0=CCW)
 * @param vel      速度 (0-5000 RPM)
 * @param acc      加速度 (0-255, 0=直接启动)
 * @param clk      脉冲数 (3200脉冲=1圈)
 * @param raF      相对/绝对标志 (0=相对, 1=绝对)
 * @param snF      多机同步标志
 */
void ZDT_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF);

/**
 * @brief 位置模式 - 角度接口 (内部转换为脉冲)
 * @param addr     电机地址
 * @param dir      方向 (0=CW, 非0=CCW)
 * @param vel      速度 (0-5000 RPM)
 * @param acc      加速度 (0-255)
 * @param angle_deg 角度 (度)
 * @param raF      相对/绝对标志
 * @param snF      多机同步标志
 */
void ZDT_Pos_Control_Deg(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, float angle_deg, uint8_t raF, uint8_t snF);

/**
 * @brief 立即停止
 * @param addr 电机地址
 * @param snF  多机同步标志
 */
void ZDT_Stop_Now(uint8_t addr, uint8_t snF);

/**
 * @brief 触发同步运动
 * @param addr 电机地址 (通常用广播地址0)
 */
void ZDT_Synchronous_motion(uint8_t addr);

/* ========== 测试函数 ========== */

/**
 * @brief 测试电机1 - 速度模式
 * @note  50:1减速电机，输出轴转速 = 电机转速/50
 *        设置电机100RPM，输出轴约2RPM
 */
void zdt_test_motor1_velocity(void);

/**
 * @brief 测试电机1 - 位置模式
 * @note  转动360度（输出轴）= 电机转18000度
 */
void zdt_test_motor1_position(void);

/**
 * @brief 测试电机1 - 使能/失能
 */
void zdt_test_motor1_enable(bool enable);

/**
 * @brief 测试电机1 - 停止
 */
void zdt_test_motor1_stop(void);

/**
 * @brief 测试电机1 - 读取状态
 */
void zdt_test_motor1_read_status(void);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TEST_H_ */

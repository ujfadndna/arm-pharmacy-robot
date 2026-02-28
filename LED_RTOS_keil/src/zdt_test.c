/**
 ******************************************************************************
 * @file    zdt_test.c
 * @brief   ZDT闭环步进电机测试 - Emm_V5固件协议
 * @note    已从X_V2协议修改为Emm协议
 *          命令格式按照Emm_V5参考代码
 ******************************************************************************
 */

#include "zdt_test.h"
#include "hal_data.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ========== CAN配置 ========== */
#define ZDT_CAN_EXTENDED_FRAME  1       /* 使用扩展帧 */
#define ZDT_CHECKSUM            0x6B    /* 校验字节 */

/* ========== 内部函数 ========== */

/**
 * @brief CAN发送命令 (适配RA6M5)
 * @param cmd 命令数据 (第一个字节是地址)
 * @param len 命令长度
 * @note  CAN帧ID = (地址 << 8)，使用扩展帧
 */
static void can_SendCmd(const uint8_t *cmd, uint8_t len)
{
    if (len < 2) return;

    uint8_t addr = cmd[0];
    uint32_t can_id = (uint32_t)addr << 8;  /* 帧ID = 地址左移8位 */

    can_frame_t frame = {
        .id               = can_id,
        .id_mode          = CAN_ID_MODE_EXTENDED,  /* 扩展帧 */
        .type             = CAN_FRAME_TYPE_DATA,
        .data_length_code = (uint8_t)(len - 1),    /* 数据长度不含地址 */
        .options          = 0
    };

    /* 复制数据 (跳过地址字节) */
    memcpy(frame.data, &cmd[1], len - 1);

    /* 打印调试信息 */
    debug_print("[ZDT TX] ID=0x");
    debug_print_hex(can_id);
    debug_print(" Data=");
    for (uint8_t i = 0; i < len - 1; i++) {
        debug_print_hex(cmd[i + 1]);
        debug_print(" ");
    }
    debug_println("");

    /* 发送 */
    fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, 0, &frame);
    if (err != FSP_SUCCESS) {
        debug_print("[ZDT] CAN TX Error: ");
        debug_print_int((int)err);
        debug_println("");
    }
}

/**
 * @brief CAN发送长命令 (>8字节需要分包)
 * @param cmd 命令数据
 * @param len 命令长度
 * @note  分包规则：帧ID = (地址<<8) + 包序号
 *        每包首字节都是功能码
 */
static void can_SendLongCmd(const uint8_t *cmd, uint8_t len)
{
    if (len < 2) return;

    uint8_t addr = cmd[0];
    uint8_t func_code = cmd[1];

    /* 短命令直接发送 */
    if (len <= 9) {  /* 地址 + 8字节数据 */
        can_SendCmd(cmd, len);
        return;
    }

    /* 长命令分包发送 */
    uint8_t packet_idx = 0;
    uint8_t data_offset = 1;  /* 跳过地址 */

    while (data_offset < len) {
        uint32_t can_id = ((uint32_t)addr << 8) + packet_idx;

        can_frame_t frame = {
            .id               = can_id,
            .id_mode          = CAN_ID_MODE_EXTENDED,
            .type             = CAN_FRAME_TYPE_DATA,
            .options          = 0
        };

        /* 每包首字节是功能码 */
        frame.data[0] = func_code;

        /* 填充数据 */
        uint8_t copy_len = 0;
        if (packet_idx == 0) {
            /* 第一包：功能码 + 后续数据 */
            copy_len = (len - data_offset > 7) ? 7 : (len - data_offset);
            memcpy(&frame.data[1], &cmd[data_offset + 1], copy_len);
            frame.data_length_code = copy_len + 1;
            data_offset += copy_len + 1;
        } else {
            /* 后续包：功能码 + 剩余数据 */
            copy_len = (len - data_offset > 7) ? 7 : (len - data_offset);
            memcpy(&frame.data[1], &cmd[data_offset], copy_len);
            frame.data_length_code = copy_len + 1;
            data_offset += copy_len;
        }

        /* 打印调试信息 */
        debug_print("[ZDT TX PKT");
        debug_print_int(packet_idx);
        debug_print("] ID=0x");
        debug_print_hex(can_id);
        debug_print(" Data=");
        for (uint8_t i = 0; i < frame.data_length_code; i++) {
            debug_print_hex(frame.data[i]);
            debug_print(" ");
        }
        debug_println("");

        /* 发送 */
        fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, 0, &frame);
        if (err != FSP_SUCCESS) {
            debug_print("[ZDT] CAN TX Error: ");
            debug_print_int((int)err);
            debug_println("");
        }

        /* 包间延时 */
        vTaskDelay(pdMS_TO_TICKS(2));

        packet_idx++;
    }
}

/* ========== ZDT官方API实现 (直接复制) ========== */

void zdt_test_init(void)
{
    debug_println("[ZDT] Test module init");

    /* CAN应该已经在motor_can_init中初始化了 */
    /* 这里只是确认一下 */
    fsp_err_t err = R_CANFD_Open(&g_can0_ctrl, &g_can0_cfg);
    if (err == FSP_ERR_ALREADY_OPEN) {
        debug_println("[ZDT] CAN already open - OK");
    } else if (err == FSP_SUCCESS) {
        debug_println("[ZDT] CAN opened - OK");
    } else {
        debug_print("[ZDT] CAN open error: ");
        debug_print_int((int)err);
        debug_println("");
    }
}

/**
 * @brief 将当前位置清零
 */
void ZDT_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;          /* 地址 */
    cmd[1] = 0x0A;          /* 功能码 */
    cmd[2] = 0x6D;          /* 辅助码 */
    cmd[3] = 0x6B;          /* 校验字节 */

    can_SendCmd(cmd, 4);
}

/**
 * @brief 复位堵转保护
 */
void ZDT_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x0E;
    cmd[2] = 0x52;
    cmd[3] = 0x6B;

    can_SendCmd(cmd, 4);
}

/**
 * @brief 读取系统参数
 */
void ZDT_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;

    switch(s) {
        case S_VER   : cmd[1] = 0x1F; break;
        case S_RL    : cmd[1] = 0x20; break;
        case S_PID   : cmd[1] = 0x21; break;
        case S_ORG   : cmd[1] = 0x22; break;
        case S_VBUS  : cmd[1] = 0x24; break;
        case S_CBUS  : cmd[1] = 0x26; break;
        case S_CPHA  : cmd[1] = 0x27; break;
        case S_ENC   : cmd[1] = 0x29; break;
        case S_CPUL  : cmd[1] = 0x30; break;
        case S_ENCL  : cmd[1] = 0x31; break;
        case S_TPUL  : cmd[1] = 0x32; break;
        case S_TPOS  : cmd[1] = 0x33; break;
        case S_OPOS  : cmd[1] = 0x34; break;
        case S_VEL   : cmd[1] = 0x35; break;
        case S_CPOS  : cmd[1] = 0x36; break;
        case S_PERR  : cmd[1] = 0x37; break;
        case S_TEMP  : cmd[1] = 0x39; break;
        case S_SFLAG : cmd[1] = 0x3A; break;
        case S_OFLAG : cmd[1] = 0x3B; break;
        case S_Conf  : cmd[1] = 0x42; cmd[2] = 0x6C; break;
        case S_State : cmd[1] = 0x43; cmd[2] = 0x7A; break;
        default: break;
    }

    if (s >= S_Conf) {
        cmd[3] = 0x6B;
        can_SendCmd(cmd, 4);
    } else {
        cmd[2] = 0x6B;
        can_SendCmd(cmd, 3);
    }
}

/**
 * @brief 使能信号控制
 */
void ZDT_En_Control(uint8_t addr, bool state, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xF3;          /* 功能码 */
    cmd[2] = 0xAB;          /* 辅助码 */
    cmd[3] = (uint8_t)state;
    cmd[4] = snF;
    cmd[5] = 0x6B;

    can_SendCmd(cmd, 6);
}

/**
 * @brief 速度模式 (Emm协议)
 * @note  Emm格式: [addr][0xF6][dir][vel_H][vel_L][acc][snF][0x6B] = 8字节
 *        X_V2格式: [addr][0xF6][dir][v_ramp_H][v_ramp_L][vel_H][vel_L][snF][0x6B] = 9字节
 */
void ZDT_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xF6;          /* 功能码 */
    cmd[2] = dir;           /* 方向 */
    cmd[3] = (uint8_t)(vel >> 8);   /* 速度高字节 */
    cmd[4] = (uint8_t)(vel >> 0);   /* 速度低字节 */
    cmd[5] = acc;                    /* 加速度 (0-255) */
    cmd[6] = snF;
    cmd[7] = 0x6B;

    can_SendCmd(cmd, 8);
}

/**
 * @brief 位置模式 (Emm协议, 0xFD)
 * @note  Emm格式: [addr][0xFD][dir][vel_H][vel_L][acc][clk×4][raF][snF][0x6B] = 13字节
 *        X_V2格式: [addr][0xFD][dir][acc_H][acc_L][dec_H][dec_L][vel_H][vel_L][pos×4][raf][snF][0x6B] = 16字节
 */
void ZDT_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    cmd[0]  = addr;
    cmd[1]  = 0xFD;         /* 功能码 */
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel >> 0);
    cmd[5]  = acc;                    /* 加速度 (0-255) */
    cmd[6]  = (uint8_t)(clk >> 24);  /* 脉冲数 高字节 */
    cmd[7]  = (uint8_t)(clk >> 16);
    cmd[8]  = (uint8_t)(clk >> 8);
    cmd[9]  = (uint8_t)(clk >> 0);   /* 脉冲数 低字节 */
    cmd[10] = raF;                    /* 0=相对, 1=绝对 */
    cmd[11] = snF;
    cmd[12] = 0x6B;

    can_SendLongCmd(cmd, 13);
}

/**
 * @brief 位置模式 - 角度接口 (内部转换为脉冲)
 */
void ZDT_Pos_Control_Deg(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, float angle_deg, uint8_t raF, uint8_t snF)
{
    uint32_t clk = EMM_DEG_TO_PULSES(angle_deg);
    ZDT_Pos_Control(addr, dir, vel, acc, clk, raF, snF);
}

/**
 * @brief 立即停止
 */
void ZDT_Stop_Now(uint8_t addr, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF;
    cmd[4] = 0x6B;

    can_SendCmd(cmd, 5);
}

/**
 * @brief 触发同步运动
 */
void ZDT_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xFF;
    cmd[2] = 0x66;
    cmd[3] = 0x6B;

    can_SendCmd(cmd, 4);
}

/* ========== 测试函数 ========== */

/**
 * @brief 测试电机1 - 速度模式
 * @note  50:1减速电机
 *        电机转速100RPM -> 输出轴2RPM
 *        电机转速500RPM -> 输出轴10RPM
 */
void zdt_test_motor1_velocity(void)
{
    debug_println("========================================");
    debug_println("[TEST] Motor1 Velocity Mode (Emm)");
    debug_println("  Addr=1, Dir=CW, Vel=100RPM, Acc=10");
    debug_println("  (50:1 reducer -> output ~2RPM)");
    debug_println("========================================");

    /* 先使能电机 */
    ZDT_En_Control(1, true, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 速度模式：地址1，CW方向，速度100RPM，加速度10 */
    ZDT_Velocity_Control(1, 0, 100, 10, 0);

    debug_println("[TEST] Command sent. Motor should be running...");
    debug_println("[TEST] Call zdt_test_motor1_stop() to stop");
}

/**
 * @brief 测试电机1 - 位置模式
 * @note  50:1减速电机
 *        输出轴转1圈 = 电机转50圈 = 160000脉冲 (3200*50)
 *        测试: 电机转10圈 = 32000脉冲
 */
void zdt_test_motor1_position(void)
{
    debug_println("========================================");
    debug_println("[TEST] Motor1 Position Mode (Emm)");
    debug_println("  Addr=1, Dir=CW, Vel=100RPM, Clk=32000");
    debug_println("  (10 revolutions, 50:1 -> output 72deg)");
    debug_println("========================================");

    /* 先使能电机 */
    ZDT_En_Control(1, true, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 位置模式：地址1，CW方向，速度100RPM，加速度10，32000脉冲(10圈)，相对位置 */
    ZDT_Pos_Control(1, 0, 100, 10, 32000, 0, 0);

    debug_println("[TEST] Command sent. Motor should move 32000 pulses...");
}

/**
 * @brief 测试电机1 - 使能/失能
 */
void zdt_test_motor1_enable(bool enable)
{
    debug_print("[TEST] Motor1 Enable: ");
    debug_println(enable ? "ON" : "OFF");

    ZDT_En_Control(1, enable, 0);
}

/**
 * @brief 测试电机1 - 停止
 */
void zdt_test_motor1_stop(void)
{
    debug_println("[TEST] Motor1 STOP");

    ZDT_Stop_Now(1, 0);
}

/**
 * @brief 测试电机1 - 读取状态
 */
void zdt_test_motor1_read_status(void)
{
    debug_println("[TEST] Motor1 Read Status (0x3A)");

    ZDT_Read_Sys_Params(1, S_SFLAG);
}

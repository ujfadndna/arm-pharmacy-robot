/**
 ******************************************************************************
 * @file    motor_test.c
 * @brief   单电机测试工具 - 用于硬件调试
 * @note    测试STM32驱动板与RA6M5主控板的CAN通信
 ******************************************************************************
 */

#include "motor_can_dummy.h"
#include "hal_data.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== 测试配置 ========== */
#define TEST_JOINT_ID       0       /* 测试关节ID (0-5) */
#define TEST_DELAY_MS       100     /* 测试间隔 */

/* ========== 测试函数声明 ========== */
static void test_enable_disable(void);
static void test_position_control(void);
static void test_velocity_control(void);
static void test_position_query(void);
static void test_temperature_query(void);
static void test_multi_joint_sync(void);
static void test_parameter_config(void);

/* ========== 主测试入口 ========== */

/**
 * @brief 测试1: 使能/失能电机
 */
static void test_enable_disable(void)
{
    LOG_I("TEST", "=== Test 1: Enable/Disable Motor ===");

    /* 使能电机 */
    LOG_I("TEST", "Enabling motor %d...", TEST_JOINT_ID);
    int ret = motor_can_set_enable(TEST_JOINT_ID, true);
    if (ret == 0) {
        LOG_I("TEST", "✓ Enable command sent");
    } else {
        LOG_E("TEST", "✗ Enable command failed");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 失能电机 */
    LOG_I("TEST", "Disabling motor %d...", TEST_JOINT_ID);
    ret = motor_can_set_enable(TEST_JOINT_ID, false);
    if (ret == 0) {
        LOG_I("TEST", "✓ Disable command sent");
    } else {
        LOG_E("TEST", "✗ Disable command failed");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    LOG_I("TEST", "=== Test 1 Complete ===\n");
}

/**
 * @brief 测试2: 位置控制 (0° → 90° → 0°)
 */
static void test_position_control(void)
{
    LOG_I("TEST", "=== Test 2: Position Control ===");

    /* 使能电机 */
    motor_can_set_enable(TEST_JOINT_ID, true);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 移动到90度 */
    LOG_I("TEST", "Moving to 90 degrees...");
    int ret = motor_can_set_position(TEST_JOINT_ID, 90.0f, true);
    if (ret == 0) {
        LOG_I("TEST", "✓ Position command sent");
    } else {
        LOG_E("TEST", "✗ Position command failed");
        return;
    }

    /* 等待到位 */
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));

        motor_state_t state;
        motor_can_get_state(TEST_JOINT_ID, &state);

        LOG_I("TEST", "Position: %.2f°, Finish: %d",
              state.current_angle, state.finish_flag);

        if (state.finish_flag) {
            LOG_I("TEST", "✓ Reached 90 degrees");
            break;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 返回0度 */
    LOG_I("TEST", "Moving back to 0 degrees...");
    ret = motor_can_set_position(TEST_JOINT_ID, 0.0f, true);
    if (ret == 0) {
        LOG_I("TEST", "✓ Position command sent");
    }

    /* 等待到位 */
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));

        motor_state_t state;
        motor_can_get_state(TEST_JOINT_ID, &state);

        LOG_I("TEST", "Position: %.2f°, Finish: %d",
              state.current_angle, state.finish_flag);

        if (state.finish_flag) {
            LOG_I("TEST", "✓ Reached 0 degrees");
            break;
        }
    }

    motor_can_set_enable(TEST_JOINT_ID, false);
    LOG_I("TEST", "=== Test 2 Complete ===\n");
}

/**
 * @brief 测试3: 速度控制
 */
static void test_velocity_control(void)
{
    LOG_I("TEST", "=== Test 3: Velocity Control ===");

    /* 使能电机 */
    motor_can_set_enable(TEST_JOINT_ID, true);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 设置速度: 30度/秒 */
    LOG_I("TEST", "Setting velocity to 30 deg/s...");
    int ret = motor_can_set_velocity(TEST_JOINT_ID, 30.0f);
    if (ret == 0) {
        LOG_I("TEST", "✓ Velocity command sent");
    } else {
        LOG_E("TEST", "✗ Velocity command failed");
        return;
    }

    /* 运行3秒 */
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));

        motor_state_t state;
        motor_can_get_state(TEST_JOINT_ID, &state);

        LOG_I("TEST", "Position: %.2f°, Velocity: %.2f deg/s",
              state.current_angle, state.velocity);
    }

    /* 停止 */
    LOG_I("TEST", "Stopping motor...");
    motor_can_set_velocity(TEST_JOINT_ID, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(500));

    motor_can_set_enable(TEST_JOINT_ID, false);
    LOG_I("TEST", "=== Test 3 Complete ===\n");
}

/**
 * @brief 测试4: 位置查询
 */
static void test_position_query(void)
{
    LOG_I("TEST", "=== Test 4: Position Query ===");

    /* 使能电机 */
    motor_can_set_enable(TEST_JOINT_ID, true);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 查询位置10次 */
    for (int i = 0; i < 10; i++) {
        int ret = motor_can_get_position(TEST_JOINT_ID);
        if (ret == 0) {
            LOG_I("TEST", "✓ Position query sent");
        } else {
            LOG_E("TEST", "✗ Position query failed");
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        motor_state_t state;
        motor_can_get_state(TEST_JOINT_ID, &state);

        LOG_I("TEST", "Query %d: Position = %.2f°", i+1, state.current_angle);
    }

    motor_can_set_enable(TEST_JOINT_ID, false);
    LOG_I("TEST", "=== Test 4 Complete ===\n");
}

/**
 * @brief 测试5: 温度查询
 */
static void test_temperature_query(void)
{
    LOG_I("TEST", "=== Test 5: Temperature Query ===");

    /* 查询温度5次 */
    for (int i = 0; i < 5; i++) {
        int ret = motor_can_get_temperature(TEST_JOINT_ID);
        if (ret == 0) {
            LOG_I("TEST", "✓ Temperature query sent");
        } else {
            LOG_E("TEST", "✗ Temperature query failed");
        }

        vTaskDelay(pdMS_TO_TICKS(200));

        motor_state_t state;
        motor_can_get_state(TEST_JOINT_ID, &state);

        LOG_I("TEST", "Query %d: Temperature = %.1f°C", i+1, state.temperature);
    }

    LOG_I("TEST", "=== Test 5 Complete ===\n");
}

/**
 * @brief 测试6: 多轴同步运动
 */
static void test_multi_joint_sync(void)
{
    LOG_I("TEST", "=== Test 6: Multi-Joint Sync ===");

    /* 使能所有电机 */
    LOG_I("TEST", "Enabling all motors...");
    motor_can_set_enable(0xFF, true);
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 所有关节移动到45度 */
    LOG_I("TEST", "Moving all joints to 45 degrees...");
    for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        motor_can_set_position(i, 45.0f, false);
    }

    /* 等待所有电机到位 */
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));

        bool all_reached = motor_can_all_reached();

        /* 打印所有关节位置 */
        for (uint8_t j = 0; j < MOTOR_CAN_NUM_JOINTS; j++) {
            motor_state_t state;
            motor_can_get_state(j, &state);
            LOG_I("TEST", "J%d: %.2f°", j, state.current_angle);
        }

        if (all_reached) {
            LOG_I("TEST", "✓ All joints reached target");
            break;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 所有关节返回0度 */
    LOG_I("TEST", "Moving all joints back to 0 degrees...");
    for (uint8_t i = 0; i < MOTOR_CAN_NUM_JOINTS; i++) {
        motor_can_set_position(i, 0.0f, false);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    /* 失能所有电机 */
    motor_can_set_enable(0xFF, false);
    LOG_I("TEST", "=== Test 6 Complete ===\n");
}

/**
 * @brief 测试7: 参数配置
 */
static void test_parameter_config(void)
{
    LOG_I("TEST", "=== Test 7: Parameter Config ===");

    /* 设置电流限制 */
    LOG_I("TEST", "Setting current limit to 2.0A...");
    int ret = motor_can_set_current_limit(TEST_JOINT_ID, 2.0f, false);
    if (ret == 0) {
        LOG_I("TEST", "✓ Current limit set");
    } else {
        LOG_E("TEST", "✗ Current limit failed");
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* 设置速度限制 */
    LOG_I("TEST", "Setting velocity limit to 180 deg/s...");
    ret = motor_can_set_velocity_limit(TEST_JOINT_ID, 180.0f, false);
    if (ret == 0) {
        LOG_I("TEST", "✓ Velocity limit set");
    } else {
        LOG_E("TEST", "✗ Velocity limit failed");
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* 设置加速度 */
    LOG_I("TEST", "Setting acceleration to 360 deg/s²...");
    ret = motor_can_set_acceleration(TEST_JOINT_ID, 360.0f, false);
    if (ret == 0) {
        LOG_I("TEST", "✓ Acceleration set");
    } else {
        LOG_E("TEST", "✗ Acceleration failed");
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* 设置DCE参数 (参考dummy_robot.h的推荐值) */
    LOG_I("TEST", "Setting DCE gains (Kp=1000, Kv=80, Ki=200, Kd=250)...");
    ret = motor_can_set_dce_gains(TEST_JOINT_ID, 1000, 80, 200, 250, false);
    if (ret == 0) {
        LOG_I("TEST", "✓ DCE gains set");
    } else {
        LOG_E("TEST", "✗ DCE gains failed");
    }

    LOG_I("TEST", "=== Test 7 Complete ===\n");
}

/**
 * @brief 运行所有测试
 */
void motor_test_run_all(void)
{
    LOG_I("TEST", "\n");
    LOG_I("TEST", "========================================");
    LOG_I("TEST", "  Motor CAN Test Suite");
    LOG_I("TEST", "  Target: STM32 Dummy Driver Board");
    LOG_I("TEST", "========================================\n");

    /* ---- CAN Bus-Off 恢复: 关闭可能处于错误状态的 CAN 外设 ---- */
    R_CANFD_Close(&g_can0_ctrl);
    vTaskDelay(pdMS_TO_TICKS(10));

    fsp_err_t can_err = R_CANFD_Open(&g_can0_ctrl, &g_can0_cfg);
    LOG_I("TEST", "CAN re-open: %s (err=%d)",
          (can_err == FSP_SUCCESS) ? "OK" : "FAIL", (int)can_err);

    /* ---- CAN 物理层探测 ---- */
    {
        /* 直接读 CANFD 寄存器诊断 */
        volatile uint32_t gsts = R_CANFD->CFDGSTS;       /* Global Status */
        volatile uint32_t chsts = R_CANFD->CFDC[0].STS;  /* Channel 0 Status */
        volatile uint32_t chctr = R_CANFD->CFDC[0].CTR;  /* Channel 0 Control */
        LOG_I("TEST", "CAN regs: GSTS=0x%08lX CH_STS=0x%08lX CH_CTR=0x%08lX",
              (unsigned long)gsts, (unsigned long)chsts, (unsigned long)chctr);

        /* 发一帧探测帧，等待 100ms，然后读错误计数器 */
        can_frame_t probe = {
            .id               = 0x7FF,  /* 不会匹配任何电机 */
            .id_mode          = CAN_ID_MODE_STANDARD,
            .type             = CAN_FRAME_TYPE_DATA,
            .data_length_code = 1,
            .options          = 0
        };
        memset(probe.data, 0, sizeof(probe.data));
        probe.data[0] = 0xAA;
        fsp_err_t wr_err = R_CANFD_Write(&g_can0_ctrl, 0, &probe);
        LOG_I("TEST", "CAN probe write: err=%d", (int)wr_err);
        vTaskDelay(pdMS_TO_TICKS(100));

        /* 读 TX mailbox 0 状态 */
        volatile uint32_t tmsts0 = R_CANFD->CFDTMSTS[0];
        volatile uint32_t tmsts1 = R_CANFD->CFDTMSTS[1];
        LOG_I("TEST", "TXMB status: MB0=0x%02lX MB1=0x%02lX",
              (unsigned long)tmsts0, (unsigned long)tmsts1);

        can_info_t info;
        if (R_CANFD_InfoGet(&g_can0_ctrl, &info) == FSP_SUCCESS) {
            LOG_I("TEST", "CAN probe: TX_ERR=%d RX_ERR=%d status=0x%02X",
                  info.error_count_transmit,
                  info.error_count_receive,
                  info.error_code);
        }

        /* 用TMTRM判断帧是否真正发出（比TX_ERR更准确） */
        volatile uint8_t mb0_sts = R_CANFD->CFDTMSTS[0];
        if (mb0_sts & 0x08) {
            /* TMTRM=1: 帧仍在mailbox中，从未发出 */
            LOG_E("TEST", "!! CAN TX stuck (TMTRM=1) - frame never sent !!");
            LOG_E("TEST", "!! Check: baud rate match, transceiver STB pin, CAN wiring !!");
            /* Abort卡死的探测帧 */
            R_CANFD->CFDTMC[0] = 0x02;
            vTaskDelay(pdMS_TO_TICKS(1));
            R_CANFD->CFDTMSTS[0] = 0;
        } else if ((mb0_sts & 0x06) == 0x04) {
            /* TMTRF=10: 发送成功完成 */
            LOG_I("TEST", "CAN bus ACK OK - frame transmitted successfully");
        } else {
            LOG_W("TEST", "CAN TX state: MB0=0x%02X", mb0_sts);
        }
    }

    /* 初始化CAN驱动 */
    motor_can_config_t config = {
        .base_addr = 1,
        .max_speed_rps = 30.0f,
        .max_acceleration = 100.0f,
        .current_limit = 2.0f,
        .use_standard_frame = true
    };
    motor_can_init(&config);

    /* 重置超时计时器 */
    motor_can_reset_timeout(0xFF);

    /* 运行测试 */
    test_enable_disable();
    vTaskDelay(pdMS_TO_TICKS(1000));

    test_position_control();
    vTaskDelay(pdMS_TO_TICKS(1000));

    test_velocity_control();
    vTaskDelay(pdMS_TO_TICKS(1000));

    test_position_query();
    vTaskDelay(pdMS_TO_TICKS(1000));

    test_temperature_query();
    vTaskDelay(pdMS_TO_TICKS(1000));

    test_parameter_config();
    vTaskDelay(pdMS_TO_TICKS(1000));

    test_multi_joint_sync();

    /* ---- 测试后 CAN 诊断 ---- */
    {
        can_info_t info;
        if (R_CANFD_InfoGet(&g_can0_ctrl, &info) == FSP_SUCCESS) {
            LOG_I("TEST", "CAN post: TX_ERR=%d RX_ERR=%d status=0x%02X",
                  info.error_count_transmit,
                  info.error_count_receive,
                  info.error_code);
        }
        uint32_t tx_cnt, err_flags;
        motor_can_get_debug_info(&tx_cnt, &err_flags);
        LOG_I("TEST", "CAN stats: tx_complete=%lu err_flags=0x%08lX",
              (unsigned long)tx_cnt, (unsigned long)err_flags);
    }

    LOG_I("TEST", "\n========================================");
    LOG_I("TEST", "  All Tests Complete");
    LOG_I("TEST", "========================================\n");
}

/**
 * @brief 运行单个测试
 */
void motor_test_run_single(uint8_t test_id)
{
    /* ---- CAN Bus-Off 恢复: 关闭可能处于错误状态的 CAN 外设 ---- */
    R_CANFD_Close(&g_can0_ctrl);
    vTaskDelay(pdMS_TO_TICKS(10));

    fsp_err_t can_err = R_CANFD_Open(&g_can0_ctrl, &g_can0_cfg);
    LOG_I("TEST", "CAN re-open: %s (err=%d)",
          (can_err == FSP_SUCCESS) ? "OK" : "FAIL", (int)can_err);

    /* ---- CAN 状态诊断 ---- */
    can_info_t can_info;
    if (R_CANFD_InfoGet(&g_can0_ctrl, &can_info) == FSP_SUCCESS) {
        LOG_I("TEST", "CAN diag: TX_ERR=%d RX_ERR=%d status=0x%02X",
              can_info.error_count_transmit,
              can_info.error_count_receive,
              can_info.error_code);
    }

    /* 初始化CAN驱动 */
    motor_can_config_t config = {
        .base_addr = 1,
        .max_speed_rps = 30.0f,
        .max_acceleration = 100.0f,
        .current_limit = 2.0f,
        .use_standard_frame = true
    };
    motor_can_init(&config);
    motor_can_reset_timeout(0xFF);

    switch (test_id) {
        case 1:
            test_enable_disable();
            break;
        case 2:
            test_position_control();
            break;
        case 3:
            test_velocity_control();
            break;
        case 4:
            test_position_query();
            break;
        case 5:
            test_temperature_query();
            break;
        case 6:
            test_multi_joint_sync();
            break;
        case 7:
            test_parameter_config();
            break;
        default:
            LOG_E("TEST", "Invalid test ID: %d", test_id);
            break;
    }
}

/**
 * @brief 手动位置控制测试
 * @param joint_id 关节ID (0-5)
 * @param target_angle 目标角度(度)
 */
void motor_test_move_to(uint8_t joint_id, float target_angle)
{
    LOG_I("TEST", "Moving joint %d to %.2f degrees...", joint_id, target_angle);

    motor_can_set_enable(joint_id, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    motor_can_set_position(joint_id, target_angle, true);

    /* 等待到位 */
    for (int i = 0; i < 100; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));

        motor_state_t state;
        motor_can_get_state(joint_id, &state);

        LOG_I("TEST", "Position: %.2f°, Finish: %d",
              state.current_angle, state.finish_flag);

        if (state.finish_flag) {
            LOG_I("TEST", "✓ Reached target");
            break;
        }
    }
}

/**
 * @brief 超时检测测试
 */
void motor_test_timeout_check(void)
{
    LOG_I("TEST", "=== Timeout Check Test ===");

    for (int i = 0; i < 10; i++) {
        int timeout_id = motor_can_check_timeout();

        if (timeout_id >= 0) {
            LOG_W("TEST", "Motor %d timeout detected!", timeout_id);

            uint32_t last_rx = motor_can_get_last_rx_time(timeout_id);
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            LOG_W("TEST", "Last RX: %lu ms ago", now - last_rx);
        } else {
            LOG_I("TEST", "All motors OK");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    LOG_I("TEST", "=== Timeout Check Complete ===\n");
}

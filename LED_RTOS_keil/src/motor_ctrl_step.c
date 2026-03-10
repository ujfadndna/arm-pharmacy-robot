/**
 ******************************************************************************
 * @file    motor_ctrl_step.c
 * @brief   dummy-auk CtrlStep CAN protocol implementation (RA6M5 + FSP CANFD)
 ******************************************************************************
 */

#include "motor_ctrl_step.h"
#include "hal_data.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <math.h>
#include <string.h>

/* ========== Driver internal constants ========== */
#define MOTOR_CTRL_STEP_TX_MAILBOX_COUNT   (4U)   /* use TX MB0..3 */
#define MOTOR_CTRL_STEP_TX_RETRY_MAX       (120U)
#define MOTOR_CTRL_STEP_TX_RETRY_DELAY_US  (50U)

/* ========== Joint configuration (from dummy_robot.cpp) ========== */
const motor_ctrl_step_joint_config_t joint_config[MOTOR_CTRL_STEP_NUM_JOINTS] =
{
    /* node, inverse, reduction, angle_min, angle_max */
    {1U, true,  30.0f, -170.0f, 170.0f},
    {2U, false, 30.0f,  -73.0f,  90.0f},
    {3U, true,  30.0f,   35.0f, 180.0f},
    {4U, false, 24.0f, -180.0f, 180.0f},
    {5U, true,  30.0f, -120.0f, 120.0f},
    {6U, true,  50.0f, -720.0f, 720.0f},
};

/* ========== Runtime state ========== */
static motor_ctrl_step_feedback_t g_feedback[MOTOR_CTRL_STEP_NUM_JOINTS];
static bool g_initialized = false;
static uint8_t g_tx_mb_index = 0U;

static SemaphoreHandle_t g_tx_sem = NULL;
static StaticSemaphore_t g_tx_sem_buffer;

/* ========== Utilities ========== */
static void motor_ctrl_step_delay_us(uint32_t us)
{
    volatile uint32_t n = us * 40U; /* coarse delay for 200 MHz */
    while (n--)
    {
        __NOP();
    }
}

static bool motor_ctrl_step_valid_joint(uint8_t joint_index)
{
    return joint_index < MOTOR_CTRL_STEP_NUM_JOINTS;
}

static int8_t motor_ctrl_step_find_joint_by_node(uint8_t node_id)
{
    for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++)
    {
        if (joint_config[i].node_id == node_id)
        {
            return (int8_t) i;
        }
    }
    return -1;
}

static uint16_t motor_ctrl_step_build_id(uint8_t node_id, uint8_t cmd)
{
    return (uint16_t) (((uint16_t) node_id << 7U) | ((uint16_t) cmd & 0x7FU));
}

static void motor_ctrl_step_float_to_le(float value, uint8_t out[4])
{
    /* Target and peer are little-endian (ARM), memcpy preserves wire format used by dummy-auk. */
    memcpy(out, &value, 4U);
}

static float motor_ctrl_step_float_from_le(const uint8_t in[4])
{
    float value = 0.0f;
    memcpy(&value, in, 4U);
    return value;
}

static float motor_ctrl_step_angle_to_motor_rev(uint8_t joint_index, float angle_deg)
{
    float mapped = joint_config[joint_index].inverse ? -angle_deg : angle_deg;
    return mapped / 360.0f * joint_config[joint_index].reduction;
}

static float motor_ctrl_step_motor_rev_to_angle(uint8_t joint_index, float motor_rev)
{
    float angle = motor_rev / joint_config[joint_index].reduction * 360.0f;
    return joint_config[joint_index].inverse ? -angle : angle;
}

static void motor_ctrl_step_log_frame(const char * prefix, uint32_t can_id, const uint8_t * data, uint8_t len)
{
    debug_print(prefix);
    debug_print(" ID=0x");
    debug_print_hex(can_id);
    debug_print(" DLC=");
    debug_print_int((int) len);
    debug_print(" DATA=");

    for (uint8_t i = 0; i < len; i++)
    {
        debug_print_hex(data[i]);
        if (i + 1U < len)
        {
            debug_print(" ");
        }
    }
    debug_println("");
}

static int motor_ctrl_step_can_write(uint16_t can_id, const uint8_t * data, uint8_t len)
{
    can_frame_t frame =
    {
        .id               = can_id,
        .id_mode          = CAN_ID_MODE_STANDARD,
        .type             = CAN_FRAME_TYPE_DATA,
        .data_length_code = len,
        .options          = 0U
    };
    memset(frame.data, 0, sizeof(frame.data));
    memcpy(frame.data, data, len);

    for (uint32_t retry = 0U; retry < MOTOR_CTRL_STEP_TX_RETRY_MAX; retry++)
    {
        for (uint8_t mb = 0U; mb < MOTOR_CTRL_STEP_TX_MAILBOX_COUNT; mb++)
        {
            uint8_t mailbox = (uint8_t) ((g_tx_mb_index + mb) % MOTOR_CTRL_STEP_TX_MAILBOX_COUNT);
            fsp_err_t err = R_CANFD_Write(&g_can0_ctrl, mailbox, &frame);

            if (FSP_SUCCESS == err)
            {
                g_tx_mb_index = (uint8_t) ((mailbox + 1U) % MOTOR_CTRL_STEP_TX_MAILBOX_COUNT);
                return 0;
            }
        }

        if (g_tx_sem != NULL)
        {
            (void) xSemaphoreTake(g_tx_sem, pdMS_TO_TICKS(1));
        }
        else
        {
            motor_ctrl_step_delay_us(MOTOR_CTRL_STEP_TX_RETRY_DELAY_US);
        }
    }

    return -1;
}

static void motor_ctrl_step_log_maintenance_cmd(const char * tag, uint8_t joint_index, const char * phase)
{
    debug_print("[CTRL_STEP] ");
    debug_print(tag);
    debug_print(" ");
    debug_print(phase);
    debug_print(", joint=");
    if (MOTOR_CTRL_STEP_ALL_JOINTS == joint_index)
    {
        debug_println("ALL");
    }
    else
    {
        debug_print_int((int) joint_index);
        debug_println("");
    }
}

static int motor_ctrl_step_send_maintenance_cmd(uint8_t joint_index, uint8_t cmd, const char * tag)
{
    if (!g_initialized)
    {
        return -1;
    }

    uint8_t node_id = 0U; /* node 0 = broadcast */
    if (MOTOR_CTRL_STEP_ALL_JOINTS != joint_index)
    {
        if (!motor_ctrl_step_valid_joint(joint_index))
        {
            return -1;
        }
        node_id = joint_config[joint_index].node_id;
    }

    /* dummy-auk maintenance commands use all-zero payload, DLC fixed to 8. */
    uint8_t data[MOTOR_CTRL_STEP_CAN_DLC] = {0};

    uint16_t can_id = motor_ctrl_step_build_id(node_id, cmd);
    motor_ctrl_step_log_maintenance_cmd(tag, joint_index, "request");
    motor_ctrl_step_log_frame("[CTRL_STEP][TX]", can_id, data, MOTOR_CTRL_STEP_CAN_DLC);

    int ret = motor_ctrl_step_can_write(can_id, data, MOTOR_CTRL_STEP_CAN_DLC);
    if (ret != 0)
    {
        debug_print("[CTRL_STEP] ");
        debug_print(tag);
        debug_println(" TX failed");
        return ret;
    }

    motor_ctrl_step_log_maintenance_cmd(tag, joint_index, "sent");
    return 0;
}

/* ========== Public API ========== */
int motor_ctrl_step_init(void)
{
    memset(g_feedback, 0, sizeof(g_feedback));
    g_tx_mb_index = 0U;

    if (g_tx_sem == NULL)
    {
        g_tx_sem = xSemaphoreCreateBinaryStatic(&g_tx_sem_buffer);
    }

    fsp_err_t err = R_CANFD_Open(&g_can0_ctrl, &g_can0_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        debug_print("[CTRL_STEP] CAN open failed, err=");
        debug_print_int((int) err);
        debug_println("");
        g_initialized = false;
        return -1;
    }

    g_initialized = true;
    debug_println("[CTRL_STEP] Init OK (standard CAN, nominal 1Mbps from g_can0_cfg)");
    return 0;
}

int motor_ctrl_step_set_enable(uint8_t joint_index, bool enable)
{
    if (!g_initialized)
    {
        return -1;
    }

    uint8_t node_id = 0U; /* node 0 = broadcast in dummy-auk */

    if (MOTOR_CTRL_STEP_ALL_JOINTS != joint_index)
    {
        if (!motor_ctrl_step_valid_joint(joint_index))
        {
            return -1;
        }
        node_id = joint_config[joint_index].node_id;
    }

    uint8_t data[MOTOR_CTRL_STEP_CAN_DLC] = {0};
    uint32_t enable_u32 = enable ? 1U : 0U;
    memcpy(&data[0], &enable_u32, sizeof(enable_u32));

    uint16_t can_id = motor_ctrl_step_build_id(node_id, MOTOR_CTRL_STEP_CMD_ENABLE);
    motor_ctrl_step_log_frame("[CTRL_STEP][TX]", can_id, data, MOTOR_CTRL_STEP_CAN_DLC);

    int ret = motor_ctrl_step_can_write(can_id, data, MOTOR_CTRL_STEP_CAN_DLC);
    if (ret != 0)
    {
        debug_println("[CTRL_STEP] set_enable TX failed");
        return ret;
    }

    if (MOTOR_CTRL_STEP_ALL_JOINTS == joint_index)
    {
        for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++)
        {
            g_feedback[i].finished = enable;
        }
    }
    else
    {
        g_feedback[joint_index].finished = enable;
    }

    return 0;
}

int motor_ctrl_step_set_position(uint8_t joint_index, float angle_deg)
{
    if (!g_initialized || !motor_ctrl_step_valid_joint(joint_index))
    {
        return -1;
    }

    if ((angle_deg < joint_config[joint_index].angle_min) ||
        (angle_deg > joint_config[joint_index].angle_max))
    {
        debug_print("[CTRL_STEP] angle out of limit, joint=");
        debug_print_int((int) joint_index);
        debug_println("");
        return -2;
    }

    float motor_rev = motor_ctrl_step_angle_to_motor_rev(joint_index, angle_deg);
    uint8_t data[MOTOR_CTRL_STEP_CAN_DLC] = {0};

    motor_ctrl_step_float_to_le(motor_rev, &data[0]);
    data[4] = 1U; /* request ACK (0x23 response) */

    uint16_t can_id = motor_ctrl_step_build_id(joint_config[joint_index].node_id, MOTOR_CTRL_STEP_CMD_SET_POS);
    motor_ctrl_step_log_frame("[CTRL_STEP][TX]", can_id, data, MOTOR_CTRL_STEP_CAN_DLC);

    int ret = motor_ctrl_step_can_write(can_id, data, MOTOR_CTRL_STEP_CAN_DLC);
    if (ret != 0)
    {
        debug_println("[CTRL_STEP] set_position TX failed");
        return ret;
    }

    g_feedback[joint_index].target_angle_deg = angle_deg;
    g_feedback[joint_index].target_motor_rev = motor_rev;
    g_feedback[joint_index].finished = false;

    return 0;
}

int motor_ctrl_step_set_position_with_speed(uint8_t joint_index, float angle_deg, float velocity_deg_s)
{
    if (!g_initialized || !motor_ctrl_step_valid_joint(joint_index))
    {
        return -1;
    }

    if ((angle_deg < joint_config[joint_index].angle_min) ||
        (angle_deg > joint_config[joint_index].angle_max))
    {
        debug_print("[CTRL_STEP] angle out of limit, joint=");
        debug_print_int((int) joint_index);
        debug_println("");
        return -2;
    }

    float motor_rev = motor_ctrl_step_angle_to_motor_rev(joint_index, angle_deg);

    /* Convert joint deg/s -> motor rev/s for protocol 0x07. */
    float velocity_deg_s_abs = fabsf(velocity_deg_s);
    float velocity_motor_rev_s = velocity_deg_s_abs / 360.0f * joint_config[joint_index].reduction;

    uint8_t data[MOTOR_CTRL_STEP_CAN_DLC] = {0};
    motor_ctrl_step_float_to_le(motor_rev, &data[0]);
    motor_ctrl_step_float_to_le(velocity_motor_rev_s, &data[4]);

    uint16_t can_id = motor_ctrl_step_build_id(joint_config[joint_index].node_id, MOTOR_CTRL_STEP_CMD_SET_POS_VEL);
    motor_ctrl_step_log_frame("[CTRL_STEP][TX]", can_id, data, MOTOR_CTRL_STEP_CAN_DLC);

    int ret = motor_ctrl_step_can_write(can_id, data, MOTOR_CTRL_STEP_CAN_DLC);
    if (ret != 0)
    {
        debug_println("[CTRL_STEP] set_position_with_speed TX failed");
        return ret;
    }

    g_feedback[joint_index].target_angle_deg = angle_deg;
    g_feedback[joint_index].target_motor_rev = motor_rev;
    g_feedback[joint_index].finished = false;

    return 0;
}

int motor_ctrl_step_query_position(uint8_t joint_index)
{
    if (!g_initialized || !motor_ctrl_step_valid_joint(joint_index))
    {
        return -1;
    }

    uint8_t data[MOTOR_CTRL_STEP_CAN_DLC] = {0};
    uint16_t can_id = motor_ctrl_step_build_id(joint_config[joint_index].node_id, MOTOR_CTRL_STEP_CMD_QUERY_POS);

    motor_ctrl_step_log_frame("[CTRL_STEP][TX]", can_id, data, MOTOR_CTRL_STEP_CAN_DLC);
    int ret = motor_ctrl_step_can_write(can_id, data, MOTOR_CTRL_STEP_CAN_DLC);
    if (ret != 0)
    {
        debug_println("[CTRL_STEP] query_position TX failed");
        return ret;
    }

    /* Try to pull pending RX frames immediately (optional fast-path). */
    (void) motor_ctrl_step_poll_rx();

    return 0;
}

int motor_ctrl_step_home(uint8_t joint_index)
{
    int ret = motor_ctrl_step_send_maintenance_cmd(
        joint_index,
        MOTOR_CTRL_STEP_CMD_HOME,
        "home");
    if (ret != 0)
    {
        return ret;
    }

    if (MOTOR_CTRL_STEP_ALL_JOINTS == joint_index)
    {
        for (uint8_t i = 0; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++)
        {
            g_feedback[i].target_angle_deg = 0.0f;
            g_feedback[i].target_motor_rev = 0.0f;
            g_feedback[i].feedback_angle_deg = 0.0f;
            g_feedback[i].feedback_motor_rev = 0.0f;
            g_feedback[i].finished = true;
        }
    }
    else if (motor_ctrl_step_valid_joint(joint_index))
    {
        g_feedback[joint_index].target_angle_deg = 0.0f;
        g_feedback[joint_index].target_motor_rev = 0.0f;
        g_feedback[joint_index].feedback_angle_deg = 0.0f;
        g_feedback[joint_index].feedback_motor_rev = 0.0f;
        g_feedback[joint_index].finished = true;
    }

    return 0;
}

int motor_ctrl_step_clear_clog(uint8_t joint_index)
{
    return motor_ctrl_step_send_maintenance_cmd(
        joint_index,
        MOTOR_CTRL_STEP_CMD_CLEAR_CLOG,
        "clear_clog");
}

int motor_ctrl_step_clear_stall(uint8_t joint_index)
{
    return motor_ctrl_step_clear_clog(joint_index);
}

int motor_ctrl_step_poll_rx(void)
{
    if (!g_initialized)
    {
        return -1;
    }

    int parsed = 0;
    while (1)
    {
        can_frame_t frame = {0};
        fsp_err_t err = R_CANFD_Read(&g_can0_ctrl, CANFD_RX_BUFFER_FIFO_0, &frame);

        if (FSP_SUCCESS == err)
        {
            if ((frame.id_mode == CAN_ID_MODE_STANDARD) && (frame.type == CAN_FRAME_TYPE_DATA))
            {
                motor_ctrl_step_rx_callback(frame.id, frame.data, frame.data_length_code);
                parsed++;
            }
            continue;
        }

        if (FSP_ERR_BUFFER_EMPTY == err)
        {
            return parsed;
        }

        debug_print("[CTRL_STEP] poll_rx failed, err=");
        debug_print_int((int) err);
        debug_println("");
        return (parsed > 0) ? parsed : -1;
    }
}

void motor_ctrl_step_rx_callback(uint32_t can_id, const uint8_t * data, uint8_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    uint8_t cmd = (uint8_t) (can_id & 0x7FU);
    uint8_t node_id = (uint8_t) ((can_id >> 7U) & 0x0FU);
    int8_t joint_index = motor_ctrl_step_find_joint_by_node(node_id);
    if (joint_index < 0)
    {
        return;
    }

    motor_ctrl_step_log_frame("[CTRL_STEP][RX]", can_id, data, len);

    if ((cmd == MOTOR_CTRL_STEP_CMD_QUERY_POS) && (len >= 5U))
    {
        float pos_motor_rev = motor_ctrl_step_float_from_le(&data[0]);
        uint8_t state = data[4];
        float angle_deg = motor_ctrl_step_motor_rev_to_angle((uint8_t) joint_index, pos_motor_rev);

        g_feedback[joint_index].feedback_motor_rev = pos_motor_rev;
        g_feedback[joint_index].feedback_angle_deg = angle_deg;
        g_feedback[joint_index].state_byte = state;
        g_feedback[joint_index].finished = (state != 0U);
        g_feedback[joint_index].online = true;
        g_feedback[joint_index].last_rx_tick = xTaskGetTickCount();
    }
}

bool motor_ctrl_step_get_feedback(uint8_t joint_index, motor_ctrl_step_feedback_t * out_feedback)
{
    if (!motor_ctrl_step_valid_joint(joint_index) || (out_feedback == NULL))
    {
        return false;
    }

    *out_feedback = g_feedback[joint_index];
    return true;
}

void motor_ctrl_step_tx_complete_notify(void)
{
    if (g_tx_sem != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

uint8_t motor_ctrl_step_check_timeout(void)
{
    if (!g_initialized)
    {
        return 0U;
    }

    uint8_t timeout_mask = 0U;
    TickType_t current_tick = xTaskGetTickCount();
    const TickType_t timeout_threshold = 100U; /* 500ms @ 5ms tick */

    for (uint8_t i = 0U; i < MOTOR_CTRL_STEP_NUM_JOINTS; i++)
    {
        /* Skip joints that never came online */
        if (!g_feedback[i].online)
        {
            continue;
        }

        /* Check if time since last RX exceeds threshold */
        TickType_t elapsed = current_tick - g_feedback[i].last_rx_tick;
        if (elapsed > timeout_threshold)
        {
            timeout_mask |= (1U << i);
        }
    }

    return timeout_mask;
}

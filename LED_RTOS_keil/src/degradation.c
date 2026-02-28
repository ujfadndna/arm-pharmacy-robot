/**
 ******************************************************************************
 * @file    degradation.c
 * @brief   Graceful degradation strategy implementation
 ******************************************************************************
 */

#include "degradation.h"
#include "motor_can.h"
#include "motion_controller.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========== Internal State ========== */
static degradation_status_t g_status;

/* Track per-joint fault counts for smart decisions */
static uint8_t g_joint_fault_count[6] = {0};

/* ========== Helper Functions ========== */

static uint32_t get_tick_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void apply_degradation(degrade_level_t level)
{
    g_status.level = level;

    switch (level) {
        case DEGRADE_NONE:
            g_status.speed_scale = 1.0f;
            break;

        case DEGRADE_SLOW:
            g_status.speed_scale = DEGRADE_SLOW_SPEED_SCALE;
            break;

        case DEGRADE_SINGLE_JOINT:
            /* Speed scale unchanged, joint disabled via bitmask */
            break;

        case DEGRADE_POSITION_HOLD:
            g_status.speed_scale = 0.0f;
            motion_stop();
            break;

        case DEGRADE_EMERGENCY:
            g_status.speed_scale = 0.0f;
            motor_can_emergency_stop();
            motion_stop();
            break;
    }
}

static int count_disabled_joints(void)
{
    int count = 0;
    for (int i = 0; i < 6; i++) {
        if (g_status.disabled_joints & (1 << i)) {
            count++;
        }
    }
    return count;
}

/* ========== Public API Implementation ========== */

void degradation_init(void)
{
    g_status.level = DEGRADE_NONE;
    g_status.last_fault = FAULT_NONE;
    g_status.fault_joint = 0xFF;
    g_status.consecutive_failures = 0;
    g_status.disabled_joints = 0;
    g_status.speed_scale = 1.0f;
    g_status.fault_timestamp = 0;

    for (int i = 0; i < 6; i++) {
        g_joint_fault_count[i] = 0;
    }
}

degrade_level_t degradation_handle_fault(fault_type_t fault, uint8_t joint_id)
{
    degrade_level_t new_level = g_status.level;

    g_status.last_fault = fault;
    g_status.fault_joint = joint_id;
    g_status.fault_timestamp = get_tick_ms();

    /* Track per-joint faults */
    if (joint_id < 6) {
        g_joint_fault_count[joint_id]++;
    }

    /* Decision logic based on fault type */
    switch (fault) {
        case FAULT_CAN_TIMEOUT:
            if (joint_id < 6) {
                /* Single joint timeout - try disabling that joint */
                if (count_disabled_joints() < 2) {
                    g_status.disabled_joints |= (1 << joint_id);
                    new_level = DEGRADE_SINGLE_JOINT;
                    debug_print("[DEGRADE] Joint ");
                    debug_print_int(joint_id);
                    debug_println(" disabled (CAN timeout)");
                } else {
                    /* Too many joints failed */
                    new_level = DEGRADE_EMERGENCY;
                    debug_println("[DEGRADE] Multiple CAN timeouts -> EMERGENCY");
                }
            } else {
                /* Unknown joint or broadcast timeout */
                new_level = DEGRADE_EMERGENCY;
            }
            break;

        case FAULT_MOTOR_STALL:
            /* Stall = potential collision, hold position and wait for user */
            new_level = DEGRADE_POSITION_HOLD;
            debug_print("[DEGRADE] Motor stall J");
            debug_print_int(joint_id);
            debug_println(" -> POSITION_HOLD");
            break;

        case FAULT_POSITION_ERROR:
            /* Position error - try slowing down first */
            if (g_status.level < DEGRADE_SLOW) {
                new_level = DEGRADE_SLOW;
                debug_println("[DEGRADE] Position error -> SLOW mode (50% speed)");
            } else {
                /* Already slow, escalate */
                g_status.consecutive_failures++;
                if (g_status.consecutive_failures >= DEGRADE_MAX_FAILURES) {
                    new_level = DEGRADE_POSITION_HOLD;
                    debug_println("[DEGRADE] Repeated position errors -> POSITION_HOLD");
                }
            }
            break;

        case FAULT_WATCHDOG:
            /* Watchdog warning - reduce load immediately */
            new_level = DEGRADE_POSITION_HOLD;
            debug_println("[DEGRADE] Watchdog warning -> POSITION_HOLD");
            break;

        default:
            break;
    }

    /* Check consecutive failure limit */
    if (new_level != DEGRADE_NONE && new_level != DEGRADE_EMERGENCY) {
        g_status.consecutive_failures++;
        if (g_status.consecutive_failures >= DEGRADE_MAX_FAILURES) {
            new_level = DEGRADE_EMERGENCY;
            debug_println("[DEGRADE] Max failures reached -> EMERGENCY");
        }
    }

    apply_degradation(new_level);
    return new_level;
}

degrade_level_t degradation_get_level(void)
{
    return g_status.level;
}

void degradation_get_status(degradation_status_t *status)
{
    if (status) {
        *status = g_status;
    }
}

bool degradation_recover(void)
{
    /* Cannot recover from emergency without explicit clear */
    if (g_status.level == DEGRADE_EMERGENCY) {
        return false;
    }

    /* Check if enough time has passed */
    uint32_t elapsed = get_tick_ms() - g_status.fault_timestamp;
    if (elapsed < DEGRADE_RECOVERY_DELAY_MS) {
        return false;
    }

    /* Try to step down one level */
    switch (g_status.level) {
        case DEGRADE_POSITION_HOLD:
            /* Try slow mode first */
            apply_degradation(DEGRADE_SLOW);
            debug_println("[DEGRADE] Recovery: POSITION_HOLD -> SLOW");
            return true;

        case DEGRADE_SLOW:
            /* Check if we can return to normal */
            if (g_status.disabled_joints == 0) {
                apply_degradation(DEGRADE_NONE);
                g_status.consecutive_failures = 0;
                debug_println("[DEGRADE] Recovery: SLOW -> NORMAL");
                return true;
            }
            break;

        case DEGRADE_SINGLE_JOINT:
            /* Cannot auto-recover disabled joints */
            break;

        default:
            break;
    }

    return false;
}

void degradation_clear(void)
{
    debug_println("[DEGRADE] Clearing all faults");

    /* Clear stall protection on all motors */
    motor_can_clear_stall(0xFF);

    /* Reset state */
    g_status.level = DEGRADE_NONE;
    g_status.last_fault = FAULT_NONE;
    g_status.fault_joint = 0xFF;
    g_status.consecutive_failures = 0;
    g_status.disabled_joints = 0;
    g_status.speed_scale = 1.0f;

    for (int i = 0; i < 6; i++) {
        g_joint_fault_count[i] = 0;
    }
}

bool degradation_is_joint_disabled(uint8_t joint_id)
{
    if (joint_id >= 6) return false;
    return (g_status.disabled_joints & (1 << joint_id)) != 0;
}

float degradation_get_speed_scale(void)
{
    return g_status.speed_scale;
}

const char* degradation_level_str(degrade_level_t level)
{
    switch (level) {
        case DEGRADE_NONE:          return "NORMAL";
        case DEGRADE_SLOW:          return "SLOW";
        case DEGRADE_SINGLE_JOINT:  return "SINGLE_JOINT";
        case DEGRADE_POSITION_HOLD: return "POSITION_HOLD";
        case DEGRADE_EMERGENCY:     return "EMERGENCY";
        default:                    return "UNKNOWN";
    }
}

const char* degradation_fault_str(fault_type_t fault)
{
    switch (fault) {
        case FAULT_NONE:            return "NONE";
        case FAULT_CAN_TIMEOUT:     return "CAN_TIMEOUT";
        case FAULT_MOTOR_STALL:     return "MOTOR_STALL";
        case FAULT_POSITION_ERROR:  return "POSITION_ERROR";
        case FAULT_WATCHDOG:        return "WATCHDOG";
        default:                    return "UNKNOWN";
    }
}

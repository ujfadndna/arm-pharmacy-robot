/**
 ******************************************************************************
 * @file    degradation.h
 * @brief   Graceful degradation strategy for robotic arm fault handling
 * @note    Provides multi-level degradation instead of immediate emergency stop
 ******************************************************************************
 */

#ifndef DEGRADATION_H_
#define DEGRADATION_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Degradation Levels ========== */
typedef enum {
    DEGRADE_NONE = 0,       /* Normal operation */
    DEGRADE_SLOW,           /* Reduced speed (50%) */
    DEGRADE_SINGLE_JOINT,   /* Single joint disabled */
    DEGRADE_POSITION_HOLD,  /* Hold position (stop motion, keep power) */
    DEGRADE_EMERGENCY       /* Emergency stop (last resort) */
} degrade_level_t;

/* ========== Fault Types ========== */
typedef enum {
    FAULT_NONE = 0,
    FAULT_CAN_TIMEOUT,      /* CAN communication timeout */
    FAULT_MOTOR_STALL,      /* Motor stall detected */
    FAULT_POSITION_ERROR,   /* Position error too large */
    FAULT_WATCHDOG          /* Watchdog about to timeout */
} fault_type_t;

/* ========== Degradation Status ========== */
typedef struct {
    degrade_level_t level;          /* Current degradation level */
    fault_type_t last_fault;        /* Last fault type */
    uint8_t fault_joint;            /* Joint that caused fault (0-5) */
    uint8_t consecutive_failures;   /* Consecutive degradation failures */
    uint8_t disabled_joints;        /* Bitmask of disabled joints */
    float speed_scale;              /* Speed scaling factor (0.0-1.0) */
    uint32_t fault_timestamp;       /* Timestamp of last fault */
} degradation_status_t;

/* ========== Configuration ========== */
#define DEGRADE_MAX_FAILURES        3   /* Max failures before emergency */
#define DEGRADE_SLOW_SPEED_SCALE    0.5f /* Speed scale in DEGRADE_SLOW */
#define DEGRADE_RECOVERY_DELAY_MS   1000 /* Delay before auto-recovery attempt */

/* ========== Public API ========== */

/**
 * @brief Initialize degradation module
 */
void degradation_init(void);

/**
 * @brief Handle a fault and determine degradation action
 * @param fault Fault type
 * @param joint_id Joint that caused fault (0-5), 0xFF if not joint-specific
 * @return New degradation level after handling
 */
degrade_level_t degradation_handle_fault(fault_type_t fault, uint8_t joint_id);

/**
 * @brief Get current degradation level
 */
degrade_level_t degradation_get_level(void);

/**
 * @brief Get full degradation status
 */
void degradation_get_status(degradation_status_t *status);

/**
 * @brief Attempt recovery from degraded state
 * @return true if recovery successful, false otherwise
 */
bool degradation_recover(void);

/**
 * @brief Clear all faults and reset to normal
 * @note Should be called after user acknowledges fault
 */
void degradation_clear(void);

/**
 * @brief Check if a joint is disabled
 * @param joint_id Joint ID (0-5)
 * @return true if joint is disabled
 */
bool degradation_is_joint_disabled(uint8_t joint_id);

/**
 * @brief Get current speed scale factor
 * @return Speed scale (0.0-1.0), 1.0 = normal speed
 */
float degradation_get_speed_scale(void);

/**
 * @brief Convert degradation level to string
 */
const char* degradation_level_str(degrade_level_t level);

/**
 * @brief Convert fault type to string
 */
const char* degradation_fault_str(fault_type_t fault);

#ifdef __cplusplus
}
#endif

#endif /* DEGRADATION_H_ */

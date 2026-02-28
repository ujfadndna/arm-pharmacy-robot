/**
 ******************************************************************************
 * @file    trajectory.h
 * @brief   轨迹规划模块 - 多点路径规划与插值
 * @note    支持Minimum Jerk和梯形速度曲线
 ******************************************************************************
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 ========== */
#define TRAJ_MAX_JOINTS         6       /* 最大关节数 */
#define TRAJ_MAX_WAYPOINTS      16      /* 最大路径点数 */
#define TRAJ_DEFAULT_PERIOD_MS  5       /* 默认插值周期(ms) - 200Hz控制频率 */

/* ========== 插值类型 ========== */
typedef enum {
    TRAJ_INTERP_MINIMUM_JERK = 0,   /* 最小加加速度(平滑) */
    TRAJ_INTERP_TRAPEZOIDAL,        /* 梯形速度曲线(快速) */
    TRAJ_INTERP_LINEAR              /* 线性插值(简单) */
} traj_interp_type_t;

/* ========== 轨迹状态 ========== */
typedef enum {
    TRAJ_STATE_IDLE = 0,    /* 空闲 */
    TRAJ_STATE_RUNNING,     /* 执行中 */
    TRAJ_STATE_PAUSED,      /* 暂停 */
    TRAJ_STATE_DONE,        /* 完成 */
    TRAJ_STATE_ERROR        /* 错误 */
} traj_state_t;

/* ========== 路径点定义 ========== */
typedef struct {
    float q[TRAJ_MAX_JOINTS];   /* 关节角度(度) */
    float duration_ms;          /* 到达此点的时间(ms) */
    traj_interp_type_t interp;  /* 插值类型 */
} traj_waypoint_t;

/* ========== 速度限制 ========== */
typedef struct {
    float max_vel[TRAJ_MAX_JOINTS];     /* 最大速度(度/秒) */
    float max_acc[TRAJ_MAX_JOINTS];     /* 最大加速度(度/秒²) */
} traj_limits_t;

/* ========== 轨迹规划器 ========== */
typedef struct {
    /* 路径点队列 */
    traj_waypoint_t waypoints[TRAJ_MAX_WAYPOINTS];
    uint32_t waypoint_count;
    uint32_t current_waypoint;

    /* 当前段状态 */
    float q_start[TRAJ_MAX_JOINTS];     /* 段起点 */
    float q_end[TRAJ_MAX_JOINTS];       /* 段终点 */
    uint32_t segment_step;              /* 当前步 */
    uint32_t segment_total_steps;       /* 总步数 */
    traj_interp_type_t current_interp;  /* 当前插值类型 */

    /* 全局状态 */
    traj_state_t state;
    uint32_t period_ms;                 /* 插值周期 */

    /* 速度限制 */
    traj_limits_t limits;
    bool use_limits;

    /* 输出 */
    float q_current[TRAJ_MAX_JOINTS];   /* 当前插值结果 */
} traj_planner_t;

/* ========== 公共接口 ========== */

/**
 * @brief 初始化轨迹规划器
 * @param planner 规划器实例
 * @param period_ms 插值周期(ms)，0=使用默认值
 */
void traj_init(traj_planner_t *planner, uint32_t period_ms);

/**
 * @brief 设置速度限制
 * @param planner 规划器实例
 * @param limits 速度限制，NULL=禁用限制
 */
void traj_set_limits(traj_planner_t *planner, const traj_limits_t *limits);

/**
 * @brief 清空路径点队列
 */
void traj_clear(traj_planner_t *planner);

/**
 * @brief 添加路径点
 * @param planner 规划器实例
 * @param waypoint 路径点
 * @return 0=成功, -1=队列已满
 */
int traj_add_waypoint(traj_planner_t *planner, const traj_waypoint_t *waypoint);

/**
 * @brief 快速添加路径点(简化版)
 * @param planner 规划器实例
 * @param q 关节角度数组
 * @param duration_ms 到达时间
 * @param interp 插值类型
 * @return 0=成功, -1=队列已满
 */
int traj_add_point(traj_planner_t *planner, const float q[TRAJ_MAX_JOINTS],
                   float duration_ms, traj_interp_type_t interp);

/**
 * @brief 从当前位置开始执行轨迹
 * @param planner 规划器实例
 * @param q_current 当前关节角度
 * @return 0=成功, -1=无路径点
 */
int traj_start(traj_planner_t *planner, const float q_current[TRAJ_MAX_JOINTS]);

/**
 * @brief 暂停轨迹执行
 */
void traj_pause(traj_planner_t *planner);

/**
 * @brief 恢复轨迹执行
 */
void traj_resume(traj_planner_t *planner);

/**
 * @brief 停止轨迹执行
 */
void traj_stop(traj_planner_t *planner);

/**
 * @brief 轨迹步进(每个周期调用一次)
 * @param planner 规划器实例
 * @param q_out 输出的关节角度
 * @return 状态: TRAJ_STATE_RUNNING/DONE/IDLE
 */
traj_state_t traj_step(traj_planner_t *planner, float q_out[TRAJ_MAX_JOINTS]);

/**
 * @brief 获取当前状态
 */
traj_state_t traj_get_state(const traj_planner_t *planner);

/**
 * @brief 获取进度(0.0~1.0)
 */
float traj_get_progress(const traj_planner_t *planner);

/* ========== 工具函数 ========== */

/**
 * @brief Minimum Jerk插值
 */
float traj_minimum_jerk(float t, float T, float x0, float xf);

/**
 * @brief 梯形速度插值
 */
float traj_trapezoidal(float t, float T, float x0, float xf, float v_max, float a_max);

/**
 * @brief 根据速度限制计算最短时间
 */
float traj_calc_min_duration(const float q_start[TRAJ_MAX_JOINTS],
                             const float q_end[TRAJ_MAX_JOINTS],
                             const traj_limits_t *limits);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_H_ */


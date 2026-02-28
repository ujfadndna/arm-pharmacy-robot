/**
 ******************************************************************************
 * @file    trajectory.c
 * @brief   轨迹规划模块实现
 ******************************************************************************
 */

#include "trajectory.h"
#include <math.h>
#include <string.h>

/* ========== 内部函数声明 ========== */
static void start_segment(traj_planner_t *planner);
static float interpolate_joint(traj_interp_type_t interp, float t, float T,
                               float x0, float xf, float v_max, float a_max);

/* ========== 公共接口实现 ========== */

void traj_init(traj_planner_t *planner, uint32_t period_ms)
{
    memset(planner, 0, sizeof(traj_planner_t));
    planner->period_ms = (period_ms > 0) ? period_ms : TRAJ_DEFAULT_PERIOD_MS;
    planner->state = TRAJ_STATE_IDLE;
    planner->use_limits = false;

    /* 默认速度限制 */
    for (int i = 0; i < TRAJ_MAX_JOINTS; i++) {
        planner->limits.max_vel[i] = 180.0f;  /* 180度/秒 */
        planner->limits.max_acc[i] = 360.0f;  /* 360度/秒² */
    }
}

void traj_set_limits(traj_planner_t *planner, const traj_limits_t *limits)
{
    if (limits) {
        memcpy(&planner->limits, limits, sizeof(traj_limits_t));
        planner->use_limits = true;
    } else {
        planner->use_limits = false;
    }
}

void traj_clear(traj_planner_t *planner)
{
    planner->waypoint_count = 0;
    planner->current_waypoint = 0;
    planner->state = TRAJ_STATE_IDLE;
}

int traj_add_waypoint(traj_planner_t *planner, const traj_waypoint_t *waypoint)
{
    if (planner->waypoint_count >= TRAJ_MAX_WAYPOINTS) {
        return -1;
    }
    memcpy(&planner->waypoints[planner->waypoint_count], waypoint,
           sizeof(traj_waypoint_t));
    planner->waypoint_count++;
    return 0;
}

int traj_add_point(traj_planner_t *planner, const float q[TRAJ_MAX_JOINTS],
                   float duration_ms, traj_interp_type_t interp)
{
    if (planner->waypoint_count >= TRAJ_MAX_WAYPOINTS) {
        return -1;
    }
    traj_waypoint_t *wp = &planner->waypoints[planner->waypoint_count];
    memcpy(wp->q, q, sizeof(float) * TRAJ_MAX_JOINTS);
    wp->duration_ms = duration_ms;
    wp->interp = interp;
    planner->waypoint_count++;
    return 0;
}

int traj_start(traj_planner_t *planner, const float q_current[TRAJ_MAX_JOINTS])
{
    if (planner->waypoint_count == 0) {
        return -1;
    }

    /* 设置起点为当前位置 */
    memcpy(planner->q_start, q_current, sizeof(float) * TRAJ_MAX_JOINTS);
    memcpy(planner->q_current, q_current, sizeof(float) * TRAJ_MAX_JOINTS);

    planner->current_waypoint = 0;
    planner->state = TRAJ_STATE_RUNNING;

    /* 开始第一段 */
    start_segment(planner);

    return 0;
}

void traj_pause(traj_planner_t *planner)
{
    if (planner->state == TRAJ_STATE_RUNNING) {
        planner->state = TRAJ_STATE_PAUSED;
    }
}

void traj_resume(traj_planner_t *planner)
{
    if (planner->state == TRAJ_STATE_PAUSED) {
        planner->state = TRAJ_STATE_RUNNING;
    }
}

void traj_stop(traj_planner_t *planner)
{
    planner->state = TRAJ_STATE_IDLE;
    planner->segment_step = 0;
}

traj_state_t traj_step(traj_planner_t *planner, float q_out[TRAJ_MAX_JOINTS])
{
    if (planner->state != TRAJ_STATE_RUNNING) {
        memcpy(q_out, planner->q_current, sizeof(float) * TRAJ_MAX_JOINTS);
        return planner->state;
    }

    /* 计算当前时间 */
    float t = (float)planner->segment_step;
    float T = (float)planner->segment_total_steps;

    /* 对每个关节进行插值 */
    for (int i = 0; i < TRAJ_MAX_JOINTS; i++) {
        float v_max = planner->limits.max_vel[i] * (float)planner->period_ms / 1000.0f;
        float a_max = planner->limits.max_acc[i] * (float)planner->period_ms / 1000.0f;

        planner->q_current[i] = interpolate_joint(
            planner->current_interp, t, T,
            planner->q_start[i], planner->q_end[i],
            v_max, a_max
        );
    }

    memcpy(q_out, planner->q_current, sizeof(float) * TRAJ_MAX_JOINTS);
    planner->segment_step++;

    /* 检查段是否完成 */
    if (planner->segment_step >= planner->segment_total_steps) {
        planner->current_waypoint++;

        if (planner->current_waypoint >= planner->waypoint_count) {
            /* 所有路径点完成 */
            planner->state = TRAJ_STATE_DONE;
        } else {
            /* 开始下一段 */
            memcpy(planner->q_start, planner->q_end, sizeof(float) * TRAJ_MAX_JOINTS);
            start_segment(planner);
        }
    }

    return planner->state;
}

traj_state_t traj_get_state(const traj_planner_t *planner)
{
    return planner->state;
}

float traj_get_progress(const traj_planner_t *planner)
{
    if (planner->waypoint_count == 0) {
        return 0.0f;
    }

    float segment_progress = 0.0f;
    if (planner->segment_total_steps > 0) {
        segment_progress = (float)planner->segment_step / (float)planner->segment_total_steps;
    }

    float total = (float)planner->current_waypoint + segment_progress;
    return total / (float)planner->waypoint_count;
}

/* ========== 插值函数实现 ========== */

float traj_minimum_jerk(float t, float T, float x0, float xf)
{
    if (T <= 0) return xf;
    float tau = t / T;
    if (tau >= 1.0f) return xf;
    if (tau <= 0.0f) return x0;

    float tau3 = tau * tau * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;
    return x0 + (xf - x0) * (10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5);
}

float traj_trapezoidal(float t, float T, float x0, float xf, float v_max, float a_max)
{
    (void)v_max;
    (void)a_max;

    if (T <= 0) return xf;
    float tau = t / T;
    if (tau >= 1.0f) return xf;
    if (tau <= 0.0f) return x0;

    /*
     * 梯形速度曲线 (归一化, 对称)
     * 加速段占比 r = 0.2, 匀速段占比 0.6, 减速段占比 0.2
     *
     * 加速段 [0, 0.2]: s = 2.5 * tau^2
     *   s(0.2) = 2.5 * 0.04 = 0.1
     *
     * 匀速段 [0.2, 0.8]: s = 0.1 + (tau - 0.2) * (0.8/0.6)
     *   斜率 = 0.8/0.6 = 4/3 ≈ 1.333
     *   s(0.8) = 0.1 + 0.6 * 1.333 = 0.9
     *
     * 减速段 [0.8, 1]: s = 1 - 2.5 * (1-tau)^2
     *   s(0.8) = 1 - 2.5 * 0.04 = 0.9 ✓
     */
    const float r = 0.2f;
    const float k = 2.5f;  /* 1/(2r) */
    float s;

    if (tau < r) {
        s = k * tau * tau;
    } else if (tau < 1.0f - r) {
        /* 匀速段: 从s(r)开始，斜率=(1-2*s(r))/(1-2r) */
        float s_r = k * r * r;  /* = 0.1 */
        float slope = (1.0f - 2.0f * s_r) / (1.0f - 2.0f * r);
        s = s_r + (tau - r) * slope;
    } else {
        float tau2 = 1.0f - tau;
        s = 1.0f - k * tau2 * tau2;
    }

    return x0 + (xf - x0) * s;
}

float traj_calc_min_duration(const float q_start[TRAJ_MAX_JOINTS],
                             const float q_end[TRAJ_MAX_JOINTS],
                             const traj_limits_t *limits)
{
    float max_duration = 0.0f;

    for (int i = 0; i < TRAJ_MAX_JOINTS; i++) {
        float delta = fabsf(q_end[i] - q_start[i]);
        float v_max = limits->max_vel[i];
        float a_max = limits->max_acc[i];

        /* 计算该关节所需最短时间 */
        float t_acc = v_max / a_max;
        float d_acc = 0.5f * a_max * t_acc * t_acc;

        float duration;
        if (2 * d_acc >= delta) {
            /* 三角形曲线 */
            duration = 2.0f * sqrtf(delta / a_max);
        } else {
            /* 梯形曲线 */
            duration = 2.0f * t_acc + (delta - 2 * d_acc) / v_max;
        }

        if (duration > max_duration) {
            max_duration = duration;
        }
    }

    return max_duration * 1000.0f;  /* 转换为毫秒 */
}

/* ========== 内部函数实现 ========== */

static void start_segment(traj_planner_t *planner)
{
    traj_waypoint_t *wp = &planner->waypoints[planner->current_waypoint];

    memcpy(planner->q_end, wp->q, sizeof(float) * TRAJ_MAX_JOINTS);
    planner->current_interp = wp->interp;
    planner->segment_step = 0;

    /* 计算步数 */
    float duration = wp->duration_ms;

    /* 如果启用速度限制，检查时间是否足够 */
    if (planner->use_limits) {
        float min_duration = traj_calc_min_duration(
            planner->q_start, planner->q_end, &planner->limits);
        if (duration < min_duration) {
            duration = min_duration;
        }
    }

    planner->segment_total_steps = (uint32_t)(duration / (float)planner->period_ms);
    if (planner->segment_total_steps < 1) {
        planner->segment_total_steps = 1;
    }
}

static float interpolate_joint(traj_interp_type_t interp, float t, float T,
                               float x0, float xf, float v_max, float a_max)
{
    switch (interp) {
        case TRAJ_INTERP_MINIMUM_JERK:
            return traj_minimum_jerk(t, T, x0, xf);

        case TRAJ_INTERP_TRAPEZOIDAL:
            return traj_trapezoidal(t, T, x0, xf, v_max, a_max);

        case TRAJ_INTERP_LINEAR:
        default:
            if (T <= 0) return xf;
            return x0 + (xf - x0) * (t / T);
    }
}


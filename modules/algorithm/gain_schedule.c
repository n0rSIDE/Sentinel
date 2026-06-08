/**
 * @file gain_schedule.c
 * @brief 增益调度模块 - 动态更新PID参数
 *
 * 核心思路: 增益调度只负责根据误差计算动态Kp/Ki/Kd，然后更新到PID实例中
 * PID计算由原有PID控制器负责，增益调度不参与PID计算
 */

#include "gain_schedule.h"
#include <math.h>

/* ==================== 内部函数 ==================== */

/**
 * @brief 3阶S曲线插值 (3t² - 2t³)
 */
static float s_curve_3rd_order(float t) {
    return t * t * (3.0f - 2.0f * t);
}

/**
 * @brief 5阶S曲线插值 (t³(6t² - 15t + 10))
 */
static float s_curve_5th_order(float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    return t3 * (6.0f * t2 - 15.0f * t + 10.0f);
}

/* ==================== 公共函数实现 ==================== */

float GainSchedule_Interpolate(float x, float x1, float x2,
                                float y1, float y2, InterpType_e type) {
    // 边界检查
    if (x <= x1) return y1;
    if (x >= x2) return y2;

    // 归一化到 [0, 1]
    float t = (x - x1) / (x2 - x1);

    // 根据插值类型计算
    float s;
    switch (type) {
        case INTERP_LINEAR:
            s = t;
            break;

        case INTERP_S_CURVE_3:
            s = s_curve_3rd_order(t);
            break;

        case INTERP_S_CURVE_5:
            s = s_curve_5th_order(t);
            break;

        default:
            s = t;
            break;
    }

    return y1 + (y2 - y1) * s;
}

void GainSchedule_Calculate(const GainScheduleParams_t *params, float error,
                            float *Kp_out, float *Ki_out, float *Kd_out) {
    float abs_error = fabsf(error);

    if (abs_error < params->error_threshold[0]) {
        // 区间1: 小误差
        *Kp_out = params->Kp_values[0];
        *Ki_out = params->Ki_values[0];
        *Kd_out = params->Kd_values[0];

    } else if (abs_error < params->error_threshold[1]) {
        // 区间2: 中小误差 (插值过渡)
        *Kp_out = GainSchedule_Interpolate(abs_error,
                                            params->error_threshold[0],
                                            params->error_threshold[1],
                                            params->Kp_values[0],
                                            params->Kp_values[1],
                                            params->interp_type);

        *Ki_out = GainSchedule_Interpolate(abs_error,
                                            params->error_threshold[0],
                                            params->error_threshold[1],
                                            params->Ki_values[0],
                                            params->Ki_values[1],
                                            params->interp_type);

        *Kd_out = GainSchedule_Interpolate(abs_error,
                                            params->error_threshold[0],
                                            params->error_threshold[1],
                                            params->Kd_values[0],
                                            params->Kd_values[1],
                                            params->interp_type);

    } else if (abs_error < params->error_threshold[2]) {
        // 区间3: 中大误差 (插值过渡)
        *Kp_out = GainSchedule_Interpolate(abs_error,
                                            params->error_threshold[1],
                                            params->error_threshold[2],
                                            params->Kp_values[1],
                                            params->Kp_values[2],
                                            params->interp_type);

        *Ki_out = GainSchedule_Interpolate(abs_error,
                                            params->error_threshold[1],
                                            params->error_threshold[2],
                                            params->Ki_values[1],
                                            params->Ki_values[2],
                                            params->interp_type);

        *Kd_out = GainSchedule_Interpolate(abs_error,
                                            params->error_threshold[1],
                                            params->error_threshold[2],
                                            params->Kd_values[1],
                                            params->Kd_values[2],
                                            params->interp_type);

    } else {
        // 区间4: 大误差
        *Kp_out = params->Kp_values[3];
        *Ki_out = params->Ki_values[3];
        *Kd_out = params->Kd_values[3];
    }
}

void GainSchedule_UpdatePIDParams(PIDInstance *pid,
                                   const GainScheduleParams_t *gain_params,
                                   float error) {
    // 计算动态增益
    float dynamic_Kp, dynamic_Ki, dynamic_Kd;
    GainSchedule_Calculate(gain_params, error, &dynamic_Kp, &dynamic_Ki, &dynamic_Kd);

    // 更新PID参数
    pid->Kp = dynamic_Kp;
    pid->Ki = dynamic_Ki;
    pid->Kd = dynamic_Kd;
}

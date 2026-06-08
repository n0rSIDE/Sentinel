/**
 * @file gain_schedule.h
 * @brief 增益调度模块 - 动态更新PID参数
 *
 * 核心思路: 增益调度只负责根据误差计算动态Kp/Ki/Kd，然后更新到PID实例中
 * PID计算由原有PID控制器负责，增益调度不参与PID计算
 */

#ifndef GAIN_SCHEDULE_H
#define GAIN_SCHEDULE_H

#include "stdint.h"
#include "controller.h"  // 包含原有PID定义

/* ==================== 插值类型定义 ==================== */
typedef enum {
    INTERP_LINEAR = 0,      // 线性插值
    INTERP_S_CURVE_3,       // 3阶S曲线 (3t² - 2t³)
    INTERP_S_CURVE_5        // 5阶S曲线 (t³(6t² - 15t + 10))
} InterpType_e;

/* ==================== 增益调度参数结构体 ==================== */
typedef struct {
    // 误差阈值 (从小到大排列)
    float error_threshold[3];   // [小误差, 中误差, 大误差]

    // 对应的PID参数 (4个区间: <阈值1, 阈值1-2, 阈值2-3, >阈值3)
    float Kp_values[4];
    float Ki_values[4];
    float Kd_values[4];

    // 插值类型
    InterpType_e interp_type;
} GainScheduleParams_t;

/* ==================== 函数声明 ==================== */

/**
 * @brief S曲线插值函数
 *
 * @param x 输入值
 * @param x1 输入范围起始
 * @param x2 输入范围结束
 * @param y1 输出范围起始
 * @param y2 输出范围结束
 * @param type 插值类型
 * @return float 插值结果
 */
float GainSchedule_Interpolate(float x, float x1, float x2,
                                float y1, float y2, InterpType_e type);

/**
 * @brief 根据误差计算动态增益
 *
 * @param params 增益调度参数
 * @param error 误差值
 * @param Kp_out 输出的Kp值
 * @param Ki_out 输出的Ki值
 * @param Kd_out 输出的Kd值
 */
void GainSchedule_Calculate(const GainScheduleParams_t *params, float error,
                            float *Kp_out, float *Ki_out, float *Kd_out);

/**
 * @brief 更新PID参数 - 根据误差动态调整Kp/Ki/Kd
 *
 * 核心函数: 计算动态增益，更新到PID实例中
 * 注意: 此函数只更新参数，不进行PID计算
 *
 * @param pid 原有PID实例指针
 * @param gain_params 增益调度参数
 * @param error 当前误差值
 */
void GainSchedule_UpdatePIDParams(PIDInstance *pid,
                                   const GainScheduleParams_t *gain_params,
                                   float error);

#endif // GAIN_SCHEDULE_H

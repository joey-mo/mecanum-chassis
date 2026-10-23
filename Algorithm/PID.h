/**
 * @file    PID.h
 * @brief   位置式速度环 PID。
 *
 * 控制律：
 *   output = Kp * e + Ki * sum(e) + Kd * (e - e_prev)
 *
 * 本模块按固定采样周期整定，不使用 dt。若调用周期变化，ki/kd 需要重新标定。
 */

#ifndef ALGORITHM_PID_H
#define ALGORITHM_PID_H

/** @brief 位置式速度环 PID 实例。 */
typedef struct
{
    /* ---- 可调参数 ---- */
    float kp; /**< 比例增益 */
    float ki; /**< 积分增益；<= 0 时不做积分 */
    float kd; /**< 微分增益 */

    float output_max; /**< 总输出上限 */
    float output_min; /**< 总输出下限 */

    /** 积分项输出限幅，用于抑制积分饱和 */
    float integral_output_max;
    float integral_output_min;

    /* ---- 运行时状态 ---- */
    float error;          /**< 当前误差 = 目标值 - 测量值 */
    float previous_error; /**< 上一次误差，用于计算差分 */
    float integral_sum;   /**< 误差累加值 */

    /* 分项和总输出，保留下来便于调试观察 */
    float proportional_output;
    float integral_output;
    float derivative_output;
    float output;
} PID_PositionTypeDef;

/**
 * @brief 初始化 PID 参数并清空运行状态。
 *
 * @param pid                  PID 实例。
 * @param kp                   比例增益。
 * @param ki                   积分增益。
 * @param kd                   微分增益。
 * @param output_max           总输出上限。
 * @param output_min           总输出下限。
 * @param integral_output_max  积分项输出上限。
 * @param integral_output_min  积分项输出下限。
 */
void PID_Position_Init(PID_PositionTypeDef *pid,
                       float kp,
                       float ki,
                       float kd,
                       float output_max,
                       float output_min,
                       float integral_output_max,
                       float integral_output_min);

/**
 * @brief 执行一次位置式 PID 计算。
 *
 * 调用周期应保持固定；本函数不接收 dt，也不做积分分离。
 *
 * @param pid         PID 实例；为 NULL 时返回 0。
 * @param target      目标值。
 * @param measurement 测量值。
 * @return 限幅后的输出。
 */
float PID_Position_Calculate(PID_PositionTypeDef *pid,
                             float target,
                             float measurement);

/**
 * @brief 清空误差、积分和分项输出，保留可调参数。
 *
 * @param pid PID 实例；为 NULL 时直接返回。
 */
void PID_Position_Reset(PID_PositionTypeDef *pid);

#endif /* ALGORITHM_PID_H */

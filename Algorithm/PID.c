/**
 * @file    PID.c
 * @brief   位置式速度环 PID 实现。
 *
 * 说明：
 *   - 增益按 1kHz 控制周期整定，本模块不使用 dt。
 *   - 积分限幅直接作用在误差累加值上，而不是只截断积分项输出。
 *     否则误差长期存在时累加值会持续增长，反向时需要很久才能退出饱和。
 */

#include "PID.h"

#include <stddef.h>

/**
 * @brief 把数值限制在 [minimum, maximum] 内。
 *
 * 注意比较顺序：先判断上限，再判断下限，最后返回原值。
 */
static float PID_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

void PID_Position_Init(PID_PositionTypeDef *pid,
                       float kp,
                       float ki,
                       float kd,
                       float output_max,
                       float output_min,
                       float integral_output_max,
                       float integral_output_min)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->integral_output_max = integral_output_max;
    pid->integral_output_min = integral_output_min;

    PID_Position_Reset(pid);
}

float PID_Position_Calculate(PID_PositionTypeDef *pid,
                             float target,
                             float measurement)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error = target - measurement;

    /* 比例项 */
    pid->proportional_output = pid->kp * pid->error;

    /*
     * 积分项：先把累加值限制在“积分输出限幅 / ki”以内，
     * 这样即使误差长期存在，累加值也不会无限增长。
     */
    if (pid->ki > 0.0f)
    {
        float sum_max;
        float sum_min;

        pid->integral_sum += pid->error;

        sum_max = pid->integral_output_max / pid->ki;
        sum_min = pid->integral_output_min / pid->ki;

        pid->integral_sum = PID_Clamp(pid->integral_sum, sum_min, sum_max);
    }
    else
    {
        /* ki <= 0 时不积分，清掉可能残留的累加值 */
        pid->integral_sum = 0.0f;
    }

    pid->integral_output = pid->ki * pid->integral_sum;
    pid->integral_output = PID_Clamp(pid->integral_output,
                                     pid->integral_output_min,
                                     pid->integral_output_max);

    /* 微分作用在误差上，目标突变时会产生冲击，调参时需注意 */
    pid->derivative_output =
        pid->kd * (pid->error - pid->previous_error);

    pid->output = pid->proportional_output +
                  pid->integral_output +
                  pid->derivative_output;

    pid->output = PID_Clamp(pid->output,
                            pid->output_min,
                            pid->output_max);

    pid->previous_error = pid->error;

    return pid->output;
}

void PID_Position_Reset(PID_PositionTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error = 0.0f;
    pid->previous_error = 0.0f;
    pid->integral_sum = 0.0f;

    pid->proportional_output = 0.0f;
    pid->integral_output = 0.0f;
    pid->derivative_output = 0.0f;
    pid->output = 0.0f;
}

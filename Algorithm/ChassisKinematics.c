/**
 * @file    ChassisKinematics.c
 * @brief   四轮麦克纳姆底盘逆运动学实现。
 */

#include "ChassisKinematics.h"

#include <stddef.h>

#include "ChassisConfig.h"

/** @brief 求浮点数绝对值，避免为一个简单运算引入 libm。 */
static float ChassisKinematics_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void ChassisKinematics_Solve(const ChassisVelocityTypeDef *velocity,
                             float max_wheel_rpm,
                             ChassisWheelRpmTypeDef *target)
{
    float half_span;
    float rpm_per_m_s;
    float max_abs_rpm;
    uint8_t i;

    if ((velocity == NULL) || (target == NULL))
    {
        return;
    }

    /* 自转力臂：半轴距与半轮距之和 */
    half_span = CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M;

    /* 轮子线速度 m/s 换算为输出轴 rpm：60 / (2 * pi * r) */
    rpm_per_m_s = 60.0f /
                  (2.0f * CHASSIS_PI * CHASSIS_WHEEL_RADIUS_M);

    /*
     * 逐轮计算输出轴转速。公式中的符号与轮位绑定；若改变轮序，
     * 必须同步修改这里的对应关系，不能只改电机映射。
     */
    target->wheel_rpm[CHASSIS_WHEEL_FL] =
        (velocity->vx_m_s - velocity->vy_m_s -
         half_span * velocity->wz_rad_s) * rpm_per_m_s;

    target->wheel_rpm[CHASSIS_WHEEL_FR] =
        (velocity->vx_m_s + velocity->vy_m_s +
         half_span * velocity->wz_rad_s) * rpm_per_m_s;

    target->wheel_rpm[CHASSIS_WHEEL_RR] =
        (velocity->vx_m_s - velocity->vy_m_s +
         half_span * velocity->wz_rad_s) * rpm_per_m_s;

    target->wheel_rpm[CHASSIS_WHEEL_RL] =
        (velocity->vx_m_s + velocity->vy_m_s -
         half_span * velocity->wz_rad_s) * rpm_per_m_s;

    if (max_wheel_rpm <= 0.0f)
    {
        return;
    }

    /* 找出绝对值最大的轮速，作为缩放依据 */
    max_abs_rpm = 0.0f;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        float magnitude = ChassisKinematics_Abs(target->wheel_rpm[i]);

        if (magnitude > max_abs_rpm)
        {
            max_abs_rpm = magnitude;
        }
    }

    /*
     * 组合运动（斜移 + 自转）可能让某一轮超过额定轮速。
     * 四轮使用同一缩放系数，保证轮速矢量方向不变。
     */
    if (max_abs_rpm > max_wheel_rpm)
    {
        float scale = max_wheel_rpm / max_abs_rpm;

        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            target->wheel_rpm[i] *= scale;
        }
    }
}

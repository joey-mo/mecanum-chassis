/**
 * @file    ChassisKinematics.h
 * @brief   四轮麦克纳姆底盘逆运动学。
 *
 * 坐标约定：
 *   +x 车头向前，+y 车体向左，+wz 俯视逆时针。
 *   各轮正方向均为驱动车体向 +x。
 */

#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdint.h>

/**
 * @brief 轮序定义。
 *
 * 该顺序必须与电机数组一致：[FL, FR, RR, RL] = [ID1, ID2, ID3, ID4]。
 */
typedef enum
{
    CHASSIS_WHEEL_FL = 0, /**< 左前轮 */
    CHASSIS_WHEEL_FR,     /**< 右前轮 */
    CHASSIS_WHEEL_RR,     /**< 右后轮 */
    CHASSIS_WHEEL_RL,     /**< 左后轮 */
    CHASSIS_WHEEL_COUNT   /**< 轮子数量 */
} ChassisWheelIdTypeDef;

/** @brief 车体坐标系下的目标速度。 */
typedef struct
{
    float vx_m_s;   /**< 前进速度，m/s，向前为正 */
    float vy_m_s;   /**< 横移速度，m/s，向左为正 */
    float wz_rad_s; /**< 自转角速度，rad/s，逆时针为正 */
} ChassisVelocityTypeDef;

/** @brief 四轮输出轴目标转速，单位 rpm。 */
typedef struct
{
    float wheel_rpm[CHASSIS_WHEEL_COUNT];
} ChassisWheelRpmTypeDef;

/**
 * @brief 将车体速度分解为四轮输出轴转速。
 *
 * 逆运动学公式：
 *   w_FL = ( vx - vy - (L + W) * wz ) / r
 *   w_FR = ( vx + vy + (L + W) * wz ) / r
 *   w_RR = ( vx - vy + (L + W) * wz ) / r
 *   w_RL = ( vx + vy - (L + W) * wz ) / r
 *
 * 其中 L、W 分别为半轴距和半轮距，r 为轮半径。输出为输出轴 rpm；
 * 电机侧还需要乘以减速比和安装方向。
 *
 * 当 max_wheel_rpm > 0 且任一轮超速时，四轮按同一比例缩放，
 * 以保持合成运动方向不变。
 *
 * @param velocity      车体目标速度，不能为 NULL。
 * @param max_wheel_rpm 输出轴允许的最大转速；<= 0 表示不做缩放。
 * @param target        四轮目标转速输出，不能为 NULL。
 */
void ChassisKinematics_Solve(const ChassisVelocityTypeDef *velocity,
                             float max_wheel_rpm,
                             ChassisWheelRpmTypeDef *target);

#endif /* CHASSIS_KINEMATICS_H */

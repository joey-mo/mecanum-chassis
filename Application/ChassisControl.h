/**
 * @file    ChassisControl.h
 * @brief   底盘状态机与四轮速度环控制接口。
 */

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>

/** @brief 底盘运行状态。 */
typedef enum
{
    CHASSIS_STATE_INIT = 0, /**< 初始化：等待 DBus、CAN 反馈和摇杆回中 */
    CHASSIS_STATE_RUN,      /**< 运行：接收遥控并驱动四轮 */
    CHASSIS_STATE_FAULT     /**< 故障：输出零电流并等待恢复条件 */
} ChassisStateTypeDef;

/**
 * @brief 初始化底盘控制状态机、速度指令和四轮 PID。
 */
void ChassisControl_Init(void);

/**
 * @brief 执行一次底盘控制周期。
 *
 * 典型链路：读取 DBus -> 映射摇杆 -> 逆运动学 -> 四轮速度环 -> 发送 0x200。
 * 应在主循环中以 1kHz 调用。
 */
void ChassisControl_Step(void);

/**
 * @brief 获取当前底盘状态。
 *
 * @return 当前状态机状态。
 */
ChassisStateTypeDef ChassisControl_GetState(void);

#endif /* CHASSIS_CONTROL_H */

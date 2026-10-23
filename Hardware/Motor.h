/**
 * @file    Motor.h
 * @brief   M3508 电机 + C620 电调的反馈、在线检测和速度环状态。
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#include "Bsp_can.h"
#include "PID.h"

/** @brief C620 反馈报文解码结果。 */
typedef struct
{
    uint16_t rotor_angle_raw;   /**< 转子机械角，0~8191 */
    int16_t rotor_speed_rpm;    /**< 转子转速，rpm */
    int16_t torque_current_raw; /**< 实际转矩电流，raw */
    uint8_t temperature_c;      /**< 电机温度，摄氏度 */
} DJI3508_FeedbackTypeDef;

/** @brief 单台 3508 的目标值、输出和速度环。 */
typedef struct
{
    float target_speed_rpm;        /**< 目标转子转速，rpm */
    int16_t current_command_raw;   /**< 电流指令，raw */
    PID_PositionTypeDef speed_pid; /**< 速度环 PID */
} DJI3508_ControlTypeDef;

/** @brief 单台 3508 的完整状态。 */
typedef struct
{
    uint8_t id; /**< C620 电调 ID，1~4 */

    DJI3508_ControlTypeDef control;

    /*
     * 反馈采用双缓冲：ISR 写非活动缓冲区，控制循环读活动缓冲区，
     * 避免控制层读到只更新了一半的报文。
     */
    volatile DJI3508_FeedbackTypeDef feedback[2];
    volatile uint8_t active_feedback_index;

    /* 在线检测 */
    volatile uint32_t last_feedback_tick; /**< 最近一帧反馈的时间戳 */
    volatile uint8_t feedback_valid;      /**< 是否至少收到过一帧反馈 */
} DJI3508_MotorTypeDef;

/** @brief 四台 3508 组成一个电机组，共用一帧 0x200 控制报文。 */
typedef struct
{
    BSP_CAN_TxFrameTypeDef tx_frame;
    DJI3508_MotorTypeDef motors[DJI_MOTOR_COUNT];
} DJI3508_MotorGroupTypeDef;

/** @brief 底盘电机组：motors[0]=FL/ID1，[1]=FR/ID2，[2]=RR/ID3，[3]=RL/ID4。 */
extern DJI3508_MotorGroupTypeDef DJI3508_ChassisMotorGroup;

/**
 * @brief 初始化电机组（电调 ID、CAN 发送帧和反馈状态）。
 */
void DJI3508_MotorGroupInit(void);

/**
 * @brief 获取当前已完整发布的反馈快照。
 *
 * @param motor 电机实例，不能为 NULL。
 * @return 指向活动反馈缓冲区的只读指针；motor 为 NULL 时返回 NULL。
 */
const volatile DJI3508_FeedbackTypeDef *DJI3508_GetFeedback(
    const DJI3508_MotorTypeDef *motor);

/**
 * @brief 检查四台电机是否都在超时窗口内收到过反馈。
 *
 * @param group  电机组。
 * @param now_ms 当前时间戳，单位 ms。
 * @return 1 表示全部在线，0 表示至少一台离线。
 */
uint8_t DJI3508_GroupIsOnline(const DJI3508_MotorGroupTypeDef *group,
                              uint32_t now_ms);

/**
 * @brief 解码一帧 C620 反馈并发布到控制层。
 *
 * 由 CAN 接收中断调用；新数据先写入非活动缓冲区，全部字段写完后
 * 再切换活动索引。
 *
 * @param motor  电机实例。
 * @param data   8 字节反馈数据。
 * @param now_ms 接收时刻，单位 ms。
 */
void DJI3508_FeedbackPublish(DJI3508_MotorTypeDef *motor,
                             const uint8_t data[8],
                             uint32_t now_ms);

#endif /* MOTOR_H */

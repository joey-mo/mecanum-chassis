/**
 * @file    Motor.c
 * @brief   M3508 电机 + C620 电调的反馈处理与在线检测。
 *
 * 电机映射：
 *   motors[0..3] = FL / FR / RR / RL = 电调 ID 1~4。
 *
 * 反馈流程：
 *   CAN 接收中断把新一帧写入 feedback[非活动区]，字段全部写完后
 *   再切换 active_feedback_index；控制层通过 DJI3508_GetFeedback()
 *   读取完整快照。
 *
 * 在线检测：
 *   每帧更新 last_feedback_tick 并置 feedback_valid；超过
 *   MOTOR_FEEDBACK_TIMEOUT_MS 没有新帧即认为该电机离线。
 */

#include "Motor.h"

#include <stddef.h>
#include <string.h>

#include "can.h"
#include "ChassisConfig.h"

DJI3508_MotorGroupTypeDef DJI3508_ChassisMotorGroup;

/** @brief 按大端顺序读取一个 16 位字。 */
static uint16_t DJI3508_ReadUint16Be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

void DJI3508_MotorGroupInit(void)
{
    uint8_t i;

    /* 四台 C620 共用 0x200 控制帧，挂在 CAN1 */
    DJI3508_ChassisMotorGroup.tx_frame.hcan = &hcan1;
    DJI3508_ChassisMotorGroup.tx_frame.header.StdId = DJI_MOTOR_CMD_STD_ID;
    DJI3508_ChassisMotorGroup.tx_frame.header.IDE = CAN_ID_STD;
    DJI3508_ChassisMotorGroup.tx_frame.header.RTR = CAN_RTR_DATA;
    DJI3508_ChassisMotorGroup.tx_frame.header.DLC = 8U;

    memset(DJI3508_ChassisMotorGroup.tx_frame.data,
           0,
           sizeof(DJI3508_ChassisMotorGroup.tx_frame.data));

    for (i = 0U; i < DJI_MOTOR_COUNT; i++)
    {
        DJI3508_MotorTypeDef *motor = &DJI3508_ChassisMotorGroup.motors[i];

        motor->id = (uint8_t)(i + 1U);
        motor->control.target_speed_rpm = 0.0f;
        motor->control.current_command_raw = 0;

        motor->active_feedback_index = 0U;
        motor->last_feedback_tick = 0U;
        motor->feedback_valid = 0U;

        /*
         * speed_pid 的增益和运行状态由 ChassisControl_Init() 统一初始化，
         * 硬件层不保存调参值。
         */
    }
}

const volatile DJI3508_FeedbackTypeDef *DJI3508_GetFeedback(
    const DJI3508_MotorTypeDef *motor)
{
    uint8_t index;

    if (motor == NULL)
    {
        return NULL;
    }

    index = motor->active_feedback_index;

    return &motor->feedback[index];
}

/**
 * @brief 判断单台电机的反馈是否仍在超时窗口内。
 */
static uint8_t DJI3508_IsFeedbackFresh(const DJI3508_MotorTypeDef *motor,
                                       uint32_t now_ms)
{
    int32_t age;

    if (motor->feedback_valid == 0U)
    {
        return 0U;
    }

    /*
     * 使用带符号差值：控制层读取 now_ms 后，CAN 中断可能把
     * last_feedback_tick 更新到 now_ms 之后，无符号相减会回绕。
     */
    age = (int32_t)(now_ms - motor->last_feedback_tick);

    if (age < 0)
    {
        age = 0;
    }

    return (age <= (int32_t)MOTOR_FEEDBACK_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t DJI3508_GroupIsOnline(const DJI3508_MotorGroupTypeDef *group,
                              uint32_t now_ms)
{
    uint8_t i;

    if (group == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < DJI_MOTOR_COUNT; i++)
    {
        if (DJI3508_IsFeedbackFresh(&group->motors[i], now_ms) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

void DJI3508_FeedbackPublish(DJI3508_MotorTypeDef *motor,
                             const uint8_t data[8],
                             uint32_t now_ms)
{
    uint8_t next_index;
    volatile DJI3508_FeedbackTypeDef *feedback;

    if ((motor == NULL) || (data == NULL))
    {
        return;
    }

    /* 写非活动缓冲区，全部字段写完后才发布索引 */
    next_index = (uint8_t)((motor->active_feedback_index == 0U) ? 1U : 0U);
    feedback = &motor->feedback[next_index];

    feedback->rotor_angle_raw = DJI3508_ReadUint16Be(&data[0]);
    feedback->rotor_speed_rpm = (int16_t)DJI3508_ReadUint16Be(&data[2]);
    feedback->torque_current_raw = (int16_t)DJI3508_ReadUint16Be(&data[4]);
    feedback->temperature_c = data[6];

    /* data[7] 在 C620 协议中保留，忽略 */

    motor->last_feedback_tick = now_ms;
    motor->feedback_valid = 1U;
    motor->active_feedback_index = next_index;
}

/**
 * @file    RcMonitor.c
 * @brief   遥控、CAN 诊断和电机数据通过 USART6 输出。
 *
 * 主循环每 1ms 调用一次，内部限频到 50Hz。发送走 USART6 中断：
 * 启动后立即返回，1kHz 控制循环不会被十几毫秒的串口发送拖住。
 */

#include "RcMonitor.h"

#include <stdio.h>

#include "Bsp_can.h"
#include "Bsp_dbus.h"
#include "can.h"
#include "ChassisControl.h"
#include "Motor.h"
#include "usart.h"

/* 上一帧启动发送的时刻，以及发送是否仍在进行 */
static uint32_t s_last_send_tick;
static volatile uint8_t s_tx_busy;

/* 必须为静态存储：HAL_UART_Transmit_IT() 返回后仍会从这块缓冲区取数据 */
static char s_tx_buffer[RC_MONITOR_TX_BUFFER_SIZE];

/** @brief 发送完成或出错时释放缓冲区。 */
static void RcMonitor_ReleaseTx(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        s_tx_busy = 0U;
    }
}

void RcMonitor_Task(void)
{
    uint32_t now_ms;
    const volatile rc_info_t *rc;

    uint32_t can_rx;
    uint32_t can_last_id;
    uint32_t can_esr;
    uint32_t can_fifo;
    uint8_t can_valid_mask;
    uint8_t can_online;
    uint8_t dbus_online;
    uint8_t i;
    int target_rpm[DJI_MOTOR_COUNT];
    int actual_rpm[DJI_MOTOR_COUNT];
    int current_raw[DJI_MOTOR_COUNT];
    int len;

    now_ms = HAL_GetTick();

    if ((now_ms - s_last_send_tick) < RC_MONITOR_PERIOD_MS)
    {
        return;
    }

    /* 上一帧还没发完就等下一毫秒再试，避免覆盖发送缓冲区 */
    if (s_tx_busy != 0U)
    {
        return;
    }

    rc = DBUS_GetFeedback();

    if (rc == NULL)
    {
        return;
    }

    can_rx = BSP_CAN_GetRxCount();
    can_last_id = BSP_CAN_GetLastStdId();

    /* ESR 包含 TEC/REC/BOFF/LEC；RF0R 低 2 位是 FIFO0 待处理帧数 */
    can_esr = hcan1.Instance->ESR;
    can_fifo = hcan1.Instance->RF0R & 0x03U;

    /* 四台电机反馈合成一个位掩码，比四路 0/1 更容易观察 */
    can_valid_mask = 0U;

    for (i = 0U; i < DJI_MOTOR_COUNT; i++)
    {
        if (DJI3508_ChassisMotorGroup.motors[i].feedback_valid != 0U)
        {
            can_valid_mask |= (uint8_t)(1U << i);
        }
    }

    can_online = DJI3508_GroupIsOnline(&DJI3508_ChassisMotorGroup, now_ms);
    dbus_online = DBUS_IsOnline(now_ms);

    /* 按电调 ID1~ID4 的顺序收集目标、实际和电流指令 */
    for (i = 0U; i < DJI_MOTOR_COUNT; i++)
    {
        const volatile DJI3508_FeedbackTypeDef *feedback =
            DJI3508_GetFeedback(&DJI3508_ChassisMotorGroup.motors[i]);

        target_rpm[i] =
            (int)DJI3508_ChassisMotorGroup.motors[i].control.target_speed_rpm;
        actual_rpm[i] = (int)feedback->rotor_speed_rpm;
        current_raw[i] =
            (int)DJI3508_ChassisMotorGroup.motors[i].control.current_command_raw;
    }

    len = snprintf(s_tx_buffer,
                   sizeof(s_tx_buffer),
                   "%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,"
                   "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   (int)rc->ch0,
                   (int)rc->ch1,
                   (int)rc->ch2,
                   (int)rc->ch3,
                   (int)rc->sw1,
                   (int)rc->sw2,
                   (unsigned int)can_rx,
                   (unsigned int)can_last_id,
                   (unsigned int)can_esr,
                   (unsigned int)can_fifo,
                   (unsigned int)can_valid_mask,
                   (unsigned int)can_online,
                   (unsigned int)dbus_online,
                   (unsigned int)ChassisControl_GetState(),
                   target_rpm[0], actual_rpm[0], current_raw[0],
                   target_rpm[1], actual_rpm[1], current_raw[1],
                   target_rpm[2], actual_rpm[2], current_raw[2],
                   target_rpm[3], actual_rpm[3], current_raw[3]);

    if (len <= 0)
    {
        return;
    }

    /* snprintf 返回值可能大于缓冲区长度，按实际可发送长度截断 */
    if (len >= (int)sizeof(s_tx_buffer))
    {
        len = (int)sizeof(s_tx_buffer) - 1;
    }

    s_tx_busy = 1U;

    if (HAL_UART_Transmit_IT(&huart6,
                             (uint8_t *)s_tx_buffer,
                             (uint16_t)len) != HAL_OK)
    {
        s_tx_busy = 0U;
        return;
    }

    /* 只有真正启动发送才更新周期基准；失败时下一毫秒重试 */
    s_last_send_tick = now_ms;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    RcMonitor_ReleaseTx(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* 发送出错也必须释放，否则监视器会永久停发 */
    RcMonitor_ReleaseTx(huart);
}

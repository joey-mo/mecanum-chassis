/**
 * @file    Bsp_can.c
 * @brief   CAN1 初始化与 C620 电机报文收发实现。
 *
 * 控制帧：0x200，包含 4 个 int16 电流，高字节在前，对应电调 ID1~ID4。
 * 反馈帧：0x201~0x204，对应 ID1~ID4；负载解析见 DJI3508_FeedbackPublish()。
 *
 * 接收策略：
 *   过滤器先全部放行，软件再按 StdId 分类。当前总线只有 4 台 C620，
 *   这样调试最直接；后续总线设备增多时，可以再收紧硬件过滤器。
 */

#include "Bsp_can.h"

#include <stddef.h>

#include "Motor.h"

/* 调试计数：FIFO0 实际读出的帧数（含非 C620 帧）与最后一帧 StdId */
static volatile uint32_t s_rx_count;
static volatile uint32_t s_last_std_id;

/**
 * @brief 按大端顺序写入一个 int16_t。
 *
 * C620 协议规定高字节在前；先转成 uint16_t，避免对负数右移。
 */
static void BSP_CAN_WriteInt16Be(uint8_t *destination, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    destination[0] = (uint8_t)(raw >> 8);
    destination[1] = (uint8_t)raw;
}

HAL_StatusTypeDef BSP_CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    if ((hcan == NULL) || (hcan->Instance != CAN1))
    {
        return HAL_ERROR;
    }

    /* 32 位掩码模式：掩码全 0，放行所有标准帧到 FIFO0 */
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BSP_CAN_SendMotorCurrents(BSP_CAN_TxFrameTypeDef *frame,
                                            int16_t current_1,
                                            int16_t current_2,
                                            int16_t current_3,
                                            int16_t current_4)
{
    static uint32_t s_tx_mailbox;

    if ((frame == NULL) || (frame->hcan == NULL))
    {
        return HAL_ERROR;
    }

    BSP_CAN_WriteInt16Be(&frame->data[0], current_1);
    BSP_CAN_WriteInt16Be(&frame->data[2], current_2);
    BSP_CAN_WriteInt16Be(&frame->data[4], current_3);
    BSP_CAN_WriteInt16Be(&frame->data[6], current_4);

    return HAL_CAN_AddTxMessage(frame->hcan,
                                &frame->header,
                                frame->data,
                                &s_tx_mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return;
    }

    /* 一次中断排空 FIFO0，避免 4 帧/ms 时 3 级 FIFO 溢出 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef header = {0};
        uint8_t data[8] = {0};
        uint8_t index;

        if (HAL_CAN_GetRxMessage(hcan,
                                 CAN_RX_FIFO0,
                                 &header,
                                 data) != HAL_OK)
        {
            break;
        }

        /* 先记录计数，再按 ID 分类，以区分“没帧”和“有帧但 ID 不匹配” */
        s_rx_count++;
        s_last_std_id = header.StdId;

        /* 只处理底盘电机组所在的那条 CAN 总线 */
        if (hcan != DJI3508_ChassisMotorGroup.tx_frame.hcan)
        {
            continue;
        }

        if ((header.StdId < DJI_MOTOR_FEEDBACK_FIRST_STD_ID) ||
            (header.StdId > DJI_MOTOR_FEEDBACK_LAST_STD_ID))
        {
            continue;
        }

        index = (uint8_t)(header.StdId - DJI_MOTOR_FEEDBACK_FIRST_STD_ID);

        if (index >= DJI_MOTOR_COUNT)
        {
            continue;
        }

        DJI3508_FeedbackPublish(&DJI3508_ChassisMotorGroup.motors[index],
                                data,
                                HAL_GetTick());
    }
}

uint32_t BSP_CAN_GetRxCount(void)
{
    return s_rx_count;
}

uint32_t BSP_CAN_GetLastStdId(void)
{
    return s_last_std_id;
}

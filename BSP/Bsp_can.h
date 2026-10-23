/**
 * @file    Bsp_can.h
 * @brief   CAN1 初始化与 C620 电机报文收发。
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>

#include "main.h"

/* C620：ID1~ID4 共用 0x200 控制帧，反馈帧为 0x201~0x204 */
#define DJI_MOTOR_CMD_STD_ID             (0x200U)
#define DJI_MOTOR_FEEDBACK_FIRST_STD_ID  (0x201U)
#define DJI_MOTOR_FEEDBACK_LAST_STD_ID   (0x204U)
#define DJI_MOTOR_COUNT                  (4U)

/** @brief 通用 CAN 发送帧：句柄、帧头和 8 字节数据。 */
typedef struct
{
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} BSP_CAN_TxFrameTypeDef;

/**
 * @brief 配置 CAN1 过滤器、启动外设并打开 RX0 接收通知。
 *
 * @param hcan CAN 句柄，当前仅支持 CAN1。
 * @return HAL_OK 表示初始化成功。
 */
HAL_StatusTypeDef BSP_CAN_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief 将四路电流指令打包成 0x200 帧并发送。
 *
 * 四个参数依次对应电调 ID1~ID4，单位均为 C620 电流 raw。
 *
 * @param frame     发送帧配置。
 * @param current_1 电调 ID1 电流。
 * @param current_2 电调 ID2 电流。
 * @param current_3 电调 ID3 电流。
 * @param current_4 电调 ID4 电流。
 * @return HAL 发送结果。
 */
HAL_StatusTypeDef BSP_CAN_SendMotorCurrents(BSP_CAN_TxFrameTypeDef *frame,
                                            int16_t current_1,
                                            int16_t current_2,
                                            int16_t current_3,
                                            int16_t current_4);

/**
 * @brief 获取 CAN1 FIFO0 累计读出的帧数（含非 C620 帧）。
 *
 * 调试提示：
 *   为 0 时，检查供电、模式、接线、终端电阻和引脚；
 *   在增长时，检查 ID 范围、回调分类和电机映射。
 * 同时可观察 hcan1.Instance->ESR / RF0R，区分总线错误和 FIFO 状态。
 *
 * @return 累计读出的 CAN 帧数。
 */
uint32_t BSP_CAN_GetRxCount(void);

/**
 * @brief 获取最后一帧的 StdId。
 *
 * @return 最后一帧的标准 ID；0 表示尚未读到过帧。
 */
uint32_t BSP_CAN_GetLastStdId(void);

#endif /* BSP_CAN_H */

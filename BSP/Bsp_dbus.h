/**
 * @file    Bsp_dbus.h
 * @brief   DJI DBus 遥控接收（USART3 + 循环 DMA + IDLE 中断）。
 */

#ifndef BSP_DBUS_H
#define BSP_DBUS_H

#include <stdint.h>

#include "main.h"

/** @brief DBus 一帧固定为 18 字节。 */
#define DBUS_FRAME_LENGTH        (18U)

/** @brief 循环 DMA 缓冲区大小：5 帧，约 70ms 余量。 */
#define DBUS_DMA_BUFFER_SIZE     (90U)

/** @brief 解析后的遥控器状态；摇杆值已减去中点 1024。 */
typedef struct
{
    int16_t ch0;    /**< 右摇杆水平 */
    int16_t ch1;    /**< 右摇杆垂直 */
    int16_t ch2;    /**< 左摇杆水平 */
    int16_t ch3;    /**< 左摇杆垂直 */
    uint8_t sw1;    /**< 右拨杆，1/2/3 */
    uint8_t sw2;    /**< 左拨杆，1/2/3 */
} rc_info_t;

/**
 * @brief 启动 USART3 循环 DMA + IDLE 接收。
 *
 * @param huart UART 句柄，当前仅支持 USART3。
 * @return HAL_OK 表示启动成功。
 */
HAL_StatusTypeDef DBUS_Init(UART_HandleTypeDef *huart);

/**
 * @brief 处理 USART3 IDLE 中断并发布完整遥控帧。
 *
 * 必须在 USART3_IRQHandler 中、HAL_UART_IRQHandler 之前调用。
 *
 * @param huart UART 句柄。
 */
void DBUS_RxIdleHandler(UART_HandleTypeDef *huart);

/**
 * @brief 获取当前已完整发布的遥控器状态。
 *
 * 双缓冲保证中断只写非活动区，控制层不会读到半帧数据。
 *
 * @return 指向活动遥控状态的只读指针。
 */
const volatile rc_info_t *DBUS_GetFeedback(void);

/**
 * @brief 判断最近一帧有效 DBus 是否仍在超时窗口内。
 *
 * @param now_ms 当前时间戳，单位 ms。
 * @return 1 表示在线，0 表示离线或尚未收到有效帧。
 */
uint8_t DBUS_IsOnline(uint32_t now_ms);

/**
 * @brief 获取累计解析成功的 DBus 帧数。
 */
uint32_t DBUS_GetFrameCount(void);

/**
 * @brief 获取累计丢弃的异常 DBus 帧数。
 */
uint32_t DBUS_GetErrorCount(void);

#endif /* BSP_DBUS_H */

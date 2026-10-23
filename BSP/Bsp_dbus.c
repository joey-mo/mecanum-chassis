/**
 * @file    Bsp_dbus.c
 * @brief   DJI DBus 遥控接收与解析。
 *
 * 接收方案：
 *   90 字节循环 DMA 持续搬运，USART3 空闲中断表示一帧结束。
 *   DBus 帧没有帧头，也没有校验，无法靠扫描帧头同步，IDLE 是唯一的
 *   帧边界；循环 DMA 不需要每次重启，漏掉一次 IDLE 也不会让接收停摆。
 *
 * 取帧方式：
 *   IDLE 时用 DMA 剩余计数（NDTR）反推写指针，取写指针之前的最后
 *   18 字节；若这 18 字节跨过缓冲区首尾，则分两段拼接。
 *
 * 发布方式：
 *   解析结果先写入 s_rc[2] 的非活动缓冲区，整帧写完后切换 active_index；
 *   控制层通过 DBUS_GetFeedback() 读取完整快照，避免读到半帧。
 *
 * 异常帧：
 *   通道超出 ±660 时丢弃该帧并计数，保留上一帧有效数据；
 *   连续丢帧由 DBUS_IsOnline() 的超时统一处理，避免单帧噪声造成目标跳变。
 */

#include "Bsp_dbus.h"

#include <stddef.h>
#include <string.h>

#include "ChassisConfig.h"

/* DBus 通道：11 位，中点 1024，实际满杆约 ±660 */
#define DBUS_CHANNEL_CENTER      (1024)
#define DBUS_CHANNEL_LIMIT       (660)
#define DBUS_CHANNEL_MASK        (0x07FFU)

/* DBus 拨杆：2 位；方案 B 不使用拨杆，因此只解析、不校验 */
#define DBUS_SWITCH_MASK         (0x03U)

/* 循环 DMA 缓冲区；DMA 全程写入，CPU 只在 IDLE 时读取 */
static uint8_t s_dma_buffer[DBUS_DMA_BUFFER_SIZE];

/* 解析结果双缓冲：中断写非活动区，控制层读活动区 */
static rc_info_t s_rc[2];
static volatile uint8_t s_active_index;

/* 在线检测与调试计数 */
static volatile uint32_t s_last_frame_tick;
static volatile uint32_t s_frame_count;
static volatile uint32_t s_error_count;
static volatile uint8_t s_has_valid_frame;

/**
 * @brief 判断摇杆通道值是否在可信范围内。
 */
static uint8_t DBUS_IsChannelInRange(int16_t value)
{
    return (uint8_t)((value >= -DBUS_CHANNEL_LIMIT) &&
                     (value <= DBUS_CHANNEL_LIMIT));
}

/**
 * @brief 将一帧 18 字节数据解析为遥控器状态。
 *
 * @param frame 原始帧数据。
 * @param out   解析结果输出。
 * @return 1 表示解析成功，0 表示该帧不可信。
 */
static uint8_t DBUS_DecodeFrame(const uint8_t frame[DBUS_FRAME_LENGTH],
                                rc_info_t *out)
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t sw1;
    uint8_t sw2;

    if (out == NULL)
    {
        return 0U;
    }

    /*
     * 通道位域（小端）：
     *   ch0 = b0[7:0] + b1[2:0]
     *   ch1 = b1[7:3] + b2[5:0]
     *   ch2 = b2[7:6] + b3[7:0] + b4[0]
     *   ch3 = b4[7:1] + b5[6:0]
     */
    ch0 = (int16_t)(((uint16_t)frame[0] |
                     ((uint16_t)frame[1] << 8)) & DBUS_CHANNEL_MASK);

    ch1 = (int16_t)((((uint16_t)frame[1] >> 3) |
                     ((uint16_t)frame[2] << 5)) & DBUS_CHANNEL_MASK);

    ch2 = (int16_t)((((uint16_t)frame[2] >> 6) |
                     ((uint16_t)frame[3] << 2) |
                     ((uint16_t)frame[4] << 10)) & DBUS_CHANNEL_MASK);

    ch3 = (int16_t)((((uint16_t)frame[4] >> 1) |
                     ((uint16_t)frame[5] << 7)) & DBUS_CHANNEL_MASK);

    /* 转为以中位为零的摇杆值 */
    ch0 = (int16_t)(ch0 - DBUS_CHANNEL_CENTER);
    ch1 = (int16_t)(ch1 - DBUS_CHANNEL_CENTER);
    ch2 = (int16_t)(ch2 - DBUS_CHANNEL_CENTER);
    ch3 = (int16_t)(ch3 - DBUS_CHANNEL_CENTER);

    sw1 = (uint8_t)((frame[5] >> 6) & DBUS_SWITCH_MASK);
    sw2 = (uint8_t)((frame[5] >> 4) & DBUS_SWITCH_MASK);

    /* 满杆约 ±660，超出范围视为数据错乱 */
    if ((DBUS_IsChannelInRange(ch0) == 0U) ||
        (DBUS_IsChannelInRange(ch1) == 0U) ||
        (DBUS_IsChannelInRange(ch2) == 0U) ||
        (DBUS_IsChannelInRange(ch3) == 0U))
    {
        return 0U;
    }

    out->ch0 = ch0;
    out->ch1 = ch1;
    out->ch2 = ch2;
    out->ch3 = ch3;
    out->sw1 = sw1;
    out->sw2 = sw2;

    return 1U;
}

HAL_StatusTypeDef DBUS_Init(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    if (huart->Instance != USART3)
    {
        return HAL_ERROR;
    }

    s_active_index = 0U;
    s_has_valid_frame = 0U;
    s_last_frame_tick = 0U;
    s_frame_count = 0U;
    s_error_count = 0U;

    memset(s_rc, 0, sizeof(s_rc));
    memset(s_dma_buffer, 0, sizeof(s_dma_buffer));

    /* 清除上电残留的 IDLE 标志，再开启空闲中断 */
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    if (HAL_UART_Receive_DMA(huart,
                             s_dma_buffer,
                             DBUS_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * HAL_UART_Receive_DMA() 会同时打开 PEIE/EIE；HAL_UART_IRQHandler()
     * 一旦检测到 PE/FE/NE/ORE，就会按阻塞性错误处理并中止 DMA，而本工程
     * 没有重启接收的路径。这里关闭这两个错误中断，错误标志仍会在清除
     * IDLE 标志时通过读 SR/DR 顺带清除。
     */
    __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
    __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

    return HAL_OK;
}

void DBUS_RxIdleHandler(UART_HandleTypeDef *huart)
{
    uint16_t write_pos;
    uint32_t remaining;
    uint8_t frame[DBUS_FRAME_LENGTH];
    rc_info_t decoded;
    uint8_t next_index;

    if (huart == NULL)
    {
        return;
    }

    /* 必须同时满足“标志置位”和“中断源使能”，才算一次有效 IDLE */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET)
    {
        return;
    }

    if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) == 0U)
    {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(huart);

    if (huart->hdmarx == NULL)
    {
        return;
    }

    /*
     * NDTR 表示还能写多少字节，总数减剩余即为当前写指针位置。
     * 先扩展到 32 位再判断，避免计数器异常时无符号相减回绕。
     */
    remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (remaining > DBUS_DMA_BUFFER_SIZE)
    {
        remaining = DBUS_DMA_BUFFER_SIZE;
    }

    write_pos = (uint16_t)(DBUS_DMA_BUFFER_SIZE - remaining);

    /* 取写指针之前的最后 18 字节；跨越缓冲区首尾时分两段拼接 */
    if (write_pos >= DBUS_FRAME_LENGTH)
    {
        memcpy(frame,
               &s_dma_buffer[write_pos - DBUS_FRAME_LENGTH],
               DBUS_FRAME_LENGTH);
    }
    else
    {
        uint16_t head_len = (uint16_t)(DBUS_FRAME_LENGTH - write_pos);

        memcpy(frame,
               &s_dma_buffer[DBUS_DMA_BUFFER_SIZE - head_len],
               head_len);
        memcpy(&frame[head_len], s_dma_buffer, write_pos);
    }

    if (DBUS_DecodeFrame(frame, &decoded) == 0U)
    {
        s_error_count++;
        return;
    }

    /* 写非活动缓冲区，整帧写完后才发布索引 */
    next_index = (uint8_t)((s_active_index == 0U) ? 1U : 0U);

    s_rc[next_index] = decoded;
    s_active_index = next_index;

    s_last_frame_tick = HAL_GetTick();
    s_has_valid_frame = 1U;
    s_frame_count++;
}

const volatile rc_info_t *DBUS_GetFeedback(void)
{
    uint8_t index = s_active_index;

    return &s_rc[index];
}

uint8_t DBUS_IsOnline(uint32_t now_ms)
{
    int32_t age;

    if (s_has_valid_frame == 0U)
    {
        return 0U;
    }

    /*
     * 使用带符号差值：主循环读取 now_ms 后，中断可能把 last_frame_tick
     * 更新到 now_ms 之后；无符号相减会回绕成巨大值，误判离线。
     */
    age = (int32_t)(now_ms - s_last_frame_tick);

    if (age < 0)
    {
        age = 0;
    }

    return (age <= (int32_t)DBUS_TIMEOUT_MS) ? 1U : 0U;
}

uint32_t DBUS_GetFrameCount(void)
{
    return s_frame_count;
}

uint32_t DBUS_GetErrorCount(void)
{
    return s_error_count;
}

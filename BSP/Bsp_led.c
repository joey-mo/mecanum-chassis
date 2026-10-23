/**
 * @file    Bsp_led.c
 * @brief   开发板 C 型板载 RGB LED 状态指示实现。
 */

#include "Bsp_led.h"

/* 心跳翻转周期：500ms 翻转一次，对应 1Hz 闪烁 */
#define LED_HEARTBEAT_TOGGLE_MS    (500U)

static LED_StateTypeDef s_state = LED_STATE_OFF;
static uint32_t s_last_toggle_ms = 0U;
static uint8_t s_heartbeat_on = 0U;

/** @brief 一次性写入蓝、绿、红三路 LED 电平。 */
static void LED_WriteAll(GPIO_PinState blue,
                         GPIO_PinState green,
                         GPIO_PinState red)
{
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, blue);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, green);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, red);
}

void LED_Init(void)
{
    s_state = LED_STATE_OFF;
    s_last_toggle_ms = HAL_GetTick();
    s_heartbeat_on = 0U;

    LED_WriteAll(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
}

void LED_Update(uint32_t now_ms, LED_StateTypeDef state)
{
    GPIO_PinState heartbeat_level;

    /* 状态切换时重置心跳相位，避免切换瞬间出现残影 */
    if (state != s_state)
    {
        s_state = state;
        s_last_toggle_ms = now_ms;
        s_heartbeat_on = 0U;
    }

    if ((now_ms - s_last_toggle_ms) >= LED_HEARTBEAT_TOGGLE_MS)
    {
        s_last_toggle_ms = now_ms;
        s_heartbeat_on = (uint8_t)((s_heartbeat_on == 0U) ? 1U : 0U);
    }

    heartbeat_level = (s_heartbeat_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (s_state)
    {
        case LED_STATE_INIT:
            /* 蓝灯心跳：固件已运行，等待外设就绪 */
            LED_WriteAll(heartbeat_level, GPIO_PIN_RESET, GPIO_PIN_RESET);
            break;

        case LED_STATE_RUN:
            /* 绿灯常亮 + 蓝灯心跳 */
            LED_WriteAll(heartbeat_level, GPIO_PIN_SET, GPIO_PIN_RESET);
            break;

        case LED_STATE_FAULT:
            /* 红灯心跳闪烁 */
            LED_WriteAll(GPIO_PIN_RESET, GPIO_PIN_RESET, heartbeat_level);
            break;

        case LED_STATE_OFF:
        default:
            LED_WriteAll(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
            break;
    }
}

/**
 * @file    Bsp_led.h
 * @brief   开发板 C 型板载 RGB LED 状态指示。
 */

#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>

#include "main.h"

/* 开发板 C 型 RGB LED 引脚独立定义，避免 CubeMX 重新生成 main.h 时丢失 */
#define LED_B_GPIO_Port    GPIOH
#define LED_B_Pin          GPIO_PIN_10
#define LED_G_GPIO_Port    GPIOH
#define LED_G_Pin          GPIO_PIN_11
#define LED_R_GPIO_Port    GPIOH
#define LED_R_Pin          GPIO_PIN_12

/**
 * @brief LED 显示状态。
 *
 * 引脚对应关系：PH10=蓝，PH11=绿，PH12=红，高电平点亮。
 * 显示约定：
 *   INIT  ：蓝灯 1Hz 心跳，表示固件已启动、正在等待外设就绪。
 *   RUN   ：绿灯常亮 + 蓝灯心跳。
 *   FAULT ：红灯 1Hz 闪烁。
 *   OFF   ：全灭。
 */
typedef enum
{
    LED_STATE_OFF = 0,
    LED_STATE_INIT,
    LED_STATE_RUN,
    LED_STATE_FAULT
} LED_StateTypeDef;

/**
 * @brief 初始化 RGB LED，全部熄灭。
 */
void LED_Init(void);

/**
 * @brief 按当前状态刷新 LED。
 *
 * 由控制层按固定周期调用；内部维护 1Hz 心跳相位。
 *
 * @param now_ms 当前时间戳，单位 ms。
 * @param state  目标显示状态。
 */
void LED_Update(uint32_t now_ms, LED_StateTypeDef state);

#endif /* BSP_LED_H */

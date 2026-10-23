/**
 * @file    RcMonitor.h
 * @brief   遥控、CAN 诊断和电机调参数据输出（USART6 -> VOFA+ FireWater）。
 *
 * 每 20ms 输出一行，共 26 个整数，字段顺序如下：
 *
 *   ch0, ch1, ch2, ch3, sw1, sw2,
 *   can_rx, can_last_id, can_esr, can_fifo, can_valid_mask,
 *   can_online, dbus_online, state,
 *   tgt_fl, act_fl, cur_fl,
 *   tgt_fr, act_fr, cur_fr,
 *   tgt_rr, act_rr, cur_rr,
 *   tgt_rl, act_rl, cur_rl
 *
 * 字段说明：
 *   tgt_*          目标转子转速，rpm
 *   act_*          实际转子转速，rpm
 *   cur_*          速度环输出的电流指令 raw（±8192 对应 ±10A）
 *   can_rx         累计读出的 CAN 帧数；0 表示一帧都没收到
 *   can_last_id    最后一帧 StdId；0 表示未收到，0x201~0x204 即 513~516
 *   can_esr        CAN1->ESR；0 表示无错误
 *   can_fifo       CAN1->RF0R 的 FMP0，即 FIFO0 待处理帧数（0~3）
 *   can_valid_mask 电机反馈有效位：bit0 FL、bit1 FR、bit2 RR、bit3 RL
 *   can_online     四台电机都在 20ms 内收到过反馈
 *   dbus_online    DBus 在 50ms 内收到过有效帧
 *   state          0=INIT，1=RUN，2=FAULT
 *
 * 电机数据按电调 ID1~ID4 排列；当前 CHASSIS_FR_RR_SWAP=0，ID 与轮位一致。
 * 只使用整数格式，避免 newlib-nano 缺少浮点 printf 支持。
 * USART6 固定 115200-8-N-1（PG14=TX，PG9=RX）。
 * 发送使用 HAL_UART_Transmit_IT()，不阻塞 1kHz 控制循环。
 */

#ifndef RC_MONITOR_H
#define RC_MONITOR_H

/** @brief 输出周期，单位 ms（50Hz）。 */
#define RC_MONITOR_PERIOD_MS        (20U)

/** @brief 发送缓冲区大小，需容纳一帧文本及结尾的 '\0'。 */
#define RC_MONITOR_TX_BUFFER_SIZE   (256U)

/**
 * @brief 由主循环周期调用。
 *
 * 内部按 RC_MONITOR_PERIOD_MS 限频，并检查上一帧是否已经发送完成。
 */
void RcMonitor_Task(void);

#endif /* RC_MONITOR_H */

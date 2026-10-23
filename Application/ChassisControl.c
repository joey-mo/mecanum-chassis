/**
 * @file    ChassisControl.c
 * @brief   底盘状态机、遥控映射与四轮速度环控制。
 *
 * 状态迁移（方案 B：不使用拨杆）：
 *   INIT   ：DBus 与四轮反馈连续正常，且摇杆回中，稳定后进入 RUN。
 *   RUN    ：遥控 -> 目标速度 -> 逆解 -> 四轮速度环 -> 0x200。
 *   FAULT  ：任一链路离线即进入；条件恢复且摇杆回中后自动回到 RUN。
 *
 * 无论处于哪个状态，每个 1ms 周期都会发送一帧 0x200。非 RUN 状态
 * 发送零电流，保证 C620 持续收到明确指令，而不会保持上一帧输出。
 */

#include "ChassisControl.h"

#include <stddef.h>

#include "Bsp_can.h"
#include "Bsp_dbus.h"
#include "Bsp_led.h"
#include "ChassisConfig.h"
#include "ChassisKinematics.h"
#include "Motor.h"
#include "PID.h"

/** @brief 条件持续计时器：条件首次成立时记录起点，条件消失时复位。 */
typedef struct
{
    uint8_t active;    /**< 条件当前是否连续成立 */
    uint32_t since_ms; /**< 条件开始成立的时刻 */
} ChassisTimerTypeDef;

static ChassisTimerTypeDef s_startup_timer;
static ChassisTimerTypeDef s_center_timer;
static ChassisTimerTypeDef s_recovery_timer;

static ChassisStateTypeDef s_state;

/* 斜坡处理后的车体速度指令，以及逆解得到的四轮目标转速 */
static ChassisVelocityTypeDef s_velocity_command;
static ChassisWheelRpmTypeDef s_wheel_rpm;

/**
 * @brief 更新条件持续计时器。
 *
 * 条件首次成立时记录 now_ms；条件消失时清除 active。
 *
 * @return 条件当前是否成立。
 */
static uint8_t Chassis_TimerUpdate(ChassisTimerTypeDef *timer,
                                   uint32_t now_ms,
                                   uint8_t condition)
{
    if (condition == 0U)
    {
        timer->active = 0U;
        return 0U;
    }

    if (timer->active == 0U)
    {
        timer->active = 1U;
        timer->since_ms = now_ms;
    }

    return 1U;
}

/**
 * @brief 判断计时器是否已连续成立至少 duration_ms。
 */
static uint8_t Chassis_TimerElapsed(const ChassisTimerTypeDef *timer,
                                    uint32_t now_ms,
                                    uint32_t duration_ms)
{
    if (timer->active == 0U)
    {
        return 0U;
    }

    return (uint8_t)((now_ms - timer->since_ms) >= duration_ms);
}

/** @brief 复位全部状态机计时器。 */
static void Chassis_ResetStateTimers(void)
{
    s_startup_timer.active = 0U;
    s_startup_timer.since_ms = 0U;

    s_center_timer.active = 0U;
    s_center_timer.since_ms = 0U;

    s_recovery_timer.active = 0U;
    s_recovery_timer.since_ms = 0U;
}

/** @brief 取 int16_t 的绝对值。 */
static int16_t RC_Abs(int16_t value)
{
    return (value >= 0) ? value : (int16_t)(-value);
}

/**
 * @brief 按配置的通道号读取原始值。
 *
 * @param rc      遥控器状态。
 * @param channel 通道号：0=ch0，1=ch1，2=ch2，3=ch3。
 * @return 对应通道的原始值；通道号无效时返回 0。
 */
static int16_t RC_ReadChannel(const volatile rc_info_t *rc, uint8_t channel)
{
    switch (channel)
    {
        case 0U:
            return rc->ch0;

        case 1U:
            return rc->ch1;

        case 2U:
            return rc->ch2;

        case 3U:
            return rc->ch3;

        default:
            return 0;
    }
}

/** @brief 判断参与运动的三个摇杆通道是否都回到中位。 */
static uint8_t RC_SticksCentered(const volatile rc_info_t *rc)
{
    const int16_t threshold = (int16_t)RC_ARM_CENTER_THRESHOLD_RAW;

    if (rc == NULL)
    {
        return 0U;
    }

    return (uint8_t)(
        (RC_Abs(RC_ReadChannel(rc, RC_VX_CHANNEL)) <= threshold) &&
        (RC_Abs(RC_ReadChannel(rc, RC_VY_CHANNEL)) <= threshold) &&
        (RC_Abs(RC_ReadChannel(rc, RC_WZ_CHANNEL)) <= threshold));
}

/** @brief 把浮点值限制在 [minimum, maximum] 内。 */
static float Chassis_ClampFloat(float value,
                                float minimum,
                                float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

/**
 * @brief 将遥控通道原始值映射到 [-1, 1]。
 *
 * 处理顺序：按满杆归一化 -> 限幅 -> 死区重映射 -> 指数曲线。
 * RC_EXPO = 1.0 时线性；RC_EXPO > 1.0 时使用平方曲线。
 *
 * @param raw       通道原始值。
 * @param direction 通道方向，取 +1.0f 或 -1.0f。
 * @return 归一化并应用曲线后的值。
 */
static float RC_ApplyCurve(int16_t raw, float direction)
{
    const float deadzone = RC_DEADZONE_RAW / RC_FULL_SCALE_RAW;
    float normalized;
    float magnitude;

    normalized = ((float)raw * direction) / RC_FULL_SCALE_RAW;
    normalized = Chassis_ClampFloat(normalized, -1.0f, 1.0f);

    magnitude = (normalized >= 0.0f) ? normalized : -normalized;

    if (magnitude <= deadzone)
    {
        return 0.0f;
    }

    /* 死区外重新映射到 0~1，保证死区边界处输出连续 */
    magnitude = (magnitude - deadzone) / (1.0f - deadzone);

    if (RC_EXPO > 1.0f)
    {
        magnitude = magnitude * magnitude;
    }

    return (normalized >= 0.0f) ? magnitude : -magnitude;
}

/**
 * @brief 对单个速度分量做一阶斜坡限制。
 *
 * @param current   当前值。
 * @param target    目标值。
 * @param max_delta 单个周期允许的最大变化量。
 * @return 斜坡处理后的值。
 */
static float Chassis_Slew(float current, float target, float max_delta)
{
    float delta = target - current;

    if (delta > max_delta)
    {
        delta = max_delta;
    }

    if (delta < -max_delta)
    {
        delta = -max_delta;
    }

    return current + delta;
}

/**
 * @brief 获取指定轮的电机安装方向。
 *
 * 轮子向前为正；若某轮实际方向相反，在 ChassisConfig.h 中配置 -1.0f。
 */
static float Chassis_GetWheelDirection(uint8_t wheel)
{
    switch (wheel)
    {
        case CHASSIS_WHEEL_FL:
            return CHASSIS_FL_DIRECTION;

        case CHASSIS_WHEEL_FR:
            return CHASSIS_FR_DIRECTION;

        case CHASSIS_WHEEL_RR:
            return CHASSIS_RR_DIRECTION;

        case CHASSIS_WHEEL_RL:
            return CHASSIS_RL_DIRECTION;

        default:
            return 1.0f;
    }
}

/** @brief 清空速度指令、四轮目标电流和 PID 历史。 */
static void Chassis_ResetControl(void)
{
    uint8_t i;

    s_velocity_command.vx_m_s = 0.0f;
    s_velocity_command.vy_m_s = 0.0f;
    s_velocity_command.wz_rad_s = 0.0f;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        DJI3508_MotorTypeDef *motor = &DJI3508_ChassisMotorGroup.motors[i];

        motor->control.target_speed_rpm = 0.0f;
        motor->control.current_command_raw = 0;
        s_wheel_rpm.wheel_rpm[i] = 0.0f;

        PID_Position_Reset(&motor->control.speed_pid);
    }
}

/** @brief 发送一帧零电流，并同步清除调试用的目标值和输出值。 */
static void Chassis_SendZeroCurrent(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        DJI3508_MotorTypeDef *motor = &DJI3508_ChassisMotorGroup.motors[i];

        motor->control.target_speed_rpm = 0.0f;
        motor->control.current_command_raw = 0;
        s_wheel_rpm.wheel_rpm[i] = 0.0f;
    }

    (void)BSP_CAN_SendMotorCurrents(&DJI3508_ChassisMotorGroup.tx_frame,
                                    0, 0, 0, 0);
}

/** @brief 切换状态，并复位计时器与控制量。 */
static void Chassis_EnterState(ChassisStateTypeDef state)
{
    s_state = state;
    Chassis_ResetStateTimers();

    if (state == CHASSIS_STATE_RUN)
    {
        /* 从零开始斜坡，避免刚使能时目标速度突变 */
        s_velocity_command.vx_m_s = 0.0f;
        s_velocity_command.vy_m_s = 0.0f;
        s_velocity_command.wz_rad_s = 0.0f;
    }
    else
    {
        Chassis_ResetControl();
    }
}

/** @brief 根据状态更新板载 RGB LED。 */
static void Chassis_UpdateLed(uint32_t now_ms, uint8_t can_online)
{
    switch (s_state)
    {
        case CHASSIS_STATE_RUN:
            LED_Update(now_ms, LED_STATE_RUN);
            break;

        case CHASSIS_STATE_FAULT:
            LED_Update(now_ms, LED_STATE_FAULT);
            break;

        case CHASSIS_STATE_INIT:
        default:
            /*
             * INIT 同时覆盖“等 CAN”和“等 DBus”两种情况。
             * CAN 未通时仍用红灯，保留最直观的故障提示。
             */
            if (can_online == 0U)
            {
                LED_Update(now_ms, LED_STATE_FAULT);
            }
            else
            {
                LED_Update(now_ms, LED_STATE_INIT);
            }
            break;
    }
}

/*
 * 物理轮 -> 电机数组下标的映射。
 * 默认 [FL, FR, RR, RL] 与 ID1~ID4 一一对应；FR/RR 互换时交换数组第 2、3 项。
 */
static const uint8_t s_wheel_to_motor[CHASSIS_WHEEL_COUNT] =
{
    CHASSIS_WHEEL_FL,
#if CHASSIS_FR_RR_SWAP
    CHASSIS_WHEEL_RR,
    CHASSIS_WHEEL_FR,
#else
    CHASSIS_WHEEL_FR,
    CHASSIS_WHEEL_RR,
#endif
    CHASSIS_WHEEL_RL
};

/** @brief RUN 状态：遥控映射 -> 斜坡 -> 逆解 -> 四轮 PID -> 0x200。 */
static void Chassis_CalculateAndSend(const volatile rc_info_t *rc)
{
    float vx_target;
    float vy_target;
    float wz_target;
    int16_t current_id1;
    int16_t current_id2;
    int16_t current_id3;
    int16_t current_id4;
    uint8_t i;

    /* 1. 遥控通道 -> 车体目标速度 */
    vx_target = RC_ApplyCurve(RC_ReadChannel(rc, RC_VX_CHANNEL),
                              RC_VX_SIGN) *
                CHASSIS_MAX_VX_M_S *
                CHASSIS_RC_SPEED_SCALE;

    vy_target = RC_ApplyCurve(RC_ReadChannel(rc, RC_VY_CHANNEL),
                              RC_VY_SIGN) *
                CHASSIS_MAX_VY_M_S *
                CHASSIS_RC_SPEED_SCALE;

    wz_target = RC_ApplyCurve(RC_ReadChannel(rc, RC_WZ_CHANNEL),
                              RC_WZ_SIGN) *
                CHASSIS_MAX_WZ_RAD_S *
                CHASSIS_RC_SPEED_SCALE;

    /* 2. 对车体速度做斜坡限制，降低电流冲击和打滑 */
    s_velocity_command.vx_m_s =
        Chassis_Slew(s_velocity_command.vx_m_s,
                     vx_target,
                     CHASSIS_MAX_ACCEL_M_S2 * CHASSIS_CONTROL_PERIOD_S);

    s_velocity_command.vy_m_s =
        Chassis_Slew(s_velocity_command.vy_m_s,
                     vy_target,
                     CHASSIS_MAX_ACCEL_M_S2 * CHASSIS_CONTROL_PERIOD_S);

    s_velocity_command.wz_rad_s =
        Chassis_Slew(s_velocity_command.wz_rad_s,
                     wz_target,
                     CHASSIS_MAX_ANG_ACCEL_RAD_S2 *
                     CHASSIS_CONTROL_PERIOD_S);

    /* 3. 车体速度 -> 四轮输出轴目标转速 */
    ChassisKinematics_Solve(&s_velocity_command,
                            CHASSIS_MAX_WHEEL_RPM,
                            &s_wheel_rpm);

    /* 4. 逐轮执行速度环，输出电流指令 */
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        uint8_t motor_index = s_wheel_to_motor[i];
        DJI3508_MotorTypeDef *motor =
            &DJI3508_ChassisMotorGroup.motors[motor_index];
        const volatile DJI3508_FeedbackTypeDef *feedback;
        float target_rpm;
        float current;

        feedback = DJI3508_GetFeedback(motor);

        /* 输出轴 rpm -> 转子 rpm，并应用该轮的安装方向 */
        target_rpm = s_wheel_rpm.wheel_rpm[i] *
                     CHASSIS_GEAR_RATIO *
                     Chassis_GetWheelDirection(i);

        /* 双保险：即使配置写错，也不让目标超过电机额定转速 */
        target_rpm = Chassis_ClampFloat(target_rpm,
                                        -CHASSIS_MAX_MOTOR_RPM,
                                        CHASSIS_MAX_MOTOR_RPM);

        motor->control.target_speed_rpm = target_rpm;

        current = PID_Position_Calculate(&motor->control.speed_pid,
                                         target_rpm,
                                         (float)feedback->rotor_speed_rpm);

        /* PID 自身已限幅，这里再按 C620 电流上限做一层硬限幅 */
        current = Chassis_ClampFloat(current,
                                     -(float)CHASSIS_CURRENT_LIMIT_RAW,
                                     (float)CHASSIS_CURRENT_LIMIT_RAW);

        motor->control.current_command_raw = (int16_t)current;
    }

    /*
     * 0x200 的四个槽位固定对应电调 ID1~ID4。
     * 这里按电机数组顺序取电流，不按物理轮顺序重排。
     */
    current_id1 =
        DJI3508_ChassisMotorGroup.motors[0].control.current_command_raw;
    current_id2 =
        DJI3508_ChassisMotorGroup.motors[1].control.current_command_raw;
    current_id3 =
        DJI3508_ChassisMotorGroup.motors[2].control.current_command_raw;
    current_id4 =
        DJI3508_ChassisMotorGroup.motors[3].control.current_command_raw;

    (void)BSP_CAN_SendMotorCurrents(&DJI3508_ChassisMotorGroup.tx_frame,
                                    current_id1,
                                    current_id2,
                                    current_id3,
                                    current_id4);
}

void ChassisControl_Init(void)
{
    uint8_t i;

    s_state = CHASSIS_STATE_INIT;
    Chassis_ResetStateTimers();

    s_velocity_command.vx_m_s = 0.0f;
    s_velocity_command.vy_m_s = 0.0f;
    s_velocity_command.wz_rad_s = 0.0f;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        DJI3508_MotorTypeDef *motor = &DJI3508_ChassisMotorGroup.motors[i];

        motor->control.target_speed_rpm = 0.0f;
        motor->control.current_command_raw = 0;
        s_wheel_rpm.wheel_rpm[i] = 0.0f;

        PID_Position_Init(&motor->control.speed_pid,
                          CHASSIS_PID_KP,
                          CHASSIS_PID_KI,
                          CHASSIS_PID_KD,
                          CHASSIS_PID_OUTPUT_LIMIT_RAW,
                          -CHASSIS_PID_OUTPUT_LIMIT_RAW,
                          CHASSIS_PID_INTEGRAL_LIMIT_RAW,
                          -CHASSIS_PID_INTEGRAL_LIMIT_RAW);
    }

    LED_Update(HAL_GetTick(), LED_STATE_INIT);
}

void ChassisControl_Step(void)
{
    uint32_t now_ms = HAL_GetTick();
    const volatile rc_info_t *rc = DBUS_GetFeedback();

    uint8_t dbus_online = DBUS_IsOnline(now_ms);
    uint8_t can_online = DJI3508_GroupIsOnline(&DJI3508_ChassisMotorGroup,
                                               now_ms);
    uint8_t rc_centered = RC_SticksCentered(rc);
    uint8_t output_sent = 0U;

    switch (s_state)
    {
        case CHASSIS_STATE_INIT:
        {
            uint8_t startup_ok;
            uint8_t centered_ok;

            startup_ok = Chassis_TimerUpdate(
                &s_startup_timer,
                now_ms,
                (uint8_t)((dbus_online != 0U) && (can_online != 0U)));

            centered_ok = Chassis_TimerUpdate(
                &s_center_timer,
                now_ms,
                (uint8_t)((startup_ok != 0U) && (rc_centered != 0U)));

            if ((startup_ok != 0U) &&
                (centered_ok != 0U) &&
                (Chassis_TimerElapsed(&s_startup_timer,
                                      now_ms,
                                      CHASSIS_INIT_STABLE_MS) != 0U) &&
                (Chassis_TimerElapsed(&s_center_timer,
                                      now_ms,
                                      CHASSIS_ARM_CENTER_STABLE_MS) != 0U))
            {
                Chassis_EnterState(CHASSIS_STATE_RUN);
            }

            break;
        }

        case CHASSIS_STATE_RUN:
            if ((dbus_online == 0U) ||
                (can_online == 0U) ||
                (rc == NULL))
            {
                Chassis_EnterState(CHASSIS_STATE_FAULT);
            }
            else
            {
                Chassis_CalculateAndSend(rc);
                output_sent = 1U;
            }

            break;

        case CHASSIS_STATE_FAULT:
        default:
        {
            uint8_t recover_ok;

            recover_ok = Chassis_TimerUpdate(
                &s_recovery_timer,
                now_ms,
                (uint8_t)((dbus_online != 0U) &&
                          (can_online != 0U) &&
                          (rc != NULL) &&
                          (rc_centered != 0U)));

            if ((recover_ok != 0U) &&
                (Chassis_TimerElapsed(&s_recovery_timer,
                                      now_ms,
                                      CHASSIS_FAULT_RECOVER_MS) != 0U))
            {
                Chassis_EnterState(CHASSIS_STATE_RUN);
            }

            break;
        }
    }

    if (output_sent == 0U)
    {
        Chassis_SendZeroCurrent();
    }

    Chassis_UpdateLed(now_ms, can_online);
}

ChassisStateTypeDef ChassisControl_GetState(void)
{
    return s_state;
}

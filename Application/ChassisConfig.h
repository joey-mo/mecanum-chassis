#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*
 * 麦克纳姆底盘全部可标定参数集中在这里。
 *
 * 约定：
 *   +x 车头向前，+y 车体向左，+wz 俯视逆时针。
 *   轮子正方向 = 该轮驱动车体向 +x。
 *   轮序        = [FL, FR, RR, RL] = [ID1, ID2, ID3, ID4]。
 */

/* ===================== 控制周期 ===================== */
#define CHASSIS_CONTROL_PERIOD_MS              (1U)
#define CHASSIS_CONTROL_PERIOD_S               (0.001f)

/* ===================== 机械参数 ===================== */
#define CHASSIS_PI                             (3.14159265358979323846f)

/* 轮子半径，单位 m（名义值，落地后可用实测速度校正） */
#define CHASSIS_WHEEL_RADIUS_M                 (0.070f)

/* 前后半轴距 = 轴距 / 2，单位 m */
#define CHASSIS_HALF_LENGTH_M                  (0.170f)

/* 左右半轮距 = 轮距 / 2，单位 m */
#define CHASSIS_HALF_WIDTH_M                   (0.235f)

/* M3508 P19 减速比 */
#define CHASSIS_GEAR_RATIO                     (3591.0f / 187.0f)

/* ===================== 电机额定值与限幅 ===================== */
/* 额定输出轴转速，rpm */
#define CHASSIS_MAX_WHEEL_RPM                  (469.0f)

/* 额定转子转速，rpm（约 9006） */
#define CHASSIS_MAX_MOTOR_RPM                  \
    (CHASSIS_MAX_WHEEL_RPM * CHASSIS_GEAR_RATIO)

/* C620 转矩电流限幅：10A / 20A * 16384 = 8192 */
#define CHASSIS_CURRENT_LIMIT_RAW              (8192)
#define CHASSIS_PID_OUTPUT_LIMIT_RAW           (8192.0f)

/* 积分项限幅，防止长期误差导致积分饱和 */
#define CHASSIS_PID_INTEGRAL_LIMIT_RAW         (2048.0f)

/* ===================== 车体速度目标 ===================== */
/* 由额定轮速换算出的最大平移速度，约 3.438 m/s */
#define CHASSIS_MAX_WHEEL_LINEAR_M_S           \
    (CHASSIS_MAX_WHEEL_RPM * 2.0f * CHASSIS_PI * \
     CHASSIS_WHEEL_RADIUS_M / 60.0f)

#define CHASSIS_MAX_VX_M_S                     CHASSIS_MAX_WHEEL_LINEAR_M_S
#define CHASSIS_MAX_VY_M_S                     CHASSIS_MAX_WHEEL_LINEAR_M_S

/* 自转上限：理论轮速上限约 8.49 rad/s，实际取 3.0 rad/s（约 172°/s） */
#define CHASSIS_MAX_WZ_RAD_S                   (3.0f)

/* 速度/角速度斜坡，避免电流冲击与打滑 */
#define CHASSIS_MAX_ACCEL_M_S2                 (3.0f)
#define CHASSIS_MAX_ANG_ACCEL_RAD_S2           (8.0f)

/* 首次落地速度比例；跑顺后改为 1.0 */
#define CHASSIS_RC_SPEED_SCALE                 (0.3f)

/* ===================== 遥控输入 ===================== */
/* DBus 摇杆满杆约 ±660 */
#define RC_FULL_SCALE_RAW                      (660.0f)

/* 摇杆死区，单位 raw */
#define RC_DEADZONE_RAW                        (30.0f)

/* 自动使能要求摇杆回中的阈值，单位 raw */
#define RC_ARM_CENTER_THRESHOLD_RAW            (50.0f)

#define RC_VX_CHANNEL                          (3U)
#define RC_VY_CHANNEL                          (2U)
#define RC_WZ_CHANNEL                          (0U)

/* 通道正方向：恢复为初始的 +1 */
#define RC_VX_SIGN                             (+1.0f)
#define RC_VY_SIGN                             (+1.0f)
#define RC_WZ_SIGN                             (+1.0f)

/* 摇杆曲线指数：1.0 = 线性，>1 低速更细腻 */
#define RC_EXPO                                (1.0f)

/* ===================== 安全与超时 ===================== */
#define DBUS_TIMEOUT_MS                        (50U)
#define MOTOR_FEEDBACK_TIMEOUT_MS              (20U)

/* 上电自检：DBus 与四轮反馈连续正常多久后进入可运行 */
#define CHASSIS_INIT_STABLE_MS                 (500U)

/* 自动使能：摇杆连续回中多久后进入 RUN */
#define CHASSIS_ARM_CENTER_STABLE_MS           (500U)

/* 故障恢复：条件恢复且摇杆回中多久后重新使能 */
#define CHASSIS_FAULT_RECOVER_MS               (500U)

/* ===================== 电机安装方向 ===================== */

#define CHASSIS_FL_DIRECTION                   (+1.0f)
#define CHASSIS_FR_DIRECTION                   (-1.0f)
#define CHASSIS_RR_DIRECTION                   (-1.0f)
#define CHASSIS_RL_DIRECTION                   (+1.0f)

/* ===================== 速度环 PID 初值 ===================== */
#define CHASSIS_PID_KP                         (3.3f)
#define CHASSIS_PID_KI                         (0.0f)
#define CHASSIS_PID_KD                         (0.0f)

#endif /* CHASSIS_CONFIG_H */

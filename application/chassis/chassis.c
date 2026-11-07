/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        平面坐标：x向前、y向右，wz逆时针为正，速度单位mm/s、degree/s。
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "daemon.h"
#include "referee_task.h"

#include "general_def.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include <math.h>

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define LINEAR_TO_MOTOR (360.0f * REDUCTION_RATIO_WHEEL / PERIMETER_WHEEL)
#define MOTOR_TO_LINEAR (1.0f / LINEAR_TO_MOTOR)

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static DaemonInstance *chassis_cmd_daemon;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
#ifdef CHASSIS_DRIVE_SWERVE
static DJIMotorInstance *steer_motor_lf, *steer_motor_rf, *steer_motor_lb, *steer_motor_rb;
static float steer_target_lf, steer_target_rf, steer_target_lb, steer_target_rb;
#endif
// 在线判断函数声明
static uint8_t ChassisFeedbackValid(void);
static uint8_t ChassisCommandValid(void);
static void SetChassisEnabled(uint8_t enabled);

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

#ifdef CHASSIS_DRIVE_SWERVE
    Motor_Init_Config_s steer_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .motor_type = GM6020,
    };
    steer_motor_config.can_init_config.tx_id = 1;
    steer_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_motor_lf = DJIMotorInit(&steer_motor_config);

    steer_motor_config.can_init_config.tx_id = 2;
    steer_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_motor_rf = DJIMotorInit(&steer_motor_config);

    steer_motor_config.can_init_config.tx_id = 4;
    steer_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_motor_lb = DJIMotorInit(&steer_motor_config);

    steer_motor_config.can_init_config.tx_id = 3;
    steer_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_motor_rb = DJIMotorInit(&steer_motor_config);
#endif
    SetChassisEnabled(0);

    referee_data = UITaskInit(&huart1,&ui_data); // 裁判系统初始化,会同时初始化UI

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan3,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .daemon_count = DAEMON_COUNT_FROM_MS(APP_COMMAND_TIMEOUT_MS),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
    Daemon_Init_Config_s cmd_daemon_config = {
        .reload_count = DAEMON_COUNT_FROM_MS(APP_COMMAND_TIMEOUT_MS),
        .require_first_reload = 1,
    };
    chassis_cmd_daemon = DaemonRegister(&cmd_daemon_config);
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
#ifdef CHASSIS_DRIVE_MECANUM
static void MecanumCalculate()
{
    // vt_* 是轮面线速度；正负号由当前底盘坐标约定及麦轮滚子方向决定。
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
}
#endif

#ifdef CHASSIS_DRIVE_OMNI_X
#define COS45 0.70710678f
#define OMNI_L (LF_CENTER)
static void OmniCalculate()
{
    // X 型布置下，各轮沿 45° 方向滚动，平移和旋转分量投影到对应轮轴。
    vt_rf =  chassis_vx * COS45 + chassis_vy * COS45 + chassis_cmd_recv.wz * OMNI_L;
    vt_lf = -chassis_vx * COS45 + chassis_vy * COS45 + chassis_cmd_recv.wz * OMNI_L;
    vt_lb = -chassis_vx * COS45 - chassis_vy * COS45 + chassis_cmd_recv.wz * OMNI_L;
    vt_rb =  chassis_vx * COS45 - chassis_vy * COS45 + chassis_cmd_recv.wz * OMNI_L;
}
#endif

#ifdef CHASSIS_DRIVE_SWERVE
/**
 * @brief 根据车体速度求四个舵轮的舵角和轮面线速度参考。
 *        车体坐标为 x 向前、y 向右，舵角以各轮朝前的标定零位为 0°。
 */
static void SwerveCalculate(void)
{
    // 将角速度转换为 rad/s，再按各轮相对底盘旋转中心的位置求轮心速度向量。
    // 四个轮组分别处理；电机反转标志统一角度正方向，舵角参考也映射到 DJI 驱动使用的电机坐标。
    float wz = chassis_cmd_recv.wz * DEGREE_2_RAD;
    float vx_lf = chassis_vx + wz * (-HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_Y);
    float vy_lf = chassis_vy - wz * (HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_X);
    float speed_lf = sqrtf(vx_lf * vx_lf + vy_lf * vy_lf);
    // 轮心速度接近零时只清轮速并保留上一舵向目标，避免静止附近角度抖动。
    if (speed_lf > 0.001f)
    {
        float direction_lf = steer_motor_lf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;
        // 先把编码器角度映射到统一的逻辑舵角，再取最短角差；误差超过 90° 时翻转轮速，避免舵轮大幅回转。
        float error_lf = theta_format(atan2f(vy_lf, vx_lf) / DEGREE_2_RAD -
            direction_lf * (steer_motor_lf->measure.total_angle -
            SWERVE_STEER_ZERO_ECD_LF * ECD_ANGLE_COEF_DJI));
        if (error_lf > 90.0f) { error_lf -= 180.0f; speed_lf = -speed_lf; }
        else if (error_lf < -90.0f) { error_lf += 180.0f; speed_lf = -speed_lf; }
        steer_target_lf = direction_lf * steer_motor_lf->measure.total_angle + error_lf;
        // 舵角尚未跟上时按 cos(error)^3 降低驱动力，减少轮子横向拖动。
        float compensation_lf = arm_cos_f32(error_lf * DEGREE_2_RAD);
        vt_lf = speed_lf * compensation_lf * compensation_lf * compensation_lf;
    }
    else vt_lf = 0;

    float vx_rf = chassis_vx + wz * (HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_Y);
    float vy_rf = chassis_vy - wz * (HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_X);
    float speed_rf = sqrtf(vx_rf * vx_rf + vy_rf * vy_rf);
    if (speed_rf > 0.001f)
    {
        float direction_rf = steer_motor_rf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;
        float error_rf = theta_format(atan2f(vy_rf, vx_rf) / DEGREE_2_RAD -
            direction_rf * (steer_motor_rf->measure.total_angle -
            SWERVE_STEER_ZERO_ECD_RF * ECD_ANGLE_COEF_DJI));
        if (error_rf > 90.0f) { error_rf -= 180.0f; speed_rf = -speed_rf; }
        else if (error_rf < -90.0f) { error_rf += 180.0f; speed_rf = -speed_rf; }
        steer_target_rf = direction_rf * steer_motor_rf->measure.total_angle + error_rf;
        float compensation_rf = arm_cos_f32(error_rf * DEGREE_2_RAD);
        vt_rf = speed_rf * compensation_rf * compensation_rf * compensation_rf;
    }
    else vt_rf = 0;

    float vx_lb = chassis_vx + wz * (-HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_Y);
    float vy_lb = chassis_vy - wz * (-HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_X);
    float speed_lb = sqrtf(vx_lb * vx_lb + vy_lb * vy_lb);
    if (speed_lb > 0.001f)
    {
        float direction_lb = steer_motor_lb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;
        float error_lb = theta_format(atan2f(vy_lb, vx_lb) / DEGREE_2_RAD -
            direction_lb * (steer_motor_lb->measure.total_angle -
            SWERVE_STEER_ZERO_ECD_LB * ECD_ANGLE_COEF_DJI));
        if (error_lb > 90.0f) { error_lb -= 180.0f; speed_lb = -speed_lb; }
        else if (error_lb < -90.0f) { error_lb += 180.0f; speed_lb = -speed_lb; }
        steer_target_lb = direction_lb * steer_motor_lb->measure.total_angle + error_lb;
        float compensation_lb = arm_cos_f32(error_lb * DEGREE_2_RAD);
        vt_lb = speed_lb * compensation_lb * compensation_lb * compensation_lb;
    }
    else vt_lb = 0;

    float vx_rb = chassis_vx + wz * (HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_Y);
    float vy_rb = chassis_vy - wz * (-HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_X);
    float speed_rb = sqrtf(vx_rb * vx_rb + vy_rb * vy_rb);
    if (speed_rb > 0.001f)
    {
        float direction_rb = steer_motor_rb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;
        float error_rb = theta_format(atan2f(vy_rb, vx_rb) / DEGREE_2_RAD -
            direction_rb * (steer_motor_rb->measure.total_angle -
            SWERVE_STEER_ZERO_ECD_RB * ECD_ANGLE_COEF_DJI));
        if (error_rb > 90.0f) { error_rb -= 180.0f; speed_rb = -speed_rb; }
        else if (error_rb < -90.0f) { error_rb += 180.0f; speed_rb = -speed_rb; }
        steer_target_rb = direction_rb * steer_motor_rb->measure.total_angle + error_rb;
        float compensation_rb = arm_cos_f32(error_rb * DEGREE_2_RAD);
        vt_rb = speed_rb * compensation_rb * compensation_rb * compensation_rb;
    }
    else vt_rb = 0;
}
#endif

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // vt_* 已经是轮面 mm/s；先按实际轮面速度限幅，再换算为 DJI 电机的 degree/s 参考值。
    float wheel_speed_lf = vt_lf;
    float wheel_speed_rf = vt_rf;
    float wheel_speed_lb = vt_lb;
    float wheel_speed_rb = vt_rb;
    float max_speed = fmaxf(fmaxf(fabsf(wheel_speed_lf), fabsf(wheel_speed_rf)),
                            fmaxf(fabsf(wheel_speed_lb), fabsf(wheel_speed_rb)));
    // 四个轮向电机按同一比例限幅，保留底盘解算得到的各轮速度比例。
    float scale = max_speed > CHASSIS_MAX_WHEEL_SPEED ? CHASSIS_MAX_WHEEL_SPEED / max_speed : 1.0f;
    wheel_speed_lf *= scale;
    wheel_speed_rf *= scale;
    wheel_speed_lb *= scale;
    wheel_speed_rb *= scale;
#ifdef CHASSIS_DRIVE_SWERVE
    //  设置舵向电机角度
    DJIMotorSetRef(steer_motor_lf, steer_target_lf);
    DJIMotorSetRef(steer_motor_rf, steer_target_rf);
    DJIMotorSetRef(steer_motor_lb, steer_target_lb);
    DJIMotorSetRef(steer_motor_rb, steer_target_rb);
    // 设置轮向电机速度
    DJIMotorSetRef(motor_lf, wheel_speed_lf * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_rf, wheel_speed_rf * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_lb, wheel_speed_lb * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_rb, wheel_speed_rb * LINEAR_TO_MOTOR);
#else
    // 设置轮子速度
    DJIMotorSetRef(motor_lf, wheel_speed_lf * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_rf, wheel_speed_rf * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_lb, wheel_speed_lb * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_rb, wheel_speed_rb * LINEAR_TO_MOTOR);
#endif
}

/**
 * @brief 根据每个轮子的速度和舵角反馈，计算底盘几何中心的实际运动速度。
 *        当前不融合 IMU；若后续用 IMU 加速度估速，需要补偿 IMU 相对几何中心的安装偏移。
 *
 */
static void EstimateSpeed()
{
    // 故障时先清零，正常时再用本周期反馈填入 mm/s 与 degree/s。
    chassis_feedback_data.real_vx = 0;
    chassis_feedback_data.real_vy = 0;
    chassis_feedback_data.real_wz = 0;
    // 任一轮组反馈离线时，反馈值保持为零，避免把过期数据继续上传。
    if (!DaemonIsFeedbackValid(chassis_cmd_daemon))
        return;
    float speed_lf = motor_lf->measure.speed_aps * MOTOR_TO_LINEAR;
    float speed_rf = motor_rf->measure.speed_aps * MOTOR_TO_LINEAR;
    float speed_lb = motor_lb->measure.speed_aps * MOTOR_TO_LINEAR;
    float speed_rb = motor_rb->measure.speed_aps * MOTOR_TO_LINEAR;
    if (motor_lf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) speed_lf = -speed_lf;
    if (motor_rf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) speed_rf = -speed_rf;
    if (motor_lb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) speed_lb = -speed_lb;
    if (motor_rb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) speed_rb = -speed_rb;
#ifdef CHASSIS_DRIVE_SWERVE
    // 转子角速度换算为轮面线速度后，使用单圈舵角和各轮前向零位还原轮子滚动方向。
    float angle_lf = (steer_motor_lf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f) *
        (steer_motor_lf->measure.angle_single_round -
        SWERVE_STEER_ZERO_ECD_LF * ECD_ANGLE_COEF_DJI) * DEGREE_2_RAD;
    float angle_rf = (steer_motor_rf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f) *
        (steer_motor_rf->measure.angle_single_round -
        SWERVE_STEER_ZERO_ECD_RF * ECD_ANGLE_COEF_DJI) * DEGREE_2_RAD;
    float angle_lb = (steer_motor_lb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f) *
        (steer_motor_lb->measure.angle_single_round -
        SWERVE_STEER_ZERO_ECD_LB * ECD_ANGLE_COEF_DJI) * DEGREE_2_RAD;
    float angle_rb = (steer_motor_rb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f) *
        (steer_motor_rb->measure.angle_single_round -
        SWERVE_STEER_ZERO_ECD_RB * ECD_ANGLE_COEF_DJI) * DEGREE_2_RAD;
    float vx_lf = speed_lf * arm_cos_f32(angle_lf);
    float vx_rf = speed_rf * arm_cos_f32(angle_rf);
    float vx_lb = speed_lb * arm_cos_f32(angle_lb);
    float vx_rb = speed_rb * arm_cos_f32(angle_rb);
    float vy_lf = speed_lf * arm_sin_f32(angle_lf);
    float vy_rf = speed_rf * arm_sin_f32(angle_rf);
    float vy_lb = speed_lb * arm_sin_f32(angle_lb);
    float vy_rb = speed_rb * arm_sin_f32(angle_rb);
    // 四轮位置关于底盘几何中心对称，平均轮心速度给出平移速度；位置加权项给出角速度。
    chassis_feedback_data.real_vx = (vx_lf + vx_rf + vx_lb + vx_rb) * 0.25f;
    chassis_feedback_data.real_vy = (vy_lf + vy_rf + vy_lb + vy_rb) * 0.25f;
    float rotation = (-HALF_TRACK_WIDTH * vx_lf - HALF_WHEEL_BASE * vy_lf) +
                     ( HALF_TRACK_WIDTH * vx_rf - HALF_WHEEL_BASE * vy_rf) +
                     (-HALF_TRACK_WIDTH * vx_lb + HALF_WHEEL_BASE * vy_lb) +
                     ( HALF_TRACK_WIDTH * vx_rb + HALF_WHEEL_BASE * vy_rb);
    chassis_feedback_data.real_wz = rotation /
        (4.0f * (HALF_WHEEL_BASE * HALF_WHEEL_BASE + HALF_TRACK_WIDTH * HALF_TRACK_WIDTH)) / DEGREE_2_RAD;
#else
#ifdef CHASSIS_DRIVE_MECANUM
    // 按麦轮滚子方向反解平移和旋转速度；输入轮速已经还原为逻辑方向的 mm/s。
    chassis_feedback_data.real_vx = (-speed_lf - speed_rf + speed_lb + speed_rb) * 0.25f;
    chassis_feedback_data.real_vy = (-speed_lf + speed_rf - speed_lb + speed_rb) * 0.25f;
    chassis_feedback_data.real_wz = -(speed_lf + speed_rf + speed_lb + speed_rb) /
        (4.0f * (HALF_WHEEL_BASE + HALF_TRACK_WIDTH) * DEGREE_2_RAD);
#else
    // X 型全向轮按各滚动方向的速度投影反解底盘平移及旋转速度。
    chassis_feedback_data.real_vx = (-speed_lf + speed_rf - speed_lb + speed_rb) / (4.0f * COS45);
    chassis_feedback_data.real_vy = (speed_lf + speed_rf - speed_lb - speed_rb) / (4.0f * COS45);
    chassis_feedback_data.real_wz = (speed_lf + speed_rf + speed_lb + speed_rb) / (4.0f * OMNI_L);
#endif
#endif
}

/* 机器人底盘控制核心任务 */
/**
 * @brief 接收控制命令，检查命令与电机反馈，运行所选底盘解算并发布反馈。
 */
void ChassisTask()
{
    // 单板从消息中心取命令，底盘板从双板 CAN 通信取命令。
    uint8_t command_received = 0;
#ifdef ONE_BOARD
    command_received = SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    command_received = CANCommReceive(chasiss_can_comm, &chassis_cmd_recv);
#endif
    // CANCommReceive 返回的是“最近有效整包仍在线”，不要求本周期一定收到新帧。
    // 无效 CRC 包不会刷新通信 daemon；通信超时或 Bus-Off 未恢复时返回 0。
    if (command_received)
        DaemonReload(chassis_cmd_daemon);

    uint32_t fault_flags = APP_FAULT_NONE;
    if (DaemonIsOnline(chassis_cmd_daemon) == 0)
        fault_flags |= APP_FAULT_COMMAND_STALE;

#ifdef CHASSIS_BOARD
    // 双板通信自身离线立即参与停机仲裁，不再等待底盘命令 daemon 的下一层超时。
    if (CANCommIsOnline(chasiss_can_comm) == 0)
        fault_flags |= APP_FAULT_COMMAND_STALE;
#endif

    const uint8_t feedback_valid = ChassisFeedbackValid();
    DaemonSetFeedbackValid(chassis_cmd_daemon, feedback_valid);
    if (feedback_valid == 0)
        fault_flags |= APP_FAULT_FEEDBACK_STALE;

    if (ChassisCommandValid() == 0)
        fault_flags |= APP_FAULT_INVALID_COMMAND;

    // 底盘上传的故障位汇总本周期检查结果，任意故障都会清零命令并停机。
    DaemonSetFaultFlags(chassis_cmd_daemon, fault_flags);
    chassis_feedback_data.fault_flags = (uint8_t)DaemonGetFaultFlags(chassis_cmd_daemon);

    if (chassis_feedback_data.fault_flags != APP_FAULT_NONE)
    {
        // 命令、反馈、通信或输入数据任一故障，清空控制量并进入零力矩。
        memset(&chassis_cmd_recv, 0, sizeof(chassis_cmd_recv));
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
    }

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
        SetChassisEnabled(0);

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_ZERO_FORCE:
        chassis_cmd_recv.vx = 0;
        chassis_cmd_recv.vy = 0;
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
    {
        // 用云台与底盘的角度误差生成旋转速度，并限制跟随速度。
        float offset = theta_format(chassis_cmd_recv.offset_angle);
        chassis_cmd_recv.wz = -50.0f * offset;
        chassis_cmd_recv.wz = float_constrain(chassis_cmd_recv.wz, -4500.0f, 4500.0f);
        break;
    }
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 4000;
        break;
    default:
        break;
    }

    // 把云台坐标系下的平移命令旋转到车体坐标系；x 向前、y 向右。
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 按编译时选择的底盘类型，将车体速度解算成各轮参考速度。
#if defined(CHASSIS_DRIVE_MECANUM)
    MecanumCalculate();
#elif defined(CHASSIS_DRIVE_OMNI_X)
    OmniCalculate();
#elif defined(CHASSIS_DRIVE_SWERVE)
    SwerveCalculate();
#endif

    // 限制轮向电机速度并写入轮向、舵向闭环参考值。
    LimitChassisOutput();
    if (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE)
        SetChassisEnabled(1);

    // 根据电机的反馈速度和舵角计算真实速度；当前不使用 IMU。
    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 将速度与故障反馈发给本板订阅者或云台板。
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}

/**
 * @brief 检查当前底盘构型所需的电机反馈和 CAN 总线是否有效。
 * @return uint8_t 所有轮向电机及舵轮构型下的舵向电机均在线且 CAN 就绪时返回 1，否则返回 0。
 */
static uint8_t ChassisFeedbackValid(void)
{
    // DJI 电机收到有效反馈时刷新各自 daemon；任一 M3508 或 GM6020 超时，整车停机。
    // CANIsReady 同时拦截 CAN 总线未运行、Bus-Off 或尚未恢复的情况。
    if (!DaemonIsOnline(motor_lf->daemon) || !CANIsReady(motor_lf->motor_can_instance) ||
        !DaemonIsOnline(motor_rf->daemon) || !CANIsReady(motor_rf->motor_can_instance) ||
        !DaemonIsOnline(motor_lb->daemon) || !CANIsReady(motor_lb->motor_can_instance) ||
        !DaemonIsOnline(motor_rb->daemon) || !CANIsReady(motor_rb->motor_can_instance))
        return 0;
#ifdef CHASSIS_DRIVE_SWERVE
    if (!DaemonIsOnline(steer_motor_lf->daemon) || !CANIsReady(steer_motor_lf->motor_can_instance) ||
        !DaemonIsOnline(steer_motor_rf->daemon) || !CANIsReady(steer_motor_rf->motor_can_instance) ||
        !DaemonIsOnline(steer_motor_lb->daemon) || !CANIsReady(steer_motor_lb->motor_can_instance) ||
        !DaemonIsOnline(steer_motor_rb->daemon) || !CANIsReady(steer_motor_rb->motor_can_instance))
        return 0;
#endif
    return 1;
}

/**
 * @brief 检查底盘命令中的速度、角度和运行模式是否受支持。
 * @return uint8_t 所有浮点字段均为有限值且模式有效时返回 1，否则返回 0。
 */
static uint8_t ChassisCommandValid(void)
{
    // 命令来自消息或 CAN 边界；拒绝非有限速度值及当前不支持的底盘模式。
    if (!isfinite(chassis_cmd_recv.vx) || !isfinite(chassis_cmd_recv.vy) ||
        !isfinite(chassis_cmd_recv.wz) || !isfinite(chassis_cmd_recv.offset_angle))
        return 0;
    return chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE ||
           chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE ||
           chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW ||
           chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW;
}

/**
 * @brief 统一使能或停止底盘轮向电机及舵向电机。
 * @param enabled 非零时使能电机；为零时清除 PID 状态、停止输出，并在舵轮构型下保持当前舵角。
 */
static void SetChassisEnabled(uint8_t enabled)
{
    if (enabled)
    {
        // 同一使能门控覆盖驱动轮和舵向电机，避免故障恢复时只启用部分轮组。
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
#ifdef CHASSIS_DRIVE_SWERVE
        DJIMotorEnable(steer_motor_lf);
        DJIMotorEnable(steer_motor_rf);
        DJIMotorEnable(steer_motor_lb);
        DJIMotorEnable(steer_motor_rb);
#endif
        return;
    }

    // 停机前清除闭环历史，避免重新使能时沿用故障前的积分量。
    if (motor_lf->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(motor_lf);
    if (motor_rf->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(motor_rf);
    if (motor_lb->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(motor_lb);
    if (motor_rb->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(motor_rb);
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
#ifdef CHASSIS_DRIVE_SWERVE
    if (steer_motor_lf->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(steer_motor_lf);
    if (steer_motor_rf->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(steer_motor_rf);
    if (steer_motor_lb->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(steer_motor_lb);
    if (steer_motor_rb->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(steer_motor_rb);
    DJIMotorStop(steer_motor_lf);
    DJIMotorStop(steer_motor_rf);
    DJIMotorStop(steer_motor_lb);
    DJIMotorStop(steer_motor_rb);
    // 舵向参考同步为当前位置，零力矩期间不要求舵轮转向。
    steer_target_lf = (steer_motor_lf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f) * steer_motor_lf->measure.total_angle;
    steer_target_rf = (steer_motor_rf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f) * steer_motor_rf->measure.total_angle;
    steer_target_lb = (steer_motor_lb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f) * steer_motor_lb->measure.total_angle;
    steer_target_rb = (steer_motor_rb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f) * steer_motor_rb->measure.total_angle;
#endif
}

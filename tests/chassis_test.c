#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "crc8.h"
#include "../application/chassis/chassis.c"

UART_HandleTypeDef huart1;
CAN_HandleTypeDef hcan1, hcan2, hcan3;
static uint32_t now_ms, irq_mask;
static CANBusStatus bus;
static uint8_t bus_ready = 1;
static uint8_t sent[64];
static size_t sent_len;
static int fail_fragment = -1, tx_count;
static float last_budget;

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t value) { irq_mask = value; }
float DWT_GetDeltaT(uint32_t *count) { *count = now_ms; return 0.005f; }
float DWT_GetTimeline_ms(void) { return now_ms; }
void CANServiceTask(void) {}
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *can = calloc(1, sizeof(*can));
    assert(can);
    can->can_handle = config->can_handle;
    can->tx_id = config->tx_id; can->rx_id = config->rx_id;
    can->id = config->id; can->can_module_callback = config->can_module_callback;
    can->txconf.DataLength = 8;
    return can;
}
uint8_t CANIsReady(const CANInstance *can) { return can && bus_ready; }
const CANBusStatus *CANGetBusStatus(const CANInstance *can) { return can ? &bus : NULL; }
void CANSetLinkOnline(const CANInstance *can, uint8_t online) { (void)can; bus.online = online; }
void CANSetDLC(CANInstance *can, uint8_t length) { can->txconf.DataLength = length; }
uint8_t CANTransmit(CANInstance *can, float timeout)
{
    assert(timeout <= CAN_COMM_TX_BUDGET_MS);
    last_budget = timeout;
    if (!bus_ready || tx_count++ == fail_fragment) return 0;
    if (can->tx_id == 0x311)
    {
        assert(sent_len + can->txconf.DataLength <= sizeof(sent));
        memcpy(sent + sent_len, can->tx_buff, can->txconf.DataLength);
        sent_len += can->txconf.DataLength;
    }
    return 1;
}
attitude_t *INS_Init(void) { static attitude_t imu; return &imu; }
referee_info_t *UITaskInit(UART_HandleTypeDef *uart, Referee_Interactive_info_t *ui)
{ (void)uart; (void)ui; return NULL; }
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config) { (void)config; return NULL; }

static void near(float actual, float expected, float tolerance)
{
    if (fabsf(actual - expected) > tolerance)
        fprintf(stderr, "actual=%f expected=%f tolerance=%f\n", actual, expected, tolerance);
    assert(fabsf(actual - expected) <= tolerance);
}
static void inject(CANInstance *can, const uint8_t *data, uint8_t len)
{
    memcpy(can->rx_buff, data, len); can->rx_len = len; can->can_module_callback(can);
}
static void command(Chassis_Ctrl_Cmd_s cmd, uint8_t corrupt)
{
    uint8_t packet[sizeof(cmd) + CAN_COMM_OFFSET_BYTES] = {CAN_COMM_HEADER, sizeof(cmd)};
    memcpy(packet + 2, &cmd, sizeof(cmd));
    packet[sizeof(packet) - 2] = crc_8(packet + 2, sizeof(cmd)) ^ corrupt;
    packet[sizeof(packet) - 1] = CAN_COMM_TAIL;
    for (size_t i = 0; i < sizeof(packet); i += 8)
        inject(chasiss_can_comm->can_ins, packet + i, sizeof(packet) - i >= 8 ? 8 : sizeof(packet) - i);
}
static void refresh(void)
{
    DaemonReload(motor_lf->daemon);
    DaemonReload(motor_rf->daemon);
    DaemonReload(motor_lb->daemon);
    DaemonReload(motor_rb->daemon);
#ifdef CHASSIS_DRIVE_SWERVE
    DaemonReload(steer_motor_lf->daemon);
    DaemonReload(steer_motor_rf->daemon);
    DaemonReload(steer_motor_lb->daemon);
    DaemonReload(steer_motor_rb->daemon);
#endif
}
static void task(void) { sent_len = 0; tx_count = 0; ChassisTask(); }
static void stopped(uint8_t expected)
{
    assert((motor_lf->stop_flag == MOTOR_STOP) == expected);
    assert((motor_rf->stop_flag == MOTOR_STOP) == expected);
    assert((motor_lb->stop_flag == MOTOR_STOP) == expected);
    assert((motor_rb->stop_flag == MOTOR_STOP) == expected);
#ifdef CHASSIS_DRIVE_SWERVE
    assert((steer_motor_lf->stop_flag == MOTOR_STOP) == expected);
    assert((steer_motor_rf->stop_flag == MOTOR_STOP) == expected);
    assert((steer_motor_lb->stop_flag == MOTOR_STOP) == expected);
    assert((steer_motor_rb->stop_flag == MOTOR_STOP) == expected);
#endif
}
#ifdef CHASSIS_DRIVE_SWERVE
static void set_angle(DJIMotorInstance *motor, float degrees)
{
    motor->measure.total_angle = degrees;
    motor->measure.angle_single_round = fmodf(degrees + 3600.0f, 360.0f);
}
static void move_feedback(float vx, float vy, float wz)
{
    chassis_vx = vx; chassis_vy = vy; chassis_cmd_recv.wz = wz;
    for (int pass = 0; pass < 2; ++pass)
    {
        SwerveCalculate();
        set_angle(steer_motor_lf, steer_target_lf);
        set_angle(steer_motor_rf, steer_target_rf);
        set_angle(steer_motor_lb, steer_target_lb);
        set_angle(steer_motor_rb, steer_target_rb);
    }
    LimitChassisOutput();
    motor_lf->measure.speed_aps = motor_lf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ?
        -motor_lf->motor_controller.pid_ref : motor_lf->motor_controller.pid_ref;
    motor_rf->measure.speed_aps = motor_rf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ?
        -motor_rf->motor_controller.pid_ref : motor_rf->motor_controller.pid_ref;
    motor_lb->measure.speed_aps = motor_lb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ?
        -motor_lb->motor_controller.pid_ref : motor_lb->motor_controller.pid_ref;
    motor_rb->measure.speed_aps = motor_rb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ?
        -motor_rb->motor_controller.pid_ref : motor_rb->motor_controller.pid_ref;
    DaemonSetFeedbackValid(chassis_cmd_daemon, 1);
    EstimateSpeed();
    near(chassis_feedback_data.real_vx, vx, 0.05f);
    near(chassis_feedback_data.real_vy, vy, 0.05f);
    near(chassis_feedback_data.real_wz, wz, 0.02f);
}
#endif
int main(void)
{
    ChassisInit(); stopped(1);
    assert(chasiss_can_comm->can_ins->can_handle == &hcan3);
    assert(chasiss_can_comm->can_ins->tx_id == 0x311 && chasiss_can_comm->can_ins->rx_id == 0x312);
    assert(!CANCommIsOnline(chasiss_can_comm));
    refresh();
    Chassis_Ctrl_Cmd_s cmd = {.vx = 1000, .chassis_mode = CHASSIS_NO_FOLLOW};
    command(cmd, 0); task(); stopped(0);
    assert(chassis_feedback_data.fault_flags == APP_FAULT_NONE);
    assert(bus.online && sent_len == sizeof(Chassis_Upload_Data_s) + CAN_COMM_OFFSET_BYTES);
    assert(sent[0] == CAN_COMM_HEADER && sent[1] == sizeof(Chassis_Upload_Data_s));
    assert(sent[sent_len - 2] == crc_8(sent + 2, sizeof(Chassis_Upload_Data_s)));
#ifdef CHASSIS_DRIVE_SWERVE
    assert(motor_lf->motor_can_instance->rx_id == 0x201);
    assert(steer_motor_lf->motor_can_instance->rx_id == 0x205);
    assert(steer_motor_rf->motor_can_instance->rx_id == 0x206);
    assert(steer_motor_lb->motor_can_instance->rx_id == 0x208);
    assert(steer_motor_rb->motor_can_instance->rx_id == 0x207);
    assert(steer_motor_lf->motor_type == GM6020 && steer_motor_lf->motor_can_instance->can_handle == &hcan1);
    assert(steer_motor_rf->motor_type == GM6020 && steer_motor_rf->motor_can_instance->can_handle == &hcan1);
    assert(steer_motor_lb->motor_type == GM6020 && steer_motor_lb->motor_can_instance->can_handle == &hcan1);
    assert(steer_motor_rb->motor_type == GM6020 && steer_motor_rb->motor_can_instance->can_handle == &hcan1);
    near(steer_target_lf, 0, 0.001f); near(steer_target_rf, 0, 0.001f);
    near(steer_target_lb, 0, 0.001f); near(steer_target_rb, 0, 0.001f);
    near(motor_lf->motor_controller.pid_ref, 1000 * LINEAR_TO_MOTOR, 0.02f);
    chassis_vx = -1000; chassis_vy = 0; chassis_cmd_recv.wz = 0; SwerveCalculate();
    near(vt_lf, -1000, 0.001f); near(steer_target_lf, 0, 0.001f);
    set_angle(steer_motor_lf, 350); chassis_vx = 1000*cosf(10*DEGREE_2_RAD); chassis_vy = 1000*sinf(10*DEGREE_2_RAD);
    SwerveCalculate(); near(steer_target_lf, 370, 0.002f);
    near(vt_lf, 1000*powf(cosf(20*DEGREE_2_RAD), 3), 0.01f);
    float held = steer_target_lf; chassis_vx = chassis_vy = 0; SwerveCalculate();
    near(steer_target_lf, held, 0.001f); near(vt_lf, 0, 0.001f);
    for (int vx = -600; vx <= 600; vx += 200)
        for (int vy = -600; vy <= 600; vy += 200)
            for (int wz = -120; wz <= 120; wz += 60) move_feedback(vx, vy, wz);
    steer_motor_lf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    steer_motor_rf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    steer_motor_lb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    steer_motor_rb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    set_angle(steer_motor_lf, -90); set_angle(steer_motor_rf, -90);
    set_angle(steer_motor_lb, -90); set_angle(steer_motor_rb, -90);
    motor_lf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf->measure.speed_aps = motor_rf->measure.speed_aps =
        motor_lb->measure.speed_aps = motor_rb->measure.speed_aps = -100 * LINEAR_TO_MOTOR;
    DaemonSetFeedbackValid(chassis_cmd_daemon, 1);
    EstimateSpeed();
    near(chassis_feedback_data.real_vx, 0, 0.05f);
    near(chassis_feedback_data.real_vy, 100, 0.05f);
    near(chassis_feedback_data.real_wz, 0, 0.02f);
    steer_motor_lf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_motor_rf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_motor_lb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_motor_rb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    vt_lf = 10000; vt_rf = 5000; vt_lb = -2500; vt_rb = 1250; LimitChassisOutput();
    near(motor_lf->motor_controller.pid_ref, CHASSIS_MAX_WHEEL_SPEED * LINEAR_TO_MOTOR, 0.01f);
    near(motor_rf->motor_controller.pid_ref, CHASSIS_MAX_WHEEL_SPEED * LINEAR_TO_MOTOR/2, 0.01f);
    near(motor_lb->motor_controller.pid_ref, -CHASSIS_MAX_WHEEL_SPEED * LINEAR_TO_MOTOR/4, 0.01f);
#else
    for (int vx = -600; vx <= 600; vx += 200)
        for (int vy = -600; vy <= 600; vy += 200)
            for (int wz = -120; wz <= 120; wz += 60)
            {
                chassis_vx = vx; chassis_vy = vy; chassis_cmd_recv.wz = wz;
#ifdef CHASSIS_DRIVE_MECANUM
                MecanumCalculate();
#else
                OmniCalculate();
#endif
                LimitChassisOutput();
                motor_lf->measure.speed_aps = -motor_lf->motor_controller.pid_ref;
                motor_rf->measure.speed_aps = -motor_rf->motor_controller.pid_ref;
                motor_lb->measure.speed_aps = -motor_lb->motor_controller.pid_ref;
                motor_rb->measure.speed_aps = -motor_rb->motor_controller.pid_ref;
                EstimateSpeed();
                near(chassis_feedback_data.real_vx, vx, 0.01f);
                near(chassis_feedback_data.real_vy, vy, 0.01f);
                near(chassis_feedback_data.real_wz, wz, 0.01f);
            }
#endif
    cmd = (Chassis_Ctrl_Cmd_s){.vx=100, .vy=200, .offset_angle=90, .chassis_mode=CHASSIS_NO_FOLLOW};
    command(cmd, 0); task(); near(chassis_vx, -200, 0.001f); near(chassis_vy, 100, 0.001f);
    cmd.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; command(cmd,0); task(); near(chassis_cmd_recv.wz,-4500,0.001f);
    cmd.chassis_mode = CHASSIS_ROTATE; command(cmd,0); task(); near(chassis_cmd_recv.wz,4000,0.001f);
    cmd.chassis_mode = CHASSIS_NO_FOLLOW; command(cmd,0); task();
    uint32_t received = chasiss_can_comm->rx_ok_count;
    command(cmd, 1); assert(chasiss_can_comm->rx_ok_count == received);
    uint8_t first[8] = {CAN_COMM_HEADER, sizeof(cmd)};
    inject(chasiss_can_comm->can_ins, first, 8); now_ms += CAN_COMM_RX_FRAGMENT_TIMEOUT_MS;
    command(cmd,0); assert(chasiss_can_comm->rx_ok_count == received + 1);
    bus.bus_off_count++; task(); stopped(1);
    assert(chassis_feedback_data.fault_flags & APP_FAULT_COMMAND_STALE);
    assert(!CANCommIsOnline(chasiss_can_comm));
    command(cmd,0); task(); stopped(0);
    for (unsigned i=0; i<DAEMON_COUNT_FROM_MS(APP_COMMAND_TIMEOUT_MS); ++i) { now_ms+=10; DaemonTask(); }
    refresh(); task(); stopped(1); assert(chassis_feedback_data.fault_flags & APP_FAULT_COMMAND_STALE);
    command(cmd,0); task(); stopped(0);
#ifdef CHASSIS_DRIVE_SWERVE
    steer_motor_lb->daemon->temp_count = 0; task(); stopped(1);
    assert(chassis_feedback_data.fault_flags & APP_FAULT_FEEDBACK_STALE);
    near(chassis_feedback_data.real_vx,0,0); refresh();
#endif
    bus_ready = 0; task(); stopped(1); bus_ready=1; command(cmd,0); task(); stopped(0);
    cmd.vx = NAN; command(cmd,0); task(); stopped(1);
    assert(chassis_feedback_data.fault_flags & APP_FAULT_INVALID_COMMAND);
    cmd.vx=1000; cmd.chassis_mode=CHASSIS_FOLDED_ROTATE; command(cmd,0); task(); stopped(1);
    cmd.chassis_mode=CHASSIS_ZERO_FORCE; command(cmd,0); task(); stopped(1);
    near(motor_lf->motor_controller.pid_ref,0,0);
    PIDInstance pid; PID_Init_Config_s cfg={.Kp=3,.Ki=2,.MaxOut=100,.IntegralLimit=10};
    PIDInit(&pid,&cfg); PIDCalculate(&pid,0,10); assert(pid.Iout > 0);
    PIDResetState(&pid); near(pid.Kp,3,0); near(pid.Ki,2,0); near(pid.Iout,0,0); near(pid.Output,0,0);
    sent_len=0; tx_count=0; fail_fragment=1; CANCommSend(chasiss_can_comm,(uint8_t *)&chassis_feedback_data);
    assert(tx_count==2 && sent_len==8 && chasiss_can_comm->tx_error_count > 0);
    assert(last_budget<=CAN_COMM_TX_BUDGET_MS); fail_fragment=-1;
    chasiss_can_comm->comm_daemon->temp_count=0; now_ms+=100; sent_len=0; tx_count=0;
    CANCommSend(chasiss_can_comm,(uint8_t *)&chassis_feedback_data);
    unsigned attempts=chasiss_can_comm->tx_attempt_count;
    CANCommSend(chasiss_can_comm,(uint8_t *)&chassis_feedback_data);
    assert(chasiss_can_comm->tx_attempt_count==attempts);
    puts("PASS production chassis + DJI + CANComm + daemon + PID: kinematics, speed feedback, modes and link failures");
}

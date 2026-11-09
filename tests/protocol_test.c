#include <stdio.h>
#include <stddef.h>
#include ROBOT_DEF_SOURCE
#include "can_comm.h"
_Static_assert(sizeof(attitude_t) == 40, "IMU feedback must contain ten 32-bit floats");
_Static_assert(offsetof(Chassis_Upload_Data_s, chassis_imu_data) == 0, "IMU feedback must be the first field");
_Static_assert(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE, "Chassis feedback must fit CANComm payload buffers");
_Static_assert(sizeof(App_Fault_s) == 4, "Shared fault status must occupy four bytes");
_Static_assert(offsetof(App_Fault_s, motor_offline_flags) == 0, "Motor flags must be the first byte");
_Static_assert(offsetof(App_Fault_s, comm_fault_flags) == 1, "Communication flags must be the second byte");
_Static_assert(offsetof(App_Fault_s, imu_fault_flags) == 2, "IMU flags must be the third byte");
_Static_assert(offsetof(App_Fault_s, control_fault_flags) == 3, "Control flags must be the fourth byte");
_Static_assert(_Generic(&((Chassis_Upload_Data_s *)0)->fault, App_Fault_s *: 1, default: 0), "Chassis must use shared fault type");
_Static_assert(_Generic(&((Gimbal_Upload_Data_s *)0)->fault, App_Fault_s *: 1, default: 0), "Gimbal must use shared fault type");
_Static_assert(_Generic(&((Shoot_Upload_Data_s *)0)->fault, App_Fault_s *: 1, default: 0), "Shoot must use shared fault type");
int main(void)
{
    printf("%zu %zu %zu %zu %zu %zu %zu\n", sizeof(Chassis_Ctrl_Cmd_s), sizeof(Chassis_Upload_Data_s),
           offsetof(Chassis_Upload_Data_s, fault), offsetof(Chassis_Upload_Data_s, real_vx),
           offsetof(Chassis_Upload_Data_s, real_vy), offsetof(Chassis_Upload_Data_s, real_wz),
           sizeof(((Chassis_Upload_Data_s *)0)->fault));
}

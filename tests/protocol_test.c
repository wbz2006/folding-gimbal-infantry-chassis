#include <stdio.h>
#include <stddef.h>
#include ROBOT_DEF_SOURCE
int main(void)
{
    printf("%zu %zu %zu %zu %zu %zu\n", sizeof(Chassis_Ctrl_Cmd_s), sizeof(Chassis_Upload_Data_s),
           offsetof(Chassis_Upload_Data_s, fault_flags), offsetof(Chassis_Upload_Data_s, real_vx),
           offsetof(Chassis_Upload_Data_s, real_vy), offsetof(Chassis_Upload_Data_s, real_wz));
}

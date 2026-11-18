#pragma once
#include <stdint.h>
typedef struct { float Gyro[3], Accel[3], Roll, Pitch, Yaw, YawTotalAngle; } attitude_t;
attitude_t *INS_Init(void);
attitude_t *INS_GetData(void);
uint8_t INS_GetFaultFlags(void);

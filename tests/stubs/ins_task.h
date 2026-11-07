#pragma once
typedef struct { float Gyro[3], Accel[3], Roll, Pitch, Yaw, YawTotalAngle; } attitude_t;
attitude_t *INS_Init(void);

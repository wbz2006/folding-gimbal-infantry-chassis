# 底盘应用

`ChassisInit()`、`ChassisTask()`沿用原接口。在 `application/robot_def.h` 选择 `CHASSIS_DRIVE_SWERVE`（默认）、`CHASSIS_DRIVE_MECANUM` 或 `CHASSIS_DRIVE_OMNI_X`，只能启用一种。

## 四舵轮接线和参数

8 个电机均使用 CAN1。四个轮组按 LF（左前）、RF（右前）、LB（左后）、RB（右后）分别配置，ID 暂沿用原轮向电机排列：

| 轮组 | M3508 ID / 反馈 ID | GM6020 ID / 反馈 ID |
| --- | --- | --- |
| 左前 | 1 / 0x201 | 1 / 0x205 |
| 右前 | 2 / 0x202 | 2 / 0x206 |
| 左后 | 4 / 0x204 | 4 / 0x208 |
| 右后 | 3 / 0x203 | 3 / 0x207 |

轮向组帧发送 ID 为 `0x200`，舵向组帧发送 ID 为 `0x1FF`。M3508 与 GM6020 的拨码编号可以相同，其反馈 ID 不同；不要将这里的 M3508 改成 5～8，否则会与 GM6020 的反馈冲突。

尺寸沿用轴距 350 mm、轮距 300 mm、轮半径 60 mm、驱动减速比 19:1。GM6020 按直接驱动舵轴处理。以下标定量在 `robot_def.h` 集中设置：

- `SWERVE_STEER_ZERO_ECD_LF/RF/LB/RB`：各轮组朝车头时的 GM6020 编码器值，当前均为 0 占位。
- 轮向和舵向电机方向分别通过 `chassis_motor_config.controller_setting_init_config.motor_reverse_flag` 与 `steer_motor_config.controller_setting_init_config.motor_reverse_flag` 设置，按 LF/RF/LB/RB 独立配置。
- `CHASSIS_MAX_WHEEL_SPEED`：实际轮面线速度上限，当前约 2391.4 mm/s；按 6000 rpm 电机上限、268/17 减速比和 60 mm 轮半径换算。

GM6020 使用角度环串速度环，初始参数为角度 Kp=8、输出限幅 720 degree/s，速度 Kp=20、输出限幅 20000。轮向沿用原 M3508 速度/电流环参数。零位、方向和 PID 均需架空逐轮标定后才能落地运行。

## 解算和单位

平面坐标使用 x 向前、y 向右、wz 逆时针为正。命令 `vx/vy` 单位为 mm/s，`wz` 单位为 degree/s，`offset_angle` 单位为 degree，保留原 CMD 到底盘的旋转变换。

第 i 轮相对旋转中心位置为 `(x_i,y_i)`，角速度换算为 rad/s 后，轮组速度为 `vx_i=vx+wz*y_i`、`vy_i=vy-wz*x_i`。轮速由向量长度得到，舵向由 `atan2(vy_i,vx_i)` 得到：

1. 相对当前多圈编码器角度，取 [-180°,180°] 内最短角差。
2. 角差超过 90° 时使用相差 180° 的等效朝向，同时反转轮向。
3. 轮向速度乘 `cos(舵角误差)^3`，转向未完成时降低驱动力。
4. 零速度保持最近舵向目标；零力矩停机时更新为当前实测舵向。
5. 四轮先按轮面线速度上限等比例限速，再按轮半径及 268/17 减速比换算成电机驱动使用的转子 degree/s。

`EstimateSpeed()`用实测轮向速度和实测舵角反解底盘几何中心的 `real_vx/real_vy/real_wz`，单位分别为 mm/s、mm/s、degree/s。使用实际反馈，不用目标速度代替。麦轮、X 型全向轮的速度反解也已补齐。电机反馈失效时速度字段清零，并设置 `APP_FAULT_FEEDBACK_STALE`。当前没有融合 IMU，也不估计打滑。

`CHASSIS_NO_FOLLOW` 保持原行为（wz=0）；`CHASSIS_FOLLOW_GIMBAL_YAW` 用原跟随系数和限幅；`CHASSIS_ROTATE` 沿用原固定 wz=4000 degree/s，实际轮速受等比例限幅约束；`CHASSIS_ZERO_FORCE` 对全部 8 个电机输出零力矩。通信或任一电机反馈失效均进入零力矩，停机切换清除 PID 历史。

当前仅实现轮速限制，原裁判功率控制仍是 TODO，没有实现 8 电机功率模型或功率分配。

## 双板通信

对照 `dual-board_communication_demo_A/双板通信移植手册.md`：当前工程已有对应的 FDCAN Bus-Off 恢复、CRC、分片超时、整包读取和离线低频发送机制。本次保留这些实现，修正应用层在通信失效后继续等待第二层超时的问题，`CANCommIsOnline()`失效即当周期停机。

- 底盘 CAN3：发 `0x311`，收 `0x312`；云台对应发 `0x312`、收 `0x311`。CAN3 与八个底盘电机所在的 CAN1 分开。
- `Chassis_Ctrl_Cmd_s` 仍为 24 字节。`Chassis_Upload_Data_s` 由 10 字节扩为 22 字节，在旧字段后追加 `real_vx/real_vy/real_wz`，故障位偏移仍为 9，三个 float 偏移分别为 10、14、18。
- 按本次要求只修改底盘板，云台工程保持原样。后续云台必须同步反馈结构并重新编译烧录；旧 10 字节反馈接收程序不能直接配合新底盘使用。
- 调试观察 `can_bus_status[2]`（CAN3）的 `online/state/tec/rec/bus_off_count/recovery_success_count/rx_drop_count`，及 `chasiss_can_comm` 的收发计数。

CAN1 使用经典 CAN 1 Mbps。八个电机的 500 Hz 反馈约为 4000 帧/秒；两组电机控制帧在 500 Hz 下再增加约 1000 帧/秒。本次将控制任务改到 500 Hz，双板通信使用 CAN3，不占用 CAN1 带宽。实车仍需观察电机丢帧和总线错误计数。

## 验证

运行 `python tests/run_host_tests.py`，测试直接编译生产 `chassis.c`、DJI 驱动、CANComm、daemon、PID 和数学库，仅替换硬件 CAN 与时钟。覆盖三种构型的速度解算、舵向跨圈/反转/补偿、限速、模式、CRC、半包超时、Bus-Off 代次失效、断线停机、发送失败中断和底盘协议 ABI。

固件按原工具链执行 `make -j4`。主机测试不能证明实物的 CAN Bus-Off 恢复时序，需在两块板上验证断线、短路、对端掉电和恢复，不需要为此新建线上测试结构体。

## 知识库依据

`D:/Main_FIles/free-deng/rm_bbs_collection/舵轮控制方法-AI知识库.md` §2～§3，以及 `code/01_Qiqi_Rudder-Swerve-Control/docs/control-algorithm.md`：轮组矢量合成、最优舵角、三次余弦补偿。移植时采用本工程的 y 向右约定，不能直接套用知识库的 y 向左符号。

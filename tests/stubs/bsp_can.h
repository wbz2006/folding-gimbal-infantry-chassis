#pragma once
#include "main.h"
#define FDCAN
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_DLC_BYTES_8 8
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_BRS_OFF 0
typedef struct CANInstance CANInstance;
typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id, rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;
struct CANInstance {
    CAN_HandleTypeDef *can_handle;
    struct { uint32_t Identifier, IdType, TxFrameType, DataLength, FDFormat, BitRateSwitch; } txconf;
    uint32_t tx_id, rx_id;
    uint8_t rx_len, rx_buff[8], tx_buff[8];
    void (*can_module_callback)(CANInstance *);
    void *id;
};
typedef struct { uint32_t bus_off_count; uint8_t online; } CANBusStatus;
CANInstance *CANRegister(CAN_Init_Config_s *config);
uint8_t CANTransmit(CANInstance *can, float timeout);
void CANSetDLC(CANInstance *can, uint8_t length);
uint8_t CANIsReady(const CANInstance *can);
const CANBusStatus *CANGetBusStatus(const CANInstance *can);
void CANSetLinkOnline(const CANInstance *can, uint8_t online);
void CANServiceTask(void);

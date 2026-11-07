#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } CAN_HandleTypeDef;
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1, hcan2, hcan3;
#define hfdcan1 hcan1
#define hfdcan2 hcan2
#define hfdcan3 hcan3
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);

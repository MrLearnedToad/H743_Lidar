#ifndef _FDCAN_BSP_H_
#define _FDCAN_BSP_H_

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "fdcan.h"
uint8_t FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan);
uint8_t FDCAN2_Init(FDCAN_HandleTypeDef *hfdcan);
__weak void FDCAN_Message_Decode(FDCAN_HandleTypeDef *hfdcan, uint32_t StdId, uint8_t *RxData);
uint8_t FDCAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t StdId, uint32_t Length);
uint8_t FDCAN_SendData_Ext(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t ExtId, uint32_t Length, uint32_t Data_type);

extern uint8_t cmd_feedback[8];
#endif

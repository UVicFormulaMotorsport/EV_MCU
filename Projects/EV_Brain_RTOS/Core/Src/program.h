/*
 * program.h
 *
 *  Created on: Jul 2, 2022
 *  Author: Randon
 */

#include "main.h"

#define FALSE 0
#define TRUE !(FALSE)

// Function prototypes
void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *pTxHeader, uint32_t dlc, uint32_t ide, uint32_t rtr, uint32_t stdId);
void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint32_t fifo, uint32_t highId, uint32_t lowId, uint32_t highMask, uint32_t lowMask, uint32_t scale);
HAL_StatusTypeDef CANsend(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint32_t, uint32_t *);

void program();

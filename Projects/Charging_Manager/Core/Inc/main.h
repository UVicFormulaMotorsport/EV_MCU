/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct pilot_struct{ //????
	uint16_t cs_current;
	uint16_t mains_voltage;
	uint8_t cs_connected;
	uint8_t flags;
}pilot_struct;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void update_charger(uint16_t max_voltage, uint16_t max_current, uint8_t charging_status);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RST_Pin GPIO_PIN_0
#define RST_GPIO_Port GPIOC
#define VRX_Pin GPIO_PIN_1
#define VRX_GPIO_Port GPIOA
#define VRY_Pin GPIO_PIN_2
#define VRY_GPIO_Port GPIOA
#define SW_Input_Pin GPIO_PIN_3
#define SW_Input_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_4
#define LCD_RST_GPIO_Port GPIOA
#define SHTDN_Trigger_Pin GPIO_PIN_11
#define SHTDN_Trigger_GPIO_Port GPIOB
#define CHRG_EN_Pin GPIO_PIN_15
#define CHRG_EN_GPIO_Port GPIOA
#define LED__Pin GPIO_PIN_5
#define LED__GPIO_Port GPIOB

//Charging preset struct
typedef struct charge_profile{
	uint16_t preset_max_current;
	uint16_t preset_max_voltage;

}charge_profile;
/* USER CODE BEGIN Private defines */
#define BMS_COBID 0x20



//#ifndef CANBUS_VARS

//extern CAN_TxHeaderTypeDef chargerTxHeader; //CAN Tx Header
//extern CAN_RxHeaderTypeDef chargerRxHeader; //CAN Rx Header
//extern uint8_t tData[8] = { 0 }; // can message data
//extern uint32_t pTxMailbox; // Message mailbox
//extern CAN_FilterTypeDef sFilterConfig; //CAN filter configuration
//extern uint32_t CHARGER_RX = 0x18FF50E5;
//extern uint32_t CHARGER_TX = 0x1806E5F4;
//extern const uint16_t BMS_ID1 = 0x180 + BMS_COBID;//BMS uses standard length identifiers, therefore uint_16 is used
//extern const uint16_t BMS_ID2 = 0x280 + BMS_COBID;
//extern const uint16_t BMS_ID3 = 0x380 + BMS_COBID;
//#endif
////CHARGER VARIABLES
//#ifndef CHRGR_VARS
//extern uint16_t MAX_VOLTAGE = 0;
//extern uint16_t MAX_CURRENT = 0;
//
//#endif
//
////BMS VAR externs
//#ifndef BMS_VARS
////no double dipping extern variables
//extern int16_t Current_current = 0;
//extern uint16_t PILOT_FLAGS = 0;
//extern uint8_t CHARGER_OUTPUT_STATUS = 0;
//extern int8_t minimum_cell_temp = 0;
//extern int8_t max_cell_temp = 0;
//extern uint8_t state_of_charge = 0;
//extern uint8_t BMS_voltage = 0;
//extern uint32_t BMS_internal_state = 0;
//extern uint32_t BMS_errors1 = 0;
//extern uint32_t BMS_errors2;
//#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

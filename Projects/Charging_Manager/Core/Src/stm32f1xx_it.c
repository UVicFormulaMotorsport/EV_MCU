/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define false 0
#define true !(false)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t canRxData[8] = { 0 };
uint8_t uartRxData[8] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern pilot_struct pilot_info;


//CAN VARS

extern CAN_TxHeaderTypeDef chargerTxHeader; //CAN Tx Header
extern CAN_RxHeaderTypeDef chargerRxHeader; //CAN Rx Header

extern const uint16_t BMS_ID1;//BMS uses standard length identifiers, therefore uint_16 is used
extern const uint16_t BMS_ID2;
extern const uint16_t BMS_ID3;
extern uint16_t MAX_VOLTAGE;
extern uint16_t MAX_CURRENT;
extern uint16_t PILOT_FLAGS;
extern uint8_t CHARGER_OUTPUT_STATUS;

extern uint32_t CHARGER_RX; //ID from incoming charger messages

//BMS variables :)
extern int16_t current_current;
extern int8_t minimum_cell_temp;
extern int8_t max_cell_temp;
extern uint8_t state_of_charge;
extern uint8_t BMS_voltage;
extern uint8_t BMS_discrete_outputs;
extern uint32_t BMS_internal_state;
extern uint32_t BMS_errors1;
extern uint32_t BMS_errors2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */
//the following line configures the CAN Handle typedef
  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */
	uint8_t error_flag = false;
	uint32_t msg_id = 0;
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &chargerRxHeader, canRxData);

	if(chargerRxHeader.IDE == CAN_ID_STD){ //determines the ID of the message
		msg_id = chargerRxHeader.StdId;
	} else {
		msg_id = chargerRxHeader.ExtId;
	}

	if(msg_id == BMS_ID1){
		BMS_discrete_outputs = canRxData[0];
		current_current = ((int16_t)canRxData[1]<<8) + canRxData[2]; //swapping from little endian to big endian
		minimum_cell_temp = canRxData[3];
		max_cell_temp = canRxData[4];
		state_of_charge = canRxData[5];
		BMS_voltage = ((uint16_t)canRxData[6]<<8) + canRxData[7];



	}else if(msg_id == BMS_ID2){

	}else if(msg_id == BMS_ID3){

	}else if(msg_id != CHARGER_RX){
		//something's messed up, handle error here:


		return;
	}


	uint16_t voltage = ((uint16_t)canRxData[0] << 8) + canRxData[1];
	uint16_t current = ((uint16_t)canRxData[2] << 8) + canRxData[3];
	uint16_t charger_status = canRxData[4];

	// TODO: Once the display works, we should display these errors
	if(voltage > MAX_VOLTAGE)
	{
		error_flag = true;
	}

	if(current > MAX_CURRENT)
	{
		error_flag = true;
	}

	// Charger_status will be zero if there are no errors
	if(charger_status != 0)
	{
		error_flag = true;
	}

	if(error_flag)
	{
		// Stop charging
		CHARGER_OUTPUT_STATUS = 1;
		update_charger(MAX_VOLTAGE, MAX_CURRENT, CHARGER_OUTPUT_STATUS);
	}
  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	// send periodic messages to charger nice
	update_charger(MAX_VOLTAGE, MAX_CURRENT, CHARGER_OUTPUT_STATUS);
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  //update the charger with the necessary information :)


  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

	// UART message should be something like
	// XXX,YYY where XXX is the max voltage and YYY is the max current. Should be 7 bytes of data
  /*
   * Time for me to commit a sin. We only know the current we can draw from an outlet, we do not know the corresponding output
   * We shall therefore do a conversion involving the power of the charger,
   * based on the max voltage, and our assumptions of the voltage and efficiency of the charger
   *
   *AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
   *
   *
   *
   *
   *
   *
   *
   *
   * */
	HAL_UART_Receive_IT(&huart1, uartRxData, 7);
	char *flagString;
	char *currentString;
	const char delimiter[2] = ",";

	flagString = strtok((char *)uartRxData, delimiter);
	currentString = strtok((char *)uartRxData, delimiter);
	 //__builtin_bswap32();
	char *ptr;
	// Set GLOBAL values to the received values
	MAX_VOLTAGE = (uint16_t)strtol(flagString, &ptr, 10);
	MAX_CURRENT = (uint16_t)strtol(currentString, &ptr, 10);

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

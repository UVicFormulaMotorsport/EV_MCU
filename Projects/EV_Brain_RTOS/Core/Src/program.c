// Includes
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "program.h"
#include "tasks/driving_loop_task.h"
#include "tasks/ready_to_drive_task.h"

// FreeRTOS Includes
#include "FreeRTOS.h"
#include "FreeRTOS/Source/include/portable.h"
#include "FreeRTOS/Source/include/task.h"
#include "FreeRTOS/Source/include/queue.h"
#include "FreeRTOS/Source/include/list.h"
#include "FreeRTOS/Source/include/timers.h"

// HANDLES
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

// CANBUS VARS
#define COBID 0x20
CAN_RxHeaderTypeDef pRxHeader; //CAN Rx Header
uint8_t tData[8] = { 0 }; // can message data
uint32_t pTxMailbox; // Message mailbox
CAN_FilterTypeDef sFilterConfig; //CAN filter configuration
uint32_t MC_RX = 0x181; // CAN ID LIST
uint32_t MC_TX = 0x101;
uint32_t BMS_RX1 = 0x180 + COBID;
uint32_t BMS_RX2 = 0x280 + COBID;
uint32_t BMS_RX3 = 0x380 + COBID;

// STATE VARS
uint8_t IMPLAUSIBLE_STATE;
uint8_t PEAK_TORQUE_MODE;
uint8_t READY_TO_DRIVE;

// Task handles
TaskHandle_t *ready_to_drive_t;
TaskHandle_t *driving_loop_t;


// Task Priorities
#define TASK_PRIORTY_HIGH 100
#define TASK_PRIORTY_LOW 1

// FUNCTION PROTOTYPES
void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *pTxHeader, uint32_t dlc, uint32_t ide, uint32_t rtr, uint32_t stdId)
{
	pTxHeader->DLC = dlc; // 'dlc' bytes of data
	pTxHeader->IDE = ide;
	pTxHeader->RTR = rtr;
	pTxHeader->StdId = stdId; //set standard identifier.
}

/**
 * @brief CAN Filter Initialization Function
 * @param hcan          : pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
 *        sFilterConfig : pointer to a CAN_FilterTypeDef structure.
 * 	   fifo          : Specifies the FIFO (0 or 1) which will be assigned to the filter. This parameter can be a value of CAN_filter_FIFO
 * 	   highID        : Specifies the filter identification number (MSBs for a 32-bit configuration, first one for a 16-bit configuration). This parameter can be a value between 0x0000 and 0xFFFF
 * 	   lowID         : Specifies the filter identification number (LSBs for a 32-bit configuration, second one for a 16-bit configuration). This parameter can be a value between 0x0000 and 0xFFFF
 * 	   highMask      : Specifies the filter mask number or identification number, according to the mode (MSBs for a 32-bit configuration, first one for a 16-bit configuration). This parameter can be a value between 0x0000 and 0xFFFF
 * 	   lowMask       : Specifies the filter mask number or identification number, according to the mode (LSBs for a 32-bit configuration, second one for a 16-bit configuration). This parameter can be a value between 0x0000 and 0xFFFF
 * 	   scale         : Specifies the filter scale. This parameter can be a value of CAN_filter_scale
 * @retval None
 */
void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint32_t fifo, uint32_t highId,
		uint32_t lowId, uint32_t highMask, uint32_t lowMask, uint32_t scale)
{
	sFilterConfig->FilterFIFOAssignment = fifo;
	sFilterConfig->FilterIdHigh = highId << 5; //must be shifted 5 bits to the left according to reference manual
	sFilterConfig->FilterIdLow = lowId;
	sFilterConfig->FilterMaskIdHigh = highMask;
	sFilterConfig->FilterMaskIdLow = lowMask;
	sFilterConfig->FilterScale = scale;
	sFilterConfig->FilterActivation = CAN_FILTER_ENABLE; //enable activation

	HAL_CAN_ConfigFilter(hcan, sFilterConfig); //config CAN filter
}

HAL_StatusTypeDef CANsend(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pTxHeader, uint32_t data, uint32_t *pTxMailbox)
{
	HAL_StatusTypeDef transmit_result;
	uint8_t tData[8] =
	{ 0 };
	uint8_t num_bytes = pTxHeader->DLC;

	for (int i = 0; i < num_bytes; i++)
	{
		tData[i] = data >> (8 * i);
	}

	transmit_result = HAL_CAN_AddTxMessage(hcan, pTxHeader, tData, pTxMailbox);
	HAL_Delay(10);

	return transmit_result;
}

void program()
{
	//Initialize CAN filter - filter ID = TxHeader Id of other device, 32 bit scale. Enables and configs filter.
	CAN_Filter_Init(&hcan1, &sFilterConfig, CAN_FILTER_FIFO0, MC_RX, 0, 0, 0,
	CAN_FILTERSCALE_32BIT);

	//start CAN
	HAL_CAN_Start(&hcan1);

	//interrupt on message pending
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	// Start ADC
	HAL_ADC_Start(&hadc1);

	// Set states
	READY_TO_DRIVE = FALSE;
	IMPLAUSIBLE_STATE = FALSE;
	PEAK_TORQUE_MODE = FALSE;

	//
	// FreeRTOS Setup
	//

	// Create tasks to be scheduled
	xTaskCreate(// The function that implements the task.
				driving_loop_task,
				// Text name for the task, just to help debugging.
				"Driving Loop Task",
				// The size (in words) of the stack that should be created
				// for the task.
				configMINIMAL_STACK_SIZE,
				/* A parameter that can be passed into the task.  Not used
				 in this simple demo. */
				NULL,
				// The priority to assign to the task.  tskIDLE_PRIORITY
				// (which is 0) is the lowest priority.  configMAX_PRIORITIES - 1
				// is the highest priority.
				TASK_PRIORTY_HIGH,
				// Used to obtain a handle to the created task.  Not used in
				// this simple demo, so set to NULL.
				driving_loop_t);

	xTaskCreate(// The function that implements the task.
				driving_loop_task, "Ready to Drive Loop Task",
				// The size (in words) of the stack that should be created
				// for the task.
				configMINIMAL_STACK_SIZE,
				// Task arguments
				NULL,
				// The priority to assign to the task.  tskIDLE_PRIORITY
				// (which is 0) is the lowest priority.  configMAX_PRIORITIES - 1
				// is the highest priority.
				TASK_PRIORTY_HIGH,
				// Used to obtain a handle to the created task.  Not used in
				// this simple demo, so set to NULL.
				ready_to_drive_t);

	// Suspend all tasks that have been added
	vTaskSuspendAll();

	// Manually set the tasks that we want to run initially
	// TODO: Once the task is implemented to check if all devices are communicating,
	// it would be run first instead of to ready to drive task
	vTaskResume(ready_to_drive_t);

	// Start the task scheduler
	// The task scheduler was will tasks that are ready to run based on their priorities
	vTaskStartScheduler();
}

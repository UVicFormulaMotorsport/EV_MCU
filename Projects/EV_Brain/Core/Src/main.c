/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <stdio.h>
//#include "demanded_torque_calculator_grt_rtw/demanded_torque_calculator.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define false 0
#define true !(false)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
// ADC1 looks at both APPS
// ADC2 looks at both BPS
// ADC3 looks at current, flow, and temp sensor


CAN_RxHeaderTypeDef pRxHeader; //CAN Rx Header
CAN_TxHeaderTypeDef setpointTxHeader; //CAN Tx Header
CAN_TxHeaderTypeDef aPedPTxHeader; // Accelerator Pedal % Tx Header
CAN_TxHeaderTypeDef aPedRTxHeader; // Accelerator Pedal Raw Tx Header
CAN_TxHeaderTypeDef bPedPTxHeader; // Brake Pedal % Tx Header
CAN_TxHeaderTypeDef bPedRTxHeader; // Brake Pedal Raw Tx Header


CAN_TxHeaderTypeDef flagsTxHeader; // Implausible state Tx header

uint32_t pTxMailbox;
CAN_FilterTypeDef sFilterConfig; //CAN filter configuration
uint32_t MC_RX = 0x181;
uint32_t MC_TX = 0x101;

// Constants for the pedal map
const uint8_t REGEN = 0;
const float GAMMA = 1;
const uint32_t A_PEDAL_RAW_MAX = 4095;
const uint32_t B_PEDAL_RAW_MAX = 4095;

float dmd_trq; // Demanded torque
float mtr_spd = 0; // motor speed
float max_cnt_pwr = 100; // max continuous power
float output_percent; // percentage of current max torque to output

uint8_t tData[8] = { 0 }; // can message data
uint16_t StatusFlags;
uint16_t trq_setpoint; // Torque setpoint
uint8_t IMPLAUSIBLE_STATE;
uint8_t PEAK_TORQUE_MODE;
uint8_t READY_TO_DRIVE;

const uint8_t MAX_CONT_TORQUE = 125;
const uint8_t MAX_PEAK_TORQUE = 240;
const
char msg[100];

uint16_t transmit_result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *pTxHeader, uint32_t dlc, uint32_t ide, uint32_t rtr, uint32_t stdId);
static void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint32_t fifo, uint32_t highId,
		uint32_t lowId, uint32_t highMask, uint32_t lowMask, uint32_t scale);
HAL_StatusTypeDef CANsend(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pTxHeader, uint32_t data, uint32_t *pTxMailbox);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Returns the maximum continuous torque available at the current rpm
float cont_torque_map(uint16_t rpm)
{
	// ax*2 + bx + c
	float val = (-0.000003 * rpm * rpm) + (0.0192 * rpm) + (97.429);
	if (val < 0)
		return 0;
	if (val > MAX_CONT_TORQUE)
		return MAX_CONT_TORQUE;
	return val;
}

// Returns the maximum peak torque available at the current rpm
float peak_torque_map(uint16_t rpm)
{
	// ax*2 + bx + c
	float val = (-0.000001 * rpm * rpm) + (0.0029 * rpm) + (239.57);
	if (val < 0)
		return 0;
	if (val > MAX_PEAK_TORQUE)
		return MAX_PEAK_TORQUE;
	return val;
}

// Calculate a setpoint value given a torque value
uint16_t calculate_setpoint(float dmd_trq)
{
	float output_percent = dmd_trq / 240;
	return output_percent * (float) 32767;
}

// Returns true if the car is not in or recovering from an implausible state
int plausibility_check(float ap_percent, float bp_percent, uint8_t *IMPLAUSIBLE_STATE)
{
	// If in an implausible state, check if the accelerator pedal
	// has dropped below 5 percent, in this case, update state and return true
	// otherwise return false

	// TODO: Check for short or open circuit
	if (*IMPLAUSIBLE_STATE == true)
	{
		if (ap_percent < 5)
		{
			*IMPLAUSIBLE_STATE = false;
			StatusFlags = 0;
			transmit_result = CANsend(&hcan1, &flagsTxHeader, StatusFlags, &pTxMailbox);
			HAL_Delay(10);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		// Check ap and bp to see if they are currently in an implausible state.
		// Change the status and return false if they are
		// TODO: Figure out the bp_percent value for mechanical brake actuation
		if (ap_percent >= 25 && bp_percent >= 90)
		{
			*IMPLAUSIBLE_STATE = true;
			StatusFlags = 0xFFFF;
			transmit_result = CANsend(&hcan1, &aPedPTxHeader, StatusFlags, &pTxMailbox);
			return false;
		}
		else
		{
			return true;
		}
	}
}

// Calculate the demanded torque based on the pedal sensor actuation
// Based on equation from section 3.3.3 of the paper linked below
// https://www.researchgate.net/publication/323522409_Design_and_realization_of_a_One-Pedal-Driving_algorithm_for_the_TUe_Lupo_EL
float pedal_map(float throttle, float max_trq)
{
	float val = ((throttle - REGEN) / (max_trq - REGEN));
	float trq_dmd = pow(val, GAMMA);
	return trq_dmd;
}

HAL_StatusTypeDef CANsend(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pTxHeader, uint32_t data, uint32_t *pTxMailbox)
{
	HAL_StatusTypeDef transmit_result;
	uint8_t tData[8] = { 0 };
	uint8_t num_bytes = pTxHeader->DLC;

	for (int i = 0; i < num_bytes; i++)
	{
		tData[i] = data >> (8 * i);
	}

	transmit_result = HAL_CAN_AddTxMessage(hcan, pTxHeader, tData, pTxMailbox);
	HAL_Delay(10);

	return transmit_result;
}

void driving_loop()
{
	float ap_raw = 0; // raw accelerator pedal sensor value
	float bp_raw = 0; // raw brake pedal sensor value
	float ap_percent = 0; // accelerator pedal value as a percent
	float bp_percent = 0; // brake pedal value as a percent
	int max_trq = 0;

	// Get ADC values
	HAL_ADC_Start(&hadc1);
	ap_raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	bp_raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	printf("AP_RAW - %f, AP_PERCENT - %f\n\r", ap_raw, ap_percent);

	// Convert [sensor value] -> [% activation]
	ap_percent = (ap_raw / A_PEDAL_RAW_MAX) * 100;
	bp_percent = (bp_raw / B_PEDAL_RAW_MAX) * 100;

	uint32_t ap_percent_i = (uint32_t) ap_percent;
	uint32_t bp_percent_i = (uint32_t) bp_percent;
	uint32_t ap_raw_i = (uint32_t) ap_raw;
	uint32_t bp_raw_i = (uint32_t) bp_raw;

	// Send pedal position can messages
	transmit_result = CANsend(&hcan1, &aPedPTxHeader, ap_percent_i, &pTxMailbox); // A pedal percent
	transmit_result = CANsend(&hcan1, &aPedRTxHeader, ap_raw_i, &pTxMailbox); // A pedal raw
	transmit_result = CANsend(&hcan1, &bPedPTxHeader, bp_percent_i, &pTxMailbox); // B pedal percent
	transmit_result = CANsend(&hcan1, &bPedRTxHeader, bp_raw_i, &pTxMailbox); // B pedal raw

	//Check pedal plausibility
	// If in a plausible state, convert pedal sensor actuation to desired torque
	if (plausibility_check(ap_percent, bp_percent, &IMPLAUSIBLE_STATE))
	{

		// Get current max torque
		// TODO: Read motor rpm
		/*
		 if (PEAK_TORQUE_MODE)
		 {
		 max_trq = peak_torque_map(current_rpm);
		 }
		 else
		 {
		 max_trq = cont_torque_map(current_rpm);
		 }
		 */

		max_trq = 100;
		dmd_trq = pedal_map(ap_percent, max_trq);
	}
	else
	{
		// If not in plausible state, set demanded torque to 0
		// TODO: May need to trigger shutdown circuit here, not sure.
		dmd_trq = 0;
	}

	// Calculate the torque setpoint
	trq_setpoint = calculate_setpoint(dmd_trq);

	// Send CAN message
	// Split the setpoint value into 2 byte blocks
	uint32_t trq_setpoint_mod = (trq_setpoint << 8) | 0x90;
	transmit_result = CANsend(&hcan1, &setpointTxHeader, trq_setpoint_mod, &pTxMailbox);

	// Convert to string and print to serial
	sprintf(msg, "AP_RAW - %lu, AP_PERCENT - %lu, BP_RAW - %lu, BP_PERCENT - %lu, SP - %i\r\n", ap_raw_i, ap_percent_i,
			bp_raw_i, bp_percent_i, trq_setpoint);
	transmit_result = HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);

	HAL_Delay(250);

	return;
}

void ready_to_drive_loop()
{
	// TODO: Figure out what signals we need to check for
	// For now it will just be a button press

	// Check if button is pressed
	if((GPIOA->IDR & GPIO_PIN_0) == true)
	{
		// Send the RTD signal
		GPIOB->ODR |= GPIO_PIN_1;

		// Wait for response
		while(((GPIOB->IDR & GPIO_PIN_0) == false));

		// Response received when sound is finished playing
		// Set signal back to low
		GPIOB->ODR &= ~GPIO_PIN_1;

		READY_TO_DRIVE = true;
	}
	return;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

	//Initialize CAN headers - standard id type, set standard Id = filter ID of other device
	CAN_TxHeader_Init(&setpointTxHeader, 3, CAN_ID_STD, CAN_RTR_DATA, MC_TX);
	CAN_TxHeader_Init(&aPedPTxHeader, 4, CAN_ID_STD, CAN_RTR_DATA, 0x301);
	CAN_TxHeader_Init(&aPedRTxHeader, 4, CAN_ID_STD, CAN_RTR_DATA, 0x302);
	CAN_TxHeader_Init(&bPedPTxHeader, 4, CAN_ID_STD, CAN_RTR_DATA, 0x303);
	CAN_TxHeader_Init(&bPedRTxHeader, 4, CAN_ID_STD, CAN_RTR_DATA, 0x304);
	CAN_TxHeader_Init(&flagsTxHeader, 2, CAN_ID_STD, CAN_RTR_DATA, 0x201);

	//Initialize CAN filter - filter ID = TxHeader Id of other device, 32 bit scale. Enables and configs filter.
	CAN_Filter_Init(&hcan1, &sFilterConfig, CAN_FILTER_FIFO0, MC_RX, 0, 0, 0, CAN_FILTERSCALE_32BIT);

	//start CAN
	transmit_result = HAL_CAN_Start(&hcan1);

	//interrupt on message pending
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	// Start ADC
	HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// Set states
	READY_TO_DRIVE = false;
	IMPLAUSIBLE_STATE = false;
	PEAK_TORQUE_MODE = false;

	while (1)
	{
		if(READY_TO_DRIVE == true)
		{
			driving_loop();
		}
		else
		{
			ready_to_drive_loop();
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *pTxHeader, uint32_t dlc, uint32_t ide, uint32_t rtr, uint32_t stdId)
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
static void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint32_t fifo, uint32_t highId,
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

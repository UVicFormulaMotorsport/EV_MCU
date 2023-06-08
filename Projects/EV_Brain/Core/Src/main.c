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

// sensor value vars
unsigned short ap_raw_buff[2] = {0, 0};     // raw accelerator pedal ADC1 values buffer for DMA
unsigned short ap_raw[2] = {0, 0};          // raw accelerator pedal ADC1 values
unsigned char ap_percent = 0;               // accelerator pedal value as a percent

unsigned short bp_raw_buff[2] = {0, 0};     // raw brake pedal ADC2 values buffer for DMA
unsigned short bp_raw[2] = {0, 0};          // raw brake pedal ADC2 values
unsigned char bp_percent = 0;               // brake pedal value as a percent

unsigned short cft_raw_buff[3] = {0, 0, 0}; // raw ADC3 values buffer for DMA
unsigned short cft_raw[3] = {0, 0, 0};      // raw ADC3 values
// order: {current, flow, temp}
unsigned short temp_sensor_value = 0;
unsigned short flow_sensor_value = 0;
unsigned short current_sensor_value = 0;

unsigned char max_torque_available = 0; // stores current max torque available from the motor based on motor rpm
unsigned short torque_setpoint = 0;     // torque setpoint to send to motor

// CAN Device IDs
unsigned short MotorControllerAccepting = 0x201; // send to this device ID if you're giving the MC a value
unsigned short MotorControllerSending = 0x181; // you will recieve from this ID if you requested a MC value
unsigned short PowerDistUnit = 0x710;

// MC register IDs
unsigned short MC_desiredTorque_register = 0x90;
unsigned short MC_ActualRPM_register = 0x30;

// CAN header templates
CAN_TxHeaderTypeDef sendMsg_CAN_Header;
// sendMsg_CAN_Header.StdId = ; // device ID (hex)
// sendMsg_CAN_Header.ExtId = ; // not using
sendMsg_CAN_Header.IDE = CAN_ID_STD;
sendMsg_CAN_Header.RTR = CAN_RTR_DATA;
sendMsg_CAN_Header.DLC = 3;
sendMsg_CAN_Header.TransmitGlobalTime = DISABLE;

// TODO: figure out how to request data from the MC with this header
CAN_RxHeaderTypeDef askData_CAN_Header;
//askData_CAN_Header.StdId = ; // device ID (hex)
//askData_CAN_Header.ExtId = ; // not using
askData_CAN_Header.IDE = CAN_ID_STD;
askData_CAN_Header.RTR = CAN_RTR_DATA;
askData_CAN_Header.DLC = 3;
//askData_CAN_Header.Timestamp = ; // what do these two need to be?
//askData_CAN_Header.FilterMatchIndex = ;

unsigned long pTxMailbox = 0; // CAN mailbox var

// TODO: what is this for?
// recieving CAL stuff
CAN_FilterTypeDef sFilterConfig; //CAN filter configuration

// TODO: figure out how the pedal map works and what it's for
// Constants for the pedal map
const unsigned char REGEN = 0;
const float GAMMA = 1;

// max and min potentiometer values
static const unsigned int AP1_RAW_MAX_ADC_VAL = 0b111111111111;
static const unsigned int BP1_RAW_MAX_ADC_VAL = 0b111111111111;
static const unsigned int AP1_RAW_MIN_ADC_VAL = 0b000000000000;
static const unsigned int BP1_RAW_MIN_ADC_VAL = 0b000000000000;
unsigned int AP1_RAW_ADC_RANGE = AP1_RAW_MAX_ADC_VAL - AP1_RAW_MIN_ADC_VAL;
unsigned int BP1_RAW_ADC_RANGE = BP1_RAW_MAX_ADC_VAL - BP1_RAW_MIN_ADC_VAL;

// TODO: what is this for?
unsigned char PEAK_TORQUE_MODE = false;

unsigned char READY_TO_DRIVE = false;

// TODO: are these values correct? what are they for?
const unsigned char MAX_CONT_TORQUE = 125;
const unsigned char MAX_PEAK_TORQUE = 240;

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

// TODO: does this need to be a function, or can the steps just be done in main?
static void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, unsigned int fifo, unsigned int highId,
		unsigned int lowId, unsigned int highMask, unsigned int lowMask, unsigned int scale);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

// destRegister is the ID (hex) of the register to send the data to, defined as a var above
// data is just a value, not an array - this function formats it for you
unsigned char sendCANmsg_MC(unsigned short destRegister, unsigned int data)
{
  sendMsg_CAN_Header.StdId = MotorControllerAccepting;
  sendMsg_CAN_Header.DLC = 3;

  unsigned char formattedData[3] = {destRegister, 0xFF | data, (0xFF00 | data) >> 8};

  if (HAL_CAN_AddTxMessage(&hcan1, &sendMsg_CAN_Header, formattedData, &pTxMailbox))
  {
    return 0;
  }

  return 1;
}

// TODO: above in global vars, define vars for each of the channels (ex. unsigned char FAN1 = 0xB)
// if highAmps == 0, PDU chooses 5A instead of 20A
// if enable == 0, PDU turns the circuit off
// channel should be one of the vars defined above
unsigned char sendCANmsg_PDU(unsigned char enable, unsigned char highAmps, unsigned char channel)
{
  sendMsg_CAN_Header.StdId = PowerDistUnit;
  sendMsg_CAN_Header.DLC = 1;

  unsigned char formattedData[1] = {0b00000000};

  if (enable) formattedData[0] = formattedData[0] | 0b00010000;
  if (highAmps) formattedData[0] = formattedData[0] | 0b00100000;
  formattedData[0] = formattedData[0] | (channel & 0b1111);

  if (HAL_CAN_AddTxMessage(&hcan1, &sendMsg_CAN_Header, formattedData, &pTxMailbox))
  {
    return 0;
  }

  return 1;
}

// TODO: implement dash function: define device IDs, figure out how to format data
unsigned char sendCANmsg_DASH(unsigned char destination, unsigned char data)
{
  sendMsg_CAN_Header.StdId = destination;
  sendMsg_CAN_Header.DLC = 1;

  unsigned char formattedData[1] = {0b00000000};

  // format data

  if (HAL_CAN_AddTxMessage(&hcan1, &sendMsg_CAN_Header, formattedData, &pTxMailbox))
  {
    return 0;
  }

  return 1;
}

// TODO: which of this and the next function should we use to calculate torque?
// Returns the maximum continuous torque available at the current rpm
float max_cont_torque_available(unsigned short rpm)
{
	float t = (-0.000003 * rpm * rpm) + (0.0192 * rpm) + (97.429);
	if (t < 0)
		return 0;
	if (t > MAX_CONT_TORQUE)
		return MAX_CONT_TORQUE;
	return t;
}

// Returns the maximum peak torque available at the current rpm
float peak_torque_available(unsigned short rpm)
{
	float t = (-0.000001 * rpm * rpm) + (0.0029 * rpm) + (239.57);
	if (t < 0)
		return 0;
	if (t > MAX_PEAK_TORQUE)
		return MAX_PEAK_TORQUE;
	return t;
}

// TODO: when is the system plausible?
// Returns true if the car is not in or recovering from an implausible state
unsigned char plausibility_check(unsigned char ap_percent, unsigned char bp_percent)
{
	// check if the system is in an implausible state
  return true;
}

// TODO: what is this for?
// Calculate the demanded torque based on the pedal sensor actuation
// Based on equation from section 3.3.3 of the paper linked below
// https://www.researchgate.net/publication/323522409_Design_and_realization_of_a_One-Pedal-Driving_algorithm_for_the_TUe_Lupo_EL
float pedal_map(float throttle, float max_trq)
{
  return pow((throttle - REGEN) / (max_trq - REGEN), GAMMA);
}

void driving_loop()
{
  // Convert [sensor value] -> [% activation]
  ap_percent = (unsigned char)((((double)(ap_raw[0] - AP1_RAW_MIN_ADC_VAL)) / AP1_RAW_ADC_RANGE) * 100);
  bp_percent = (unsigned char)((((double)(bp_raw[0] - BP1_RAW_MIN_ADC_VAL)) / BP1_RAW_ADC_RANGE) * 100);

  // TODO: check accelerator values against eachother.

  // TODO: Send pedal position can messages to LCD

  // If in a plausible state, convert pedal sensor actuation to desired torque
  if (plausibility_check(ap_percent, bp_percent))
  {
    // TODO: get motor current RPM
    // TODO: calculate max torque value here, based on motor RPM
    max_torque_available = 100;
    torque_setpoint = (pedal_map(ap_percent, max_torque_available) / 240) * 32767;
  }else{
    // if not in plausible state, set demanded torque to 0
    // ! may need to trigger shutdown circuit here
    torque_setpoint = 0;
  }

  sendCANmsg_MC(MC_desiredTorque_register, torque_setpoint); // send torque CAN message

  return;
}

void waiting_to_drive()
{
	// TODO: Figure out what signals we need to check for
	// for now it will just be a button press

	// Check if the button is pressed
	if((GPIOA->IDR & GPIO_PIN_0) == true)
	{
		// TODO: pdu message to play sound

    // TODO: pdu message to close hw enable switch on controller

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

	//Initialize CAN filter - filter ID = TxHeader Id of other device, 32 bit scale. Enables and configs filter.
	CAN_Filter_Init(&hcan1, &sFilterConfig, CAN_FILTER_FIFO0, MotorControllerAccepting, 0, 0, 0, CAN_FILTERSCALE_32BIT);

	//start CAN
	HAL_CAN_Start(&hcan1);

	//interrupt on message pending
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // start ADC1 and DMA channel, looking at both accelerator sensors
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, ap_raw_buff, 2);

  // start ADC2 and DMA channel, looking at both brake sensors
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start_DMA(&hadc2, bp_raw_buff, 2);

  // start ADC3 and DMA channel, looking at current, flow, and temp sensors
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start_DMA(&hadc3, cft_raw_buff, 3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
		if(READY_TO_DRIVE == true)
		{
		  driving_loop();
		}
		else
		{
			waiting_to_drive();
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
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
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
static void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, unsigned int fifo, unsigned int highId,
		unsigned int lowId, unsigned int highMask, unsigned int lowMask, unsigned int scale)
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

// Called when ADC1 DMA buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1)
{
  ap_raw[0] = ap_raw_buff[0];
  ap_raw[1] = ap_raw_buff[1];
}

// Called when ADC2 DMA buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc2)
{
  bp_raw[0] = bp_raw_buff[0];
  bp_raw[1] = bp_raw_buff[1];
}

// Called when ADC3 DMA buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc3)
{
  temp_sensor_value = cft_raw_buff[0];
  flow_sensor_value = cft_raw_buff[1];
  current_sensor_value = cft_raw_buff[2];
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
void assert_failed(unsigned char *file, unsigned int line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

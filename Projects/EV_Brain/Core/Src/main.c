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
volatile unsigned int ADC1_outputBuffer[7] = {0, 0, 0, 0, 0, 0, 0}; // raw ADC1 values buffer for DMA
volatile unsigned int acceleratorPedal_ADCoutput[2] = {0, 0};       // raw accelerator pedal ADC1 values
volatile unsigned int brakePedal_ADCoutput[2] = {0, 0};             // raw brake pedal ADC1 values
volatile unsigned int tfc_ADCoutput[3] = {0, 0, 0};                 // raw TFC ADC1 values
unsigned char  brakePedal_percent = 0;                         // brake pedal value as a percent

unsigned char  brakePedal_percent = 0;       // brake pedal value as a percent
unsigned char  acceleratorPedal_percent = 0; // accelerator pedal value as a percent
unsigned short temp_sensor_DegreesC = 0;     // degrees C
unsigned short flow_sensor_LitresPerMin = 0; // L/min
unsigned short current_sensor_Amperes = 0;   // amps

unsigned char  max_torque_available = 0; // stores current max torque available from the motor based on motor rpm
unsigned short torque_setpoint = 0;      // torque setpoint to send to motor

unsigned char BMS_discreete_inputs_1 = 0; //bitfield for high level BMS information
//int BMS_current_measured = 0; //Not currently being used, we have our own current sensor
char min_cell_temp = 0;
char max_cell_temp = 0;
unsigned char state_of_charge = 0;
unsigned short battery_voltage = 0; //0.1V per byte, I.E. receiving 0x12C0 = 4800 -> divite by 10 = 480V

unsigned long BMS_internal_state = 0;
unsigned long BMS_error_register_1 = 0;

unsigned long BMS_error_register_2 = 0;
unsigned short BMS_discrete_inputs_2 = 0;


// CAN Device IDs
unsigned short MotorControllerCAN_acceptingID = 0x201; // send to this device ID if you're giving the MC a value
unsigned short MotorControllerCAN_sendingID = 0x181;   // you will recieve from this ID if you requested a MC value
unsigned short PowerDistUnitCAN_acceptingID = 0x710;
unsigned short BMS_sendingID_1 = 0x1A0;
/* Data from BMS_sendingID_1 includes:
 * some high level status in a bitfield
 * Minimum and Maximum cell temperature
 * Current current, current voltage, state of charge
 */
unsigned short BMS_sendingID_2 = 0x2A0;
/* Data from BMS_sendingID_2 includes:
 * BMS Internal state in a uint32 bitfield
 * Errors register part 1 in a uint32 bitfield
 */
unsigned short BMS_sendingID_3 = 0x3A0;
/* Data from BMS_sendingId_3 includes:
 * Errors register part 2, a uint32 bitfield
 * Additional discrete inputs
 */

// MC register IDs
unsigned short MotorControllerCAN_desiredTorque_register = 0x90;
unsigned short MotorControllerCAN_actualRPM_register = 0x30;

volatile unsigned char conversion_done = 0;

// CAN header templates
CAN_TxHeaderTypeDef sendCANheaderTemplate = {0x201, 0, CAN_ID_STD, CAN_RTR_DATA, 3, 0};

// TODO: figure out how to request data from the MC with this header
//CAN_RxHeaderTypeDef requestCANheaderTemplate = {0x181, 0, CAN_ID_STD, CAN_RTR_DATA, 3, 0, 0};

CAN_RxHeaderTypeDef pRxHeader; //This is where the header for received can headers is stored, so it can be processed
unsigned char rData[8]; // where the received CAN messages go

unsigned long pTxMailbox = 0; // CAN mailbox var

// TODO: what is this for?
// recieving CAL stuff
CAN_FilterTypeDef sFilterConfig; //CAN filter configuration

// TODO: figure out how the pedal map works and what it's for
// Constants for the pedal map
const unsigned char REGEN = 0;
const float GAMMA = 1;

// max and min potentiometer values
static const unsigned int AcceleratorPedalSensor1_MAX_ADC_VALUE = 0b111111111111;
static const unsigned int BrakePedalSensor1_MAX_ADC_VALUE = 0b111111111111;
static const unsigned int AcceleratorPedalSensor1_MIN_ADC_VALUE = 0b000000000000;
static const unsigned int BrakePedalSensor1_MIN_ADC_VALUE = 0b000000000000;
unsigned int AcceleratorPedalSensor1_ADC_RANGE = AcceleratorPedalSensor1_MAX_ADC_VALUE - AcceleratorPedalSensor1_MIN_ADC_VALUE;
unsigned int BrakePedalSensor1_ADC_RANGE = BrakePedalSensor1_MAX_ADC_VALUE - BrakePedalSensor1_MIN_ADC_VALUE;

// TODO: what is this for?
unsigned char PEAK_TORQUE_MODE = false;

unsigned char READY_TO_DRIVE = false;

// TODO: are these values correct? what are they for?
const unsigned char MAX_CONTINUOUS_TORQUE = 125;
const unsigned char MAX_PEAK_TORQUE = 240;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

// TODO: does this need to be a function, or can the steps just be done in main?
static void Initialize_CAN_Filter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, unsigned int fifo, unsigned int highId,
		unsigned int lowId, unsigned int highMask, unsigned int lowMask, unsigned int scale);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// destinationRegisterID is the ID (hex) of the register to send the data to, defined as a var above
// data is just a value, not an array - this function formats it for you
unsigned char sendCAN_MotorController(unsigned short destinationRegisterID, unsigned int data)
{
  sendCANheaderTemplate.StdId = MotorControllerCAN_acceptingID;
  sendCANheaderTemplate.DLC = 3;

  unsigned char formattedData[3] = {destinationRegisterID, 0xFF & data, (0xFF00 & data) >> 8};

  if (HAL_CAN_AddTxMessage(&hcan1, &sendCANheaderTemplate, formattedData, &pTxMailbox))
  {
    return 0;
  }

  return 1;
}

// TODO: above in global vars, define vars for each of the channels (ex. unsigned char FAN1 = 0xB)
// if highAmpsBool == 0, PDU chooses 5A instead of 20A
// if enableCircuitBool == 0, PDU turns the circuit off
// channelHex should be one of the vars defined above
unsigned char sendCANmsg_PDU(unsigned char enableCircuitBool, unsigned char highAmpsBool, unsigned char channelHex)
{
  sendCANheaderTemplate.StdId = PowerDistUnitCAN_acceptingID;
  sendCANheaderTemplate.DLC = 1;

  unsigned char formattedData[1] = {0b00000000}; // HAL_CAN_AddTxMessage takes an array for data

  if (enableCircuitBool) formattedData[0] = formattedData[0] | 0b00010000;   // set enableCircuitBool bit
  if (highAmpsBool) formattedData[0] = formattedData[0] | 0b00100000; // set highAmpsBool bit
  formattedData[0] = formattedData[0] | (channelHex & 0b1111);       // set channel bits

  if (HAL_CAN_AddTxMessage(&hcan1, &sendCANheaderTemplate, formattedData, &pTxMailbox) == HAL_OK)
  {
    return 0;
  }

  return 1;
}

// TODO: implement dash function: define device IDs, figure out how to format data
unsigned char sendCANmsg_DASH(unsigned char destinationID, unsigned char data)
{
  sendCANheaderTemplate.StdId = destinationID;
  sendCANheaderTemplate.DLC = 1;

  unsigned char formattedData[1] = {0b00000000}; // HAL_CAN_AddTxMessage takes an array for data

  // TODO: format data here

  if (HAL_CAN_AddTxMessage(&hcan1, &sendCANheaderTemplate, formattedData, &pTxMailbox) == HAL_OK)
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
	if (t > MAX_CONTINUOUS_TORQUE)
		return MAX_CONTINUOUS_TORQUE;
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
unsigned char plausibility_check(unsigned char acceleratorPedal_percent, unsigned char brakePedal_percent)
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

void driving_tasks()
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_outputBuffer, 7);
	while(!conversion_done)
	{
		// wait
	}
	conversion_done = 0;

  // Convert [sensor value] -> [% activation]
  acceleratorPedal_percent = (unsigned char)((((double)(acceleratorPedal_ADCoutput[0] - AcceleratorPedalSensor1_MIN_ADC_VALUE)) / AcceleratorPedalSensor1_ADC_RANGE) * 100);
  brakePedal_percent = (unsigned char)((((double)(brakePedal_ADCoutput[0] - BrakePedalSensor1_MIN_ADC_VALUE)) / BrakePedalSensor1_ADC_RANGE) * 100);

  // TODO: check accelerator values against eachother.

  // TODO: Send pedal position can messages to LCD

  // If in a plausible state, convert pedal sensor actuation to desired torque
  if (plausibility_check(acceleratorPedal_percent, brakePedal_percent))
  {
    // TODO: get motor current RPM
    // TODO: calculate max torque value here, based on motor RPM
    max_torque_available = 100;
    torque_setpoint = (pedal_map(acceleratorPedal_percent, max_torque_available) / 240) * 32767;
  }else{
    // if not in plausible state, set demanded torque to 0
    // ! may need to trigger shutdown circuit here
    torque_setpoint = 0;
  }

  // TEST CAN MESSAGES

  // for PCAN
  //sendCAN_MotorController(0x069, acceleratorPedal_percent);
  //sendCAN_MotorController(0x096, brakePedal_percent);

  // for motor rpm
  //sendCAN_MotorController(0x31, (acceleratorPedal_percent*3277)/100); // send motor desired RPM to MC
  //sendCAN_MotorController(0x31, 0xF0F0);

  unsigned char formattedData[3] = {(0xF00 & acceleratorPedal_ADCoutput[0]) >> 8, (0x0F0 & acceleratorPedal_ADCoutput[0]) >> 4, 0x00F & acceleratorPedal_ADCoutput[0]};
  sendCANheaderTemplate.StdId = MotorControllerCAN_acceptingID;
  sendCANheaderTemplate.DLC = 3;
  HAL_CAN_AddTxMessage(&hcan1, &sendCANheaderTemplate, formattedData, &pTxMailbox);

  // END TEST CAN MESSAGES

  // ACTUAL CAN MESSAGE
  //sendCAN_MotorController(MotorControllerCAN_desiredTorque_register, torque_setpoint); // send torque CAN message

  return;
}

void waiting_to_drive_tasks()
{
	// TODO: Figure out what signals we need to check for
	// for now it will just be a button press

	/*
	// Check if the button is pressed
	if((GPIOA->IDR & GPIO_PIN_0) == true)
	{
		// TODO: pdu message to play sound

		// TODO: pdu message to close hw enable switch on controller

		READY_TO_DRIVE = true;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); //orng
	}
	*/

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) //PB0, 35, has pull-up
	{
		READY_TO_DRIVE = true;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //grn
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
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //grn
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); //orng

	//Initialize CAN filter - filter ID = TxHeader Id of other device, 32 bit scale. Enables and configs filter.
	Initialize_CAN_Filter(&hcan1, &sFilterConfig, CAN_FILTER_FIFO0, MotorControllerCAN_acceptingID, 0, 0, 0, CAN_FILTERSCALE_32BIT);

	//start CAN
	HAL_CAN_Start(&hcan1);

	//interrupt on message pending
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // start ADC1 and DMA channel
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_outputBuffer, 7);
  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while(1)
	{
		if(READY_TO_DRIVE == true)
		{
		  driving_tasks();
		}
		else
		{
			waiting_to_drive_tasks();
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
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
static void Initialize_CAN_Filter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, unsigned int fifo, unsigned int highId,
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
  acceleratorPedal_ADCoutput[0] = ADC1_outputBuffer[0];
  acceleratorPedal_ADCoutput[1] = ADC1_outputBuffer[1];
  brakePedal_ADCoutput[0] = ADC1_outputBuffer[2];
  brakePedal_ADCoutput[1] = ADC1_outputBuffer[3];
  tfc_ADCoutput[0] = ADC1_outputBuffer[4];
  tfc_ADCoutput[1] = ADC1_outputBuffer[5];
  tfc_ADCoutput[2] = ADC1_outputBuffer[6];
  conversion_done = 1;
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

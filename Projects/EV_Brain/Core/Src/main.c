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
#include "usb_host.h"

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

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef pTxHeader; //CAN Tx Header
CAN_RxHeaderTypeDef pRxHeader; //CAN Rx Header

CAN_TxHeaderTypeDef aPedPTxHeader; // Accelerator Pedal % Tx Header
CAN_TxHeaderTypeDef aPedRTxHeader; // Accelerator Pedal R Tx Header
CAN_TxHeaderTypeDef bPedPTxHeader; // Brake Pedal % Tx Header
CAN_TxHeaderTypeDef bPedRTxHeader; // Brake Pedal R Tx Header

CAN_TxHeaderTypeDef flagsTxHeader; // Implausible state Tx header

uint32_t pTxMailbox;
CAN_FilterTypeDef sFilterConfig; //CAN filter configuration
uint32_t MC_RX = 0x181;
uint32_t MC_TX = 0x101;

// Constants for the pedal map
const uint8_t REGEN = 0;
const float GAMMA = 1;

float dmd_trq; // demanded torque
float mtr_spd = 0; // motor speed
float max_cnt_pwr = 100; // max continuous power
float output_percent; // percentage of current max torque to output

uint8_t tData[8] =
{ 0 }; // can message data
uint16_t StatusFlags;
uint16_t trq_setpoint; // torque setpoint
uint8_t IMPLAUSIBLE_STATE;
uint8_t PEAK_TORQUE_MODE;
uint8_t READY_TO_DRIVE;

const uint8_t MAX_CONT_TORQUE = 125;
const uint8_t MAX_PEAK_TORQUE = 240;

float max_trq;
char msg[100];

uint16_t transmit_result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
void MX_USB_HOST_Process(void);

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
	float val = ((throttle - REGEN) / (100 - REGEN));
	float trq_dmd = pow(val, GAMMA);
	return trq_dmd;
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

void driving_loop()
{
	float ap_raw = 0; // raw accelerator pedal sensor value
	float bp_raw = 0; // raw brake pedal sensor value
	float ap_percent = 0; // accelerator pedal value as a percent
	float bp_percent = 0; // brake pedal value as a percent

	// Get ADC values
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ap_raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(&hadc1);
	bp_raw = HAL_ADC_GetValue(&hadc1);

	// Convert [sensor value] -> [% throttle activated]
	ap_percent = (ap_raw / 4095.0) * 100;
	bp_percent = (bp_raw / 4095.0) * 100;

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

		// Set inputs for torque_calculator
		//.regen_percentage = regen_percent; // regen not yet implemented
		//demanded_torque_calculator_U.throttle = ap_percent * 100;
		//demanded_torque_calculator_U.max_torque = max_trq;

		//Step (calculate model outputs from inputs)
		//demanded_torque_calculator_step();

		// Get outputs from torque_calculator object
		//dmd_trq = demanded_torque_calculator_Y.demanded_torque;
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
	transmit_result = CANsend(&hcan1, &pTxHeader, trq_setpoint_mod, &pTxMailbox);

	// Convert to string and print to serial
	sprintf(msg, "AP_RAW - %lu, AP_PERCENT - %lu, BP_RAW - %lu, BP_PERCENT - %lu, SP - %i\r\n", ap_raw_i, ap_percent_i,
			bp_raw_i, bp_percent_i, trq_setpoint);
	transmit_result = HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);

	HAL_Delay(250);

	return;
}

void ready_to_drive_loop()
{
	// TODO: Write this

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
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_USB_HOST_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_CAN1_Init();
	/* USER CODE BEGIN 2 */

	//Initialize CAN headers - standard id type, set standard Id = filter ID of other device
	CAN_TxHeader_Init(&pTxHeader, 3, CAN_ID_STD, CAN_RTR_DATA, MC_TX);
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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// Initialize torque calculator object
	//demanded_torque_calculator_initialize();

	// Set states
	READY_TO_DRIVE = true;
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
		MX_USB_HOST_Process();

		/* USER CODE BEGIN 3 */
	}

	// Terminate torque calculator object
	//demanded_torque_calculator_terminate();

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
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
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_I2C_SPI_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PDM_OUT_Pin */
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : I2S3_WS_Pin */
	GPIO_InitStruct.Pin = I2S3_WS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CLK_IN_Pin */
	GPIO_InitStruct.Pin = CLK_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
	GPIO_InitStruct.Pin = I2S3_MCK_Pin | I2S3_SCK_Pin | I2S3_SD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef pTxHeader; //CAN Tx Header
CAN_RxHeaderTypeDef pRxHeader; //CAN Rx Header
uint32_t pTxMailbox;
uint8_t a,r; //transmit, receive byte through CAN
CAN_FilterTypeDef sFilterConfig; //CAN filter configuration

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *pTxHeader, uint32_t dlc,uint32_t ide,uint32_t rtr,uint32_t stdId);
static void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint32_t fifo,uint32_t highId,uint32_t lowId,uint32_t highMask,uint32_t lowMask,uint32_t scale);
HAL_StatusTypeDef CANsend(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pTxHeader, uint8_t data[], uint32_t *pTxMailbox);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  //Initialize CAN header - standard id type, set standard Id = filter ID of other device
    CAN_TxHeader_Init(&pTxHeader, 1,CAN_ID_STD, CAN_RTR_DATA, 0x244);

    //Initialize CAN filter - filter ID = TxHeader Id of other device, 32 bit scale. Enables and configs filter.
    CAN_Filter_Init(&hcan1, &sFilterConfig, CAN_FILTER_FIFO0, 0x245, 0, 0, 0, CAN_FILTERSCALE_32BIT);

    //start CAN
    HAL_CAN_Start(&hcan1);

    //interrupt on message pending
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for (int n=0;n<10000000;n++); //delay
	  a++; //increment 8 bit data
	  CANsend(&hcan1, &pTxHeader, &a, &pTxMailbox);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief CAN Tx Header Initialization Function
  * @param pTxHeader : CAN Tx header structure.
  * 	   dlc       : Specifies the length of the frame that will be transmitted.
  *        ide       : Specifies the type of identifier for the message that will be transmitted.
  *        rtr 	     : Specifies the type of frame for the message that will be transmitted.
  *        stdId     : Specifies the standard identifier. This parameter can be a value between 0x0000 and 0x07FF.
  * @retval None
  */
static void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *pTxHeader, uint32_t dlc,uint32_t ide,uint32_t rtr,uint32_t stdId)
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
static void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint32_t fifo,uint32_t highId,uint32_t lowId,uint32_t highMask,uint32_t lowMask,uint32_t scale)
{
	  sFilterConfig->FilterFIFOAssignment = fifo;
	  sFilterConfig->FilterIdHigh = highId<<5; //must be shifted 5 bits to the left according to reference manual
	  sFilterConfig->FilterIdLow = lowId;
	  sFilterConfig->FilterMaskIdHigh = highMask;
	  sFilterConfig->FilterMaskIdLow = lowMask;
	  sFilterConfig->FilterScale = scale;
	  sFilterConfig->FilterActivation = CAN_FILTER_ENABLE; //enable activation

	  HAL_CAN_ConfigFilter(hcan, sFilterConfig); //config CAN filter
}

/**
  * @brief CAN Transmit (after header, filter config)
  * @param hcan       : pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
  *        pTxHeader  : pointer to a CAN_TxHeaderTypeDef structure.
  *        data       : array containing the payload of the Tx frame.
  *        pTxMailbox : pointer to a variable where the function will return the TxMailbox used to store the Tx message. This parameter can be a value of @arg CAN_Tx_Mailboxes.
  * @retval None
  */
HAL_StatusTypeDef CANsend(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pTxHeader, uint8_t data[], uint32_t *pTxMailbox)
{
	return (HAL_CAN_AddTxMessage(hcan, pTxHeader, data, pTxMailbox)); //transmit over CAN, return HAL status
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader;
CAN_RxHeaderTypeDef RxHeader2;
//CAN_FilterTypeDef sFilterConfig;

uint32_t TxMailbox;
uint8_t  TxData[8];
uint8_t  RxData[8];

int datacheck = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Button_Pin)
  {
	if( GPIO_Button_Pin == GPIO_PIN_13)
		{
			 HAL_GPIO_TogglePin(Amarillo_GPIO_Port, Amarillo_Pin);
//		 	 TxData[0]=1;
//		 	 TxData[1]=2;
//		     TxData[2]=3;
//		 	 TxData[3]=4;
//		 	 TxData[4]=5;
//	         TxData[5]=6;
//			 TxData[6]=7;
//		 	 TxData[7]=8;
		     TxData[7] = TxData[7] + 1;
		 	 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		 		{
		 			HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
		 			Error_Handler ();
		 		}


/*		 	if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData) != HAL_OK)
		 		{
		 		    HAL_GPIO_TogglePin(Rojo_GPIO_Port, Rojo_Pin);
		 		    Error_Handler();
		 		}
*/
		}
  }


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan2)
  {
	HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);

	if (HAL_CAN_GetRxMessage(hcan2, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if ((RxHeader2.StdId == 0x103))
	  {
		  datacheck = 1;
	  }
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */


  //Se configuran los filtros para el puerto CAN

/*  sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh=0x12<<5;
  sFilterConfig.FilterIdLow=0;
  sFilterConfig.FilterMaskIdHigh=0;
  sFilterConfig.FilterMaskIdLow=0;
  sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation=ENABLE;
*/
 // HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig);
  if(HAL_CAN_Start(&hcan1) != HAL_OK)
  	  {
	  	  HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
	  	  Error_Handler();
  	  }

 // HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig);

  if(HAL_CAN_Start(&hcan2)!= HAL_OK)
  	  {
	      HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
  	  	  Error_Handler();
  	  }


  if ( HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  	  {
	      HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
  	  	  Error_Handler();
	  }


  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 23;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  RxHeader.IDE = CAN_ID_STD;
  RxHeader.StdId = 23;
  RxHeader.RTR = CAN_RTR_DATA;
  RxHeader.DLC = 8;

  TxHeader2.IDE = CAN_ID_STD;
  TxHeader2.StdId = 12;
  TxHeader2.RTR = CAN_RTR_DATA;
  TxHeader2.DLC = 8;
  TxHeader2.TransmitGlobalTime = DISABLE;

  RxHeader2.IDE = CAN_ID_STD;
  RxHeader2.StdId = 12;
  RxHeader2.RTR = CAN_RTR_DATA;
  RxHeader2.DLC = 8;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*  TxData[7] = TxData[7] + 1;

			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  		 	{
				   HAL_GPIO_TogglePin(Amarillo_GPIO_Port, Amarillo_Pin);
	  		 	   Error_Handler ();
	  		 	}

			HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
	        HAL_Delay(500);


			if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData) != HAL_OK)
	  		 	{
				   HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
	  		 	   Error_Handler ();
	  		 	}

	  if (datacheck)
	  {
		 for(int i=0; i< RxData[1]; i++ )
			 { HAL_GPIO_TogglePin(Amarillo_GPIO_Port, Amarillo_Pin);
		       HAL_Delay(50);
			 }
		       datacheck = 0;
	  }
*/

  //	  HAL_CAN_GetRxMessage(&hcan2, tamaño, &RxHeader, RxData);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Amarillo_Pin|Rojo_Pin|Azul_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Amarillo_Pin Rojo_Pin Azul_Pin */
  GPIO_InitStruct.Pin = Amarillo_Pin|Rojo_Pin|Azul_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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

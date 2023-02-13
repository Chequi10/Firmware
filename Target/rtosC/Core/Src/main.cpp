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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "can_service.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
QueueHandle_t COLA_1;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
TaskHandle_t task_handle_task_1;
TaskHandle_t task_handle_task_2;
TaskHandle_t task_handle_task_3;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader;
CAN_RxHeaderTypeDef RxHeader2;
CAN_FilterTypeDef sFilterConfig;

uint32_t TxMailbox;
uint8_t  TxData[1];
uint8_t  RxData[1];

int datacheck = 0;
int i=0,a;

int _write(int file,char *ptr,int len)
{
	 HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	 return len;
}


void Task_1( void* taskParmPtr )
{
	    while( 1 )
    {for(a=49;a<58;a++)
	  {  TxData[0] = a;

			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  		 	{
				   HAL_GPIO_TogglePin(Amarillo_GPIO_Port, Amarillo_Pin);
	  		 	   Error_Handler ();
	  		 	}
			printf("\nCAN2 RX:- CANID: %d, LEN: %d  RxData:%s\n\r",(char *)RxHeader2.StdId,( char *)RxHeader2.DLC,(uint8_t *)TxData);
			HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
			osDelay(500);


	  if (datacheck)
	  {
		  HAL_GPIO_TogglePin(Rojo_GPIO_Port, Rojo_Pin);
		  osDelay(500);

		  datacheck = 0;
	  }

	  }


    }


}

void Task_2( void* taskParmPtr )
{
	    while( 1 )
    {
       HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
       osDelay(50);
    }
}


void  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan2)
  {
	HAL_GPIO_TogglePin(Amarillo_GPIO_Port, Amarillo_Pin);

	if (HAL_CAN_GetRxMessage(hcan2, CAN_RX_FIFO0, &RxHeader2, RxData) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if ((RxHeader2.StdId == 146))
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


printf("Protocolo de Comuncacion CAN activo:\n\rCAN 1: PB8=Rx PB9=Tx\n\rCAN 2: PB5=Rx PB6=Tx \n\r");

TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 146;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 1;
  TxHeader.TransmitGlobalTime = DISABLE;

  RxHeader.IDE = CAN_ID_STD;
  RxHeader.StdId = 146;
  RxHeader.RTR = CAN_RTR_DATA;
  RxHeader.DLC = 1;

  TxHeader2.IDE = CAN_ID_STD;
  TxHeader2.StdId = 20;
  TxHeader2.RTR = CAN_RTR_DATA;
  TxHeader2.DLC = 1;
  TxHeader2.TransmitGlobalTime = DISABLE;

  RxHeader2.IDE = CAN_ID_STD;
  RxHeader2.StdId = 20;
  RxHeader2.RTR = CAN_RTR_DATA;
  RxHeader2.DLC = 1;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  BaseType_t res1 =
       xTaskCreate(
           Task_1,                     // Funcion de la tarea a ejecutar
           ( const char * )"Task 1",   // Nombre de la tarea como String amigable para el usuario
           configMINIMAL_STACK_SIZE*2, /* tamaño del stack de cada tarea (words) */
           NULL,                       // Parametros de tarea
           tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	   &task_handle_task_1             // Puntero a la tarea creada en el sistema
       );

  BaseType_t res2 =
       xTaskCreate(
           Task_2,                     // Funcion de la tarea a ejecutar
           ( const char * )"Task 2",   // Nombre de la tarea como String amigable para el usuario
           configMINIMAL_STACK_SIZE*2, /* tamaño del stack de cada tarea (words) */
           NULL,                       // Parametros de tarea
           tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	   &task_handle_task_2             // Puntero a la tarea creada en el sistema
       );


  configASSERT( res1 == pdPASS && res2 == pdPASS);

  COLA_1 = xQueueCreate( 1, sizeof(int  ));
     configASSERT( COLA_1 != NULL );
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

      CAN_FilterTypeDef  sFilterConfig;

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
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
  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	  sFilterConfig.FilterIdHigh=0;
	  sFilterConfig.FilterIdLow=0;
	  sFilterConfig.FilterMaskIdHigh=0;
	  sFilterConfig.FilterMaskIdLow=0;
	  sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterActivation=ENABLE;
      sFilterConfig.SlaveStartFilterBank = 14;

      if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        {
          /* Filter configuration Error */
          Error_Handler();
        }

      HAL_CAN_Start(&hcan1);

        /*##-4- Activate CAN RX notification #######################################*/
        if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        {
          /* Notification Error */
          Error_Handler();
        }
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

	      CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
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
  sFilterConfig.FilterBank = 14;
		  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		  sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
		  sFilterConfig.FilterIdHigh=0;
		  sFilterConfig.FilterIdLow=0;
		  sFilterConfig.FilterMaskIdHigh=0;
		  sFilterConfig.FilterMaskIdLow=0;
		  sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
		  sFilterConfig.FilterActivation=ENABLE;
	      sFilterConfig.SlaveStartFilterBank = 14;

	      if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
	        {
	          /* Filter configuration Error */
	          Error_Handler();
	        }

	      HAL_CAN_Start(&hcan2);

	        /*##-4- Activate CAN RX notification #######################################*/
	        if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	        {
	          /* Notification Error */
	          Error_Handler();
	        }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Amarillo_Pin|Rojo_Pin|Azul_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Amarillo_Pin Rojo_Pin Azul_Pin */
  GPIO_InitStruct.Pin = Amarillo_Pin|Rojo_Pin|Azul_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
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
#include <string.h>
#include "interface.h"
#include "printscreen.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_RATE_MS 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
interface stm32_interface;
//printer imprime;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
UART_HandleTypeDef huart3;

osThreadId Tarea_1Handle;
osThreadId Tarea_2Handle;
osThreadId Tarea_3Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
void StartTask01(void const *argument);
void StartTask02(void const *argument);
void StartTask03(void const *argument);

/* USER CODE BEGIN PFP */
TaskHandle_t task_handle_task_1;
TaskHandle_t task_handle_task_2;
TaskHandle_t task_handle_task_3;
SemaphoreHandle_t BinarySemaphoreHandle;
SemaphoreHandle_t BinarySemaphoreHandle2;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader;
CAN_RxHeaderTypeDef RxHeader2;
CAN_FilterTypeDef sFilterConfig;

uint32_t TxMailbox;
uint8_t TxData[1];


typedef struct {
	//  keys_ButtonState_t state;   //variables

	TickType_t time_down; //timestamp of the last High to Low transition of the key
	TickType_t time_up;	//timestamp of the last Low to High transition of the key
	TickType_t time_diff;	    //variables
} t_key_data;

t_key_data keys_data;
/* Contador de mensajes de SYNC enviados por canal 1. */

   char sync_counter;
void Task_serial_read_command(void *taskParmPtr) {
	while (1) {
		xSemaphoreTake(BinarySemaphoreHandle, portMAX_DELAY);
		stm32_interface.serial_read_command();

	}

}
void Task_can1_send_sync(void *taskParmPtr) {
	while (1) {

		    TxData[0] = sync_counter;

			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)
					!= HAL_OK) {
				Error_Handler();
			}
			  // Incrementar contador con rollover en 19.
			    sync_counter++;
			    sync_counter%=8;
			//HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
			vTaskDelay( LED_RATE_MS / portTICK_RATE_MS);


	}
}
void Task_can2_read_message(void *taskParmPtr) {
	while (1) {
		stm32_interface.can_read_message();

	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan2) {

	if (HAL_CAN_GetRxMessage(hcan2, CAN_RX_FIFO0, &RxHeader2, stm32_interface.RxData)
			!= HAL_OK) {
		Error_Handler();
	}


	if (stm32_interface.RxData[0]== 3) {
		stm32_interface.datacheck=1;
	}

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART3) {
		BaseType_t xHigherPriorityTaskWoken;

		xHigherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(BinarySemaphoreHandle, &xHigherPriorityTaskWoken);

		HAL_UART_Receive_IT(&huart3, stm32_interface.cadena, 12);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}

}

void config(void) {
	BaseType_t res1 = xTaskCreate(Task_serial_read_command, // Funcion de la tarea a ejecutar
			(const char*) "serial_read_command", // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE * 2, /* tamaño del stack de cada tarea (words) */
			NULL,                       // Parametros de tarea
			tskIDLE_PRIORITY + 2,         // Prioridad de la tarea
			&task_handle_task_1       // Puntero a la tarea creada en el sistema
			);

	BaseType_t res2 = xTaskCreate(Task_can1_send_sync, // Funcion de la tarea a ejecutar
			(const char*) "can1_send_sync", // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE * 2, /* tamaño del stack de cada tarea (words) */
			NULL,                       // Parametros de tarea
			tskIDLE_PRIORITY + 3,         // Prioridad de la tarea
			&task_handle_task_2       // Puntero a la tarea creada en el sistema
			);

	BaseType_t res3 = xTaskCreate(Task_can2_read_message, // Funcion de la tarea a ejecutar
			(const char*) "can2_read_message", // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE * 2, /* tamaño del stack de cada tarea (words) */
			NULL,                       // Parametros de tarea
			tskIDLE_PRIORITY + 1,         // Prioridad de la tarea
			&task_handle_task_3       // Puntero a la tarea creada en el sistema
			);

	configASSERT(res1 == pdPASS && res2 == pdPASS && res3 == pdPASS);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	HAL_UART_Receive_IT(&huart3, stm32_interface.cadena, 12);
	/* USER CODE BEGIN 2 */
	//  imprime.vPrintString("Protocolo de Comuncacion CAN activo:\n\rCAN 1: PB8=Rx PB9=Tx\n\rCAN 2: PB5=Rx PB6=Tx \n\r");
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
	BinarySemaphoreHandle = xSemaphoreCreateBinary();
	configASSERT(BinarySemaphoreHandle != NULL);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of Tarea_1 */

	osThreadDef(Tarea_1, StartTask01, osPriorityNormal, 0, 128);
	Tarea_1Handle = osThreadCreate(osThread(Tarea_1), NULL);

	/* definition and creation of Tarea_2 */
	osThreadDef(Tarea_2, StartTask02, osPriorityNormal, 0, 128);
	Tarea_2Handle = osThreadCreate(osThread(Tarea_2), NULL);

	/* definition and creation of Tarea_3 */
	osThreadDef(Tarea_3, StartTask03, osPriorityNormal, 0, 128);
	Tarea_3Handle = osThreadCreate(osThread(Tarea_3), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	config();
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		//service.setup();
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

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
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	HAL_CAN_Start(&hcan1);

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
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
static void MX_CAN2_Init(void) {

	/* USER CODE BEGIN CAN2_Init 0 */

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
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */
	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	HAL_CAN_Start(&hcan2);

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Amarillo_Pin | Rojo_Pin | Azul_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : Amarillo_Pin Rojo_Pin Azul_Pin */
	GPIO_InitStruct.Pin = Amarillo_Pin | Rojo_Pin | Azul_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
 * @brief  Function implementing the Tarea_1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask01 */
void StartTask01(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the Tarea_2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const *argument) {
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartTask02 */
}
/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the Tarea_3 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const *argument) {
	/* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartTask03 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

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
#include "jetson_watchdog.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "interface.h"
#include "printscreen.h"
#include "encoder.h"
#include "valve_controller.h"


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
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId Tarea_1Handle;
osThreadId Tarea_2Handle;
osThreadId Tarea_3Handle;
/* USER CODE BEGIN PV */

extern "C" {
    TIM_HandleTypeDef htim1;
    TIM_HandleTypeDef htim2;
    TIM_HandleTypeDef htim3;
    TIM_HandleTypeDef htim4;
    TIM_HandleTypeDef htim5;
    TIM_HandleTypeDef htim8;
}
Encoder encoder1(&htim2, 32);
Encoder encoder2(&htim5, 32);
Encoder encoder3(&htim3, 16);
Encoder encoder4(&htim4, 16);
Encoder encoder5(&htim8, 16);
Encoder encoder6(&htim1, 16);

constexpr uint8_t VALVE_MODULE_COUNT = 3;

ValveController valveModule1(&hcan1, 0x21);
ValveController valveModule2(&hcan1, 0x22);
ValveController valveModule3(&hcan1, 0x23);

ValveController* valveModules[VALVE_MODULE_COUNT] =
{
    &valveModule1,
    &valveModule2,
    &valveModule3
};

constexpr uint8_t ENCODER_COUNT = 6;

Encoder* encoders[ENCODER_COUNT] =
{
    &encoder1,
    &encoder2,
    &encoder3,
    &encoder4,
    &encoder5,
    &encoder6
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_DMA_Init(void);
void StartTask01(void const *argument);
void StartTask02(void const *argument);
void StartTask03(void const *argument);


/* USER CODE BEGIN PFP */
TaskHandle_t task_handle_jetson_serial_rx;

TaskHandle_t task_handle_encoder;
TaskHandle_t task_handle_can1_axiomatic_tx;
TaskHandle_t task_handle_can1_axiomatic_rx;
TaskHandle_t task_handle_jetson_telemetry_tx;
SemaphoreHandle_t BinarySemaphoreHandle_SERIAL;

QueueHandle_t AxiomaticRxQueueHandle;
QueueHandle_t JetsonRxQueueHandle;


#define JETSON_DMA_RX_BUFFER_SIZE 256

uint8_t jetson_dma_rx_buffer[JETSON_DMA_RX_BUFFER_SIZE];


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
uint8_t ant;
volatile int64_t encoder_position[ENCODER_COUNT] = {0};
volatile int32_t encoder_delta[ENCODER_COUNT] = {0};
volatile int8_t encoder_direction[ENCODER_COUNT] = {0};
volatile float encoder_height_mm[ENCODER_COUNT] = {0.0f};
volatile uint16_t target_height_mm[ENCODER_COUNT] = {0};
volatile uint32_t jetson_telemetry_tx_count = 0;
volatile uint32_t jetson_valve_tx_count = 0;
volatile uint32_t jetson_diagnostic_tx_count = 0;
volatile uint32_t jetson_stm32_state_tx_count = 0;
volatile uint32_t jetson_uart_error_count = 0;

volatile uint32_t jetson_rx_queue_dropped = 0;
volatile uint32_t jetson_valid_packet_count = 0;

volatile uint32_t jetson_watchdog_trip_count = 0;
volatile TickType_t jetson_last_watchdog_elapsed = 0;
volatile TickType_t jetson_max_packet_gap = 0;

volatile uint32_t jetson_dma_event_count = 0;
volatile uint32_t jetson_dma_byte_count = 0;
volatile uint16_t jetson_dma_last_size = 0;


/*
 * Cantidad de heartbeats válidos recibidos
 * desde la PC / Jetson.
 */
volatile uint32_t jetson_heartbeat_count = 0;



constexpr uint8_t VALVE_COUNT = 6;
volatile uint32_t axiomatic_rx_dropped = 0;
/*
 * Watchdog de comunicación con la Jetson.
 */
volatile TickType_t jetson_last_valid_packet_tick = 0;
volatile bool jetson_connection_ok = false;

const TickType_t JETSON_WATCHDOG_TIMEOUT =
    pdMS_TO_TICKS(500);

typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CanRxFrame_t;




typedef struct
{
    int16_t command;
    bool enabled;
} ValveCommand_t;

volatile ValveCommand_t valve_command[VALVE_COUNT] = {};


typedef struct {
	//  keys_ButtonState_t state;   //variables

	TickType_t time_down; //timestamp of the last High to Low transition of the key
	TickType_t time_up;	//timestamp of the last Low to High transition of the key
	TickType_t time_diff;	    //variables
} t_key_data;

t_key_data keys_data;
/* Contador de mensajes de SYNC enviados por canal 1. */

char sync_counter;

void Task_jetson_telemetry_tx(void *taskParmPtr)
{
    (void)taskParmPtr;

    while (1)
    {
        /*
         * OPCODE 'E':
         * alturas actuales de los 6 encoders.
         */
        stm32_interface.send_encoder_state();
        jetson_telemetry_tx_count++;

        /*
         * OPCODE 'F':
         * comandos actuales de los 6 cuerpos.
         */
        stm32_interface.send_valve_state();
        jetson_valve_tx_count++;

        stm32_interface.send_diagnostic_state();
        jetson_diagnostic_tx_count++;

        /*
         * OPCODE 'H':
         * estado general de la STM32.
         */
        stm32_interface.send_stm32_state();
        jetson_stm32_state_tx_count++;

        /*
         * Telemetría a 10 Hz.
         */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Task_jetson_serial_rx(void *taskParmPtr)
{
    (void)taskParmPtr;

    uint8_t byte;

    while (1)
    {
        if (xQueueReceive(
                JetsonRxQueueHandle,
                &byte,
                portMAX_DELAY) == pdPASS)
        {
        	stm32_interface.serial_feed_byte(byte);
        }
    }
}



void Task_encoder(void *taskParmPtr)
{
    (void)taskParmPtr;

    /*
     * Marca de tiempo utilizada para hacer titilar
     * el LED sin bloquear la lectura de encoders.
     */
    TickType_t lastLedToggle = xTaskGetTickCount();

    /*
     * Iniciar los seis timers configurados
     * en modo encoder.
     */
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
        encoders[i]->start();
    }

    /*
     * Calibraciones provisorias.
     *
     * Parámetros:
     * 1) cuentas en posición baja
     * 2) cuentas en posición alta
     * 3) altura mínima en milímetros
     * 4) altura máxima en milímetros
     */
    encoder1.setCalibration(0, 20000, 400.0f, 1100.0f);
    encoder2.setCalibration(0, 20000, 300.0f, 1100.0f);
    encoder3.setCalibration(0, 20000, 200.0f, 1100.0f);
    encoder4.setCalibration(0, 20000, 470.0f, 1100.0f);
    encoder5.setCalibration(0, 20000, 800.0f, 1100.0f);
    encoder6.setCalibration(0, 20000, 300.0f, 1100.0f);

    while (1)
    {
        /*
         * Leer y procesar los seis encoders.
         */
        for (uint8_t i = 0; i < ENCODER_COUNT; i++)
        {
            encoders[i]->update();

            encoder_position[i] =
                encoders[i]->getPosition();

            encoder_delta[i] =
                encoders[i]->getDelta();

            encoder_direction[i] =
                encoders[i]->getDirection();

            encoder_height_mm[i] =
                encoders[i]->getHeightMm();
        }

        /*
         * Heartbeat de la tarea:
         * cambia el estado del LED azul cada 500 ms.
         *
         * Si el LED deja de titilar, significa que
         * esta tarea dejó de ejecutarse.
         */
        TickType_t currentTick = xTaskGetTickCount();

        if ((currentTick - lastLedToggle) >= pdMS_TO_TICKS(500))
        {
            HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
            lastLedToggle = currentTick;
        }

        /*
         * Actualización de los encoders cada 10 ms.
         */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setBodyValveCommand(uint8_t body, int16_t command)
{
    /*
     * body:
     * 0 = cuerpo 1
     * 1 = cuerpo 2
     * ...
     * 5 = cuerpo 6
     *
     * command:
     * -1000 = bajar al máximo
     *     0 = detener
     * +1000 = subir al máximo
     */

    if (body >= VALVE_COUNT)
    {
        return;
    }

    /*
     * Limitar la orden al rango válido.
     */
    if (command > 1000)
    {
        command = 1000;
    }
    else if (command < -1000)
    {
        command = -1000;
    }

    /*
     * Cada cuerpo usa dos salidas:
     *
     * cuerpo 0 → salidas globales 0 y 1
     * cuerpo 1 → salidas globales 2 y 3
     * cuerpo 2 → salidas globales 4 y 5
     * ...
     */
    uint8_t globalOutputUp   = body * 2;
    uint8_t globalOutputDown = globalOutputUp + 1;

    /*
     * Determinar en qué módulo está cada salida.
     */
    uint8_t moduleUp   = globalOutputUp / 4;
    uint8_t outputUp   = globalOutputUp % 4;

    uint8_t moduleDown = globalOutputDown / 4;
    uint8_t outputDown = globalOutputDown % 4;

    /*
     * Seguridad:
     * primero apagar ambas bobinas.
     */
    valveModules[moduleUp]->setOutput(outputUp, 0);
    valveModules[moduleDown]->setOutput(outputDown, 0);

    /*
     * Aplicar solamente una dirección.
     */
    if (command > 0)
    {
        /*
         * Subir.
         */
        valveModules[moduleUp]->setOutput(
            outputUp,
            static_cast<uint16_t>(command)
        );
    }
    else if (command < 0)
    {
        /*
         * Bajar.
         */
        valveModules[moduleDown]->setOutput(
            outputDown,
            static_cast<uint16_t>(-command)
        );
    }

    /*
     * Guardar el comando para diagnóstico.
     */
    valve_command[body].command = command;
    valve_command[body].enabled = (command != 0);
}

int16_t getBodyValveCommand(uint8_t body)
{
    if (body >= VALVE_COUNT)
    {
        return 0;
    }

    return valve_command[body].command;
}

void Task_can1_axiomatic_tx(void *taskParmPtr)
{
    (void)taskParmPtr;

    /*
     * Comenzar siempre con las doce
     * salidas desactivadas.
     */

    for (uint8_t i = 0; i < VALVE_MODULE_COUNT; i++)
    {
        valveModules[i]->disableAll();
    }
    for (uint8_t body = 0; body < VALVE_COUNT; body++)
    {
        setBodyValveCommand(body, 0);
    }

    TickType_t lastLedToggle = xTaskGetTickCount();
    while (1)
    {
        TickType_t currentTick = xTaskGetTickCount();

        /*
         * Watchdog de comunicación con la Jetson.
         *
         * Solamente se controla después de haber recibido
         * al menos un paquete válido.
         */
        if (jetson_connection_ok)
        {
            TickType_t elapsed =
                currentTick - jetson_last_valid_packet_tick;

            if (elapsed > jetson_max_packet_gap)
            {
                jetson_max_packet_gap = elapsed;
            }

            if (elapsed >= JETSON_WATCHDOG_TIMEOUT)
            {  jetson_watchdog_trip_count++;
               jetson_last_watchdog_elapsed = elapsed;
                /*
                 * Se perdió la comunicación:
                 * detener los seis cuerpos.
                 */
                for (uint8_t body = 0;
                     body < VALVE_COUNT;
                     body++)
                {
                    setBodyValveCommand(body, 0);
                }

                /*
                 * Evita ejecutar continuamente el apagado.
                 * Un nuevo paquete válido volverá a ponerla
                 * en true desde protocol.cpp.
                 */
                jetson_connection_ok = false;

            }
        }

        /*
         * Transmitir por CAN1 las órdenes actuales
         * hacia los tres módulos Axiomatic.
         */
        for (uint8_t i = 0;
             i < VALVE_MODULE_COUNT;
             i++)
        {
            valveModules[i]->send();
        }
        /*
         * Heartbeat de la tarea CAN1.
         *
         * Si este LED deja de titilar,
         * significa que la tarea dejó
         * de ejecutarse.
         */


        if ((currentTick - lastLedToggle) >= pdMS_TO_TICKS(1000))
        {
            HAL_GPIO_TogglePin(Rojo_GPIO_Port, Rojo_Pin);
            lastLedToggle = currentTick;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void Task_can1_axiomatic_rx(void *taskParmPtr)
{
    (void)taskParmPtr;

    CanRxFrame_t frame;

    while (1)
    {
        /*
         * La tarea queda bloqueada hasta que llegue
         * un mensaje por CAN1.
         */
        if (xQueueReceive(
                AxiomaticRxQueueHandle,
                &frame,
                portMAX_DELAY) == pdPASS)
        {
            /*
             * Acá procesaremos los mensajes recibidos
             * desde los módulos Axiomatic.
             *
             * frame.header.ExtId  -> identificador J1939
             * frame.header.DLC    -> cantidad de bytes
             * frame.data[0..7]    -> datos recibidos
             */

            /*
             * Por ahora no hacemos nada con el mensaje.
             * Más adelante interpretaremos:
             *
             * - corriente real de cada salida;
             * - circuito abierto;
             * - cortocircuito;
             * - sobretemperatura;
             * - estado del módulo;
             * - confirmación de comandos.
             */
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /*
     * Recepción desde los módulos Axiomatic por CAN1.
     */
    if (hcan->Instance == CAN1)
    {
        CanRxFrame_t frame;

        if (HAL_CAN_GetRxMessage(
                hcan,
                CAN_RX_FIFO0,
                &frame.header,
                frame.data) == HAL_OK)
        {
            /*
             * Durante el arranque puede llegar una trama
             * antes de que la cola haya sido creada.
             */
            if (AxiomaticRxQueueHandle != NULL)
            {
                if (xQueueSendFromISR(
                        AxiomaticRxQueueHandle,
                        &frame,
                        &xHigherPriorityTaskWoken) != pdPASS)
                {
                    /*
                     * La cola está llena.
                     * La trama se descarta y se registra.
                     */
                	axiomatic_rx_dropped++;
                }
            }
        }
    }




    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        jetson_uart_error_count++;

        /*
         * Detener la recepción DMA actual.
         */
        HAL_UART_DMAStop(huart);

        /*
         * Limpiar errores UART.
         */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        /*
         * Reiniciar recepción DMA + IDLE.
         */
        if (HAL_UARTEx_ReceiveToIdle_DMA(
                &huart3,
                jetson_dma_rx_buffer,
                JETSON_DMA_RX_BUFFER_SIZE) == HAL_OK)
        {
            /*
             * No queremos interrupción
             * de Half Transfer.
             */
            __HAL_DMA_DISABLE_IT(
                huart3.hdmarx,
                DMA_IT_HT
            );
        }
    }
}

void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size)
{
    if (huart->Instance != USART3)
    {
        return;
    }

    jetson_dma_event_count++;
    jetson_dma_byte_count += Size;
    jetson_dma_last_size = Size;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /*
     * Posición anterior dentro del buffer circular DMA.
     */
    static uint16_t old_pos = 0;

    /*
     * Caso normal:
     * los nuevos bytes están entre old_pos y Size.
     */
    if (Size > old_pos)
    {
        for (uint16_t i = old_pos; i < Size; i++)
        {
            uint8_t byte = jetson_dma_rx_buffer[i];

            if (xQueueSendFromISR(
                    JetsonRxQueueHandle,
                    &byte,
                    &xHigherPriorityTaskWoken) != pdPASS)
            {
                jetson_rx_queue_dropped++;
            }
        }
    }

    /*
     * El DMA circular dio la vuelta al buffer.
     */
    else if (Size < old_pos)
    {
        for (uint16_t i = old_pos;
             i < JETSON_DMA_RX_BUFFER_SIZE;
             i++)
        {
            uint8_t byte = jetson_dma_rx_buffer[i];

            if (xQueueSendFromISR(
                    JetsonRxQueueHandle,
                    &byte,
                    &xHigherPriorityTaskWoken) != pdPASS)
            {
                jetson_rx_queue_dropped++;
            }
        }

        for (uint16_t i = 0; i < Size; i++)
        {
            uint8_t byte = jetson_dma_rx_buffer[i];

            if (xQueueSendFromISR(
                    JetsonRxQueueHandle,
                    &byte,
                    &xHigherPriorityTaskWoken) != pdPASS)
            {
                jetson_rx_queue_dropped++;
            }
        }
    }

    old_pos = Size;

    if (old_pos >= JETSON_DMA_RX_BUFFER_SIZE)
    {
        old_pos = 0;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void config(void) {
	BaseType_t res1 = xTaskCreate(
					Task_jetson_serial_rx,
					"jetson_serial_rx",
					configMINIMAL_STACK_SIZE * 2,
					NULL,
					tskIDLE_PRIORITY + 2,
					&task_handle_jetson_serial_rx
				);





	BaseType_t res4 = xTaskCreate(Task_encoder, // Funcion de la tarea a ejecutar
				(const char*) "encoder", // Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE * 2, /* tamaño del stack de cada tarea (words) */
				NULL,                       // Parametros de tarea
				tskIDLE_PRIORITY + 1,         // Prioridad de la tarea
				&task_handle_encoder       // Puntero a la tarea creada en el sistema
				);
	BaseType_t res5 = xTaskCreate(
				Task_can1_axiomatic_tx,
				"axiomatic_tx",
				configMINIMAL_STACK_SIZE * 2,
				NULL,
				tskIDLE_PRIORITY + 2,
				&task_handle_can1_axiomatic_tx
			);
	BaseType_t res6 = xTaskCreate(
					Task_can1_axiomatic_rx,
					"axiomatic_rx",
					configMINIMAL_STACK_SIZE * 2,
					NULL,
					tskIDLE_PRIORITY + 2,
					&task_handle_can1_axiomatic_rx
				);

	BaseType_t res7 = xTaskCreate(
					Task_jetson_telemetry_tx,
					"jetson_telemetry_tx",
					configMINIMAL_STACK_SIZE * 2,
					NULL,
					tskIDLE_PRIORITY + 1,
					&task_handle_jetson_telemetry_tx
				);

	configASSERT(res1 == pdPASS  && res4 == pdPASS  && res5 == pdPASS && res6 == pdPASS && res7 == pdPASS);
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
	MX_DMA_Init();
	MX_CAN1_Init();



	MX_USART3_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();



	/* USER CODE BEGIN 2 */
	//  imprime.vPrintString("Protocolo de Comuncacion CAN activo:\n\rCAN 1: PB8=Rx PB9=Tx\n\rCAN 2: PB5=Rx PB6=Tx \n\r");
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 111111111;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;
	TxHeader.TransmitGlobalTime = DISABLE;

	RxHeader.IDE = CAN_ID_STD;
	RxHeader.StdId = 100000001;
	RxHeader.RTR = CAN_RTR_DATA;
	RxHeader.DLC = 2;

	TxHeader2.IDE = CAN_ID_STD;
	TxHeader2.StdId = 100000024;
	TxHeader2.RTR = CAN_RTR_DATA;
	TxHeader2.DLC = 2;
	TxHeader2.TransmitGlobalTime = DISABLE;

	RxHeader2.IDE = CAN_ID_STD;
	RxHeader2.StdId = 100000024;
	RxHeader2.RTR = CAN_RTR_DATA;
	RxHeader2.DLC = 2;

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	JetsonRxQueueHandle =
	    xQueueCreate(
	        128,
	        sizeof(uint8_t)
	    );

	configASSERT(
	    JetsonRxQueueHandle != NULL
	);

	if (HAL_UARTEx_ReceiveToIdle_DMA(
	        &huart3,
	        jetson_dma_rx_buffer,
	        JETSON_DMA_RX_BUFFER_SIZE) != HAL_OK)
	{
	    Error_Handler();
	}

	/*
	 * No necesitamos interrupción
	 * de mitad de transferencia.
	 */
	__HAL_DMA_DISABLE_IT(
	    huart3.hdmarx,
	    DMA_IT_HT
	);



	AxiomaticRxQueueHandle = xQueueCreate(
	    16,
	    sizeof(CanRxFrame_t)
	);

	configASSERT(AxiomaticRxQueueHandle != NULL);

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
static void MX_DMA_Init(void)
{
    /*
     * Habilitar clock del controlador DMA1.
     */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /*
     * Interrupción DMA1 Stream 1
     * utilizada por USART3_RX.
     */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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

static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}
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

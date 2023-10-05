/*
 * printscreen.cpp
 *
 *  Created on: 28 feb. 2023
 *      Author: ezequiel
 */

#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include "printscreen.h"
#include <stdbool.h>

printer::printer () {
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
}

void printer::vPrintString( const char *pcString )
{
	/* Print the string, using a critical section as a crude method of mutual
	exclusion. */

	taskENTER_CRITICAL();

		HAL_UART_Transmit( &huart3, (uint8_t *)pcString, (uint16_t) strlen((char *)pcString), HAL_MAX_DELAY );

	taskEXIT_CRITICAL();
}

uint8_t printer::uartRecvString(uint8_t *ptrstring, uint8_t cantidad)
{

	HAL_UART_Receive_IT(&huart3,ptrstring, 1);

}




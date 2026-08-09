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

printer::printer ()
{
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
	//taskENTER_CRITICAL();
	if (HAL_UART_Receive(&huart3,ptrstring, cantidad,1000) == HAL_OK) return true;
	return false;
	//taskENTER_CRITICAL();
}




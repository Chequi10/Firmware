/*
 * API_uart.c
 *
 *  Created on: 24 nov. 2021
 *      Author: ezequiel
 */
#include "API_uart.h"

static UART_HandleTypeDef UartHandle;

bool_t uartInit(){

	  UartHandle.Instance        = USART3;
	  UartHandle.Init.BaudRate   = 4800;
	  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	  UartHandle.Init.StopBits   = UART_STOPBITS_1;
	  UartHandle.Init.Parity     = UART_PARITY_NONE;
	  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	  UartHandle.Init.Mode       = UART_MODE_TX_RX;
	  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	  bool_t EstadoUart;
	  uint8_t miString[] = "BaudRate = 4800;\n\rDataBits=8;\n\rStopBits = 1;\n\rParity=NONE\n\n\rListo para recibir trama NMEA\n\r";


	  if (HAL_UART_Init(&UartHandle) != HAL_OK){
		  EstadoUart = false;
	  }	else {
		  EstadoUart = true;
		  HAL_UART_Transmit(&UartHandle, (uint8_t *) miString, sizeof(miString)/sizeof(char), 1000);
	  }

	  return EstadoUart;

}

void uartSendString(uint8_t *ptrstring){

	uint8_t cant=0;

	while(*(ptrstring+cant) != 0) cant++;
	HAL_UART_Transmit(&UartHandle, ptrstring, cant, 1000);

}

bool_t uartRecvString(uint8_t *ptrstring, uint8_t cantidad){
	if (HAL_UART_Receive(&UartHandle,ptrstring, cantidad,10) == HAL_OK) return true;
	return false;
}



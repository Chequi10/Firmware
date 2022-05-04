/*
 * API_uart.h
 *
 *  Created on: 24 nov. 2021
 *      Author: ezequiel
 */
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


#ifndef API_INC_API_UART_H_
#define API_INC_API_UART_H_

/*defino variable booleana*/
typedef bool bool_t;



bool_t uartInit();                         /*inicializa la UART y configura los parametros de comunicacion*/
void uartSendString(uint8_t *ptrstring);                       /*esta funcion envia datos por la UART*/
bool_t uartRecvString(uint8_t *ptrstring, uint8_t cantidad);   /*esta funcion recibe datos por la UART*/

#endif /* API_API_UART_H_ */


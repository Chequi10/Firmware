/*
 * printscreen.h
 *
 *  Created on: 28 feb. 2023
 *      Author: ezequiel
 */
#include <stdbool.h>
#ifndef INC_PRINTSCREEN_H_
#define INC_PRINTSCREEN_H_

class printer
{
	private:
	UART_HandleTypeDef huart3;
	typedef bool bool_t;


	public:
	printer();
	uint8_t uartRecvString(uint8_t *ptrstring, uint8_t cantidad);   /*esta funcion recibe datos por la UART*/
	void vPrintString( const char *pcString );

};


#endif /* INC_PRINTSCREEN_H_ */

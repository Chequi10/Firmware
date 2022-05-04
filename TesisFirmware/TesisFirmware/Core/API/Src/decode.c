/*
 * decode.c
 *
 *  Created on: 19 nov. 2021
 *      Author: ezequiel
 */

#include "decode.h"
#include <string.h>


/* Typedef -----------------------------------------------------------*/

typedef enum{
	S0,         /*para el estado del caracter $*/
	S1,			/*para el estado del caracter G*/
	S2,			/*para el estado del caracter P*/
	S3			/*para el estado del caracter R*/
} decode_t;

// Variable de estado (global)
 decode_t currentStatus;
 decode_t startNewState;
 decode_t startOldState;
/* Private variables ---------------------------------------------------------*/
static uint8_t startFrame[4] = {'$', 'G', 'P', 'R'};
static uint8_t endFrame = '*';
/* ---------------------------------------------------------------------------*/
uint8_t * myBuffer;
uint8_t myLength;
bool_t flagStart;
uint8_t counter;

/* Private function prototypes -----------------------------------------------*/
static uint8_t isStartFrame(uint8_t val);
static uint8_t isEndFrame(uint8_t val);


/* ---------------------------------------------------------------------------*/

/* Esta funcion se utiliza para la inicializacion de datos y la MEF */

bool_t decodeInit(uint8_t * buffer, uint8_t length)
{
	currentStatus = S0;
	startNewState = S0;
	startOldState = S0;
	flagStart = false;
	counter = 0;
	myBuffer = buffer;
	myLength = length;
	return true;
}

/* Esta funcion recibe un dato del puerto serie y los analiza*/

int8_t getData(uint8_t data, uint8_t largo_trama){
	int8_t r = 0;

	if (isStartFrame(data) && !flagStart) {
		flagStart = true;
	} else {
		if (flagStart) {
			if (isEndFrame(data)) {
				if (largo_trama == counter) {
					r= counter;
				}
				flagStart=false;
				counter=0;
			} else {
				myBuffer[counter] = data;
				counter++;
			}
		}
	}


	return r;
}

/* Esta función se utiliza para validar el inicio de la trama  con el patron de caracteres $GPR*/

uint8_t isStartFrame(uint8_t val){

	bool_t flag = false;
	switch (startOldState){
		case S0:
		{
			if(val ^ startFrame[0]){
				startNewState = S0;
			}else{
				startNewState = S1;
			}
			break;
		}
		case S1:
		{
			if(val ^ startFrame[1]){
				startNewState = S0;
			}else{
				startNewState = S2;
			}
			break;
		}
		case S2:
		{
			if(val ^ startFrame[2]){
				startNewState = S0;
			}else{
				startNewState = S3;
			}
			break;
		}
		case S3:
		{
			if(val ^ startFrame[3]){
				startNewState = S0;
			}else{
				startNewState = S0;
				flag = true;
			}

			break;
		}
		default:
			//decodeInit();
			break;
	}
	startOldState = startNewState;

	return flag;
}

/* Esta función se utiliza para validar el fin de la trama con el caracter (*)   */

uint8_t isEndFrame(uint8_t val){

	bool_t flag = false;
	if(val == endFrame){
		flag = true;
	}
	return flag;
}

/* Esta función se utiliza para filtrar/obtener los datos del horario, latitud y longitud*/

uint8_t * filter(uint8_t *data){
	static uint8_t salida[255];
	uint8_t horario[7];
	uint8_t latitud[12];
	uint8_t longitud[13];
	memcpy(horario,data+3,6);     //hora
	memcpy(latitud,data+16,11);   //latitud
	memcpy(longitud,data+28,12);  //longitud
	horario[6] = 0;
	latitud[11] = 0;
	longitud[12] = 0;
	sprintf((char *)salida,"\nDatos de posicionamiento> Hora UTC: %s  Latitud: %s  Longitud %s\n\r",horario,latitud,longitud);

	return salida;
}






/*
 * can_service.cpp
 *
 *  Created on: 4 feb. 2023
 *      Author: ezequiel
 */
#include "main.h"
#include "cmsis_os.h"
#include <errno.h>
#include <interface.h>
#include <stdio.h>
#include <string.h>
#include "printscreen.h"
printer imp;

interface::interface() :
		opcodes {

		{ &interface::cmd_send_message, opcode_flags::default_flags }

		},

		sync_counter(0)
//    can_event_thread(osPriorityHigh)
{
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
}

void interface::setup() {

}

/* Enviar mensaje SYNC desde CAN 1. */
void interface::can1_send_sync_message() {

}

/* Leer mensaje de CAN e imprimir en pantalla */
void interface::can_read_message() {

	if (datacheck == 1) {
		HAL_GPIO_TogglePin(Amarillo_GPIO_Port, Amarillo_Pin);
		datacheck = 0;
	}
}

void interface::serial_read_command() {

	for (size_t i = 0; i < 12; i++) {
		feed(cadena[i]);

	}
}

void interface::handle_packet(const uint8_t *payload, uint8_t n) {
	// El byte 0 es el código de opcode, el resto el payload. */
	uint8_t opcode = payload[0];
	switch (opcode) {
	// Event 0: envio mensaje can.
	case '0': {
		HAL_GPIO_TogglePin(Azul_GPIO_Port, Azul_Pin);
		uint8_t buffer[100];
		uint8_t buffer1[20];
		uint8_t buffer3[10];

		buffer[0] = 'P';
		buffer[1] = 'K';
		buffer[2] = 'T';
		buffer[3] = '!';
		buffer[4] = 0x6;
		buffer[5] = '0';
		for (uint8_t sdf = 6; sdf <= 10; sdf++) {
			buffer1[5] = '0';
			buffer1[sdf] = 0x66 + sdf - 6;
			//	uint16_t crc = calc_crc16(buffer1, 5);
			buffer[sdf] = buffer1[sdf];
			buffer[11] = 0b10110011;
			//	buffer[11]=(crc >> 8) & 0xFF;
			//	buffer[12]=crc & 0xFF;
			buffer[12] = 0b10101110;
			buffer[13] = '\n';

		}
		send_impl(buffer, 12);
		//imp.vPrintString((char*) buffer);
	}
		break;
		// Event 1: enciende led rojo.
	case '3': {
		HAL_GPIO_TogglePin(Rojo_GPIO_Port, Rojo_Pin);
	}
		(opcode < OPCODE_LAST) ?
				(this->*(opcodes[opcode].fn))(payload + 1, n - 1) :
				error_code::unknown_opcode;

	default: {

		// error, unknown packet
		//	this->set_error(error_code::unknown_opcode);
		this->reset();
	}
	}

}

void interface::send_impl(const uint8_t *buf, uint8_t n) {
	taskENTER_CRITICAL();

	HAL_UART_Transmit(&huart3, (uint8_t*) buf, (uint16_t) strlen((char*) buf),
			HAL_MAX_DELAY);

	taskEXIT_CRITICAL();
}

interface::error_code interface::cmd_send_message(const uint8_t *payload,
		uint8_t n) {
	TxHeader2.StdId = (payload[1] << 24) | (payload[2] << 16)
			| (payload[3] << 8) | (payload[4] << 0);
	TxHeader.DLC = payload[5];
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*) &payload[6], &TxMailbox);
	return error_code::success;
}


/*
 * can_service.h
 *
 *  Created on: 4 feb. 2023
 *      Author: ezequiel
 */

#ifndef INC_CAN_SERVICE_H_
#define INC_CAN_SERVICE_H_

#pragma once

#if !DEVICE_CAN

#endif

#include "main.h"
//#include "config.h"
#include "protocol.h"
#include "cmd_def.h"
#include "cmsis_os.h"


class interface:
    private protocol::packet_encoder,
    private protocol::packet_decoder
{
public:
	interface();
    ~interface(){};
    void setup();

    void serial_read_command();
    void can_read_message();
    void can1_send_sync_message();
    /* Contador de mensajes de SYNC enviados por canal 1. */
    char sync_counter;
    CAN_HandleTypeDef hcan1;
    CAN_HandleTypeDef hcan2;
    UART_HandleTypeDef huart3;
    CAN_TxHeaderTypeDef TxHeader;
    CAN_TxHeaderTypeDef TxHeader2;
    CAN_RxHeaderTypeDef RxHeader;
    CAN_RxHeaderTypeDef RxHeader2;
    uint32_t TxMailbox;

    /* Cola de eventos de acceso a dispositivo CAN (prioridad alta) */
 //   EventQueue can_dev_queue;

    /* Hilo de alta prioridad para manejo de dispositivos. */
 //   Thread can_event_thread;

    /* Ticker para mensajes SYNC */
 //   Ticker can_sync_ticker;

    /* Protocolo de comunicación serie */
	using opcode_callback = interface::error_code(interface::*)(const uint8_t* payload, uint8_t n);

    /** Telecomandos */
    /** Flags de un telecomando
     *  Por defecto los telecomandos sólo se reciben y se procesan.
     */
	enum opcode_flags {
		default_flags = 0x00
	};

    /** Descriptor de un OPCODE. */
	struct opcode_descr {
		opcode_callback fn;
		uint8_t flags;
	};

    /* Tabla de opcodes */
	opcode_descr opcodes[OPCODE_LAST];

	/* requeridos por packet_decoder */

    /** Dispatcher de telecomandos recibidos.
      * @param payload bytes del paylaod del telecomando.
      * @param n cantidad de bytes.
      * @warning Tener mucho cuidado de desde donde se llama a esta función.
      *          Si se está ejecutando la FSM en el contexto de interrupciones,
      *          Despachar al contexto de aplicación.
      */
	void handle_packet(const uint8_t* payload, uint8_t n) override;

    /** Implementación de envío de bytes. En este caso se utiliza puerto serie.
     * @param buf buffer con datos a transmitir (inmutable).
     * @param n cantidad de bytes.
     */
	void send_impl(const uint8_t* buf, uint8_t n) override;

    /* Comandos */

    interface::error_code cmd_send_message(const uint8_t* payload, uint8_t n);
};




#endif /* INC_CAN_SERVICE_H_ */

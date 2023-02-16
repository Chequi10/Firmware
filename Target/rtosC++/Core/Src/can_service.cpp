/*
 * can_service.cpp
 *
 *  Created on: 4 feb. 2023
 *      Author: ezequiel
 */
#include "main.h"
#include "cmsis_os.h"
#include "can_service.h"
#include <errno.h>

can_service::can_service()
:
		opcodes
		{

			{ &can_service::cmd_send_message, opcode_flags::default_flags }
		},

    sync_counter(0)
//    can_event_thread(osPriorityHigh)
{}



void can_service::setup()
{          static char buf[32] = {0};
     		HAL_UART_Receive(&huart3, (uint8_t *)buf, sizeof(buf), 1000);

    /* Registrar la función de lectura de CAN en la interrupción Rx.
       Notar que la función se ejecuta en una cola, y no directamente en la interrupción. */
//   can[0].attach(can_dev_queue.event(this, &can_service::can_read_message, 0), CAN::IrqType::RxIrq);

//    serial_port.sigio(can_dev_queue.event(this,&can_service::serial_read_command));

//    can_event_thread.start(callback(&can_dev_queue, &EventQueue::dispatch_forever));

    /* Programación de mensajes SYNC. */

//    can_sync_ticker.attach(can_dev_queue.event(this, &can_service::can1_send_sync_message), 50ms);
}

/* Enviar mensaje SYNC desde CAN 1. */
void can_service::can1_send_sync_message()
{
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.StdId = 146;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.DLC = 1;
	  TxHeader.TransmitGlobalTime = DISABLE;
	  RxHeader.IDE = CAN_ID_STD;
	  RxHeader.StdId = 146;
	  RxHeader.RTR = CAN_RTR_DATA;
	  RxHeader.DLC = 1;


    char data[2];
    data[0] = 0;
    data[1] = sync_counter;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*)data, &TxMailbox) != HAL_OK)
     	{
    	    Error_Handler ();
    	}
    // Incrementar contador con rollover en 19.
    sync_counter++;
    sync_counter%=19;


}

/* Leer mensaje de CAN e imprimir en pantalla */
void can_service::can_read_message()
{

	      TxHeader2.IDE = CAN_ID_STD;
		  TxHeader2.StdId = 20;
		  TxHeader2.RTR = CAN_RTR_DATA;
		  TxHeader2.DLC = 1;
		  TxHeader2.TransmitGlobalTime = DISABLE;
		  RxHeader2.IDE = CAN_ID_STD;
		  RxHeader2.StdId = 20;
		  RxHeader2.RTR = CAN_RTR_DATA;
		  RxHeader2.DLC = 1;
          uint8_t RxData[8];

          if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData) != HAL_OK)
          	  {
          	    Error_Handler();
          	  }
		        get_payload_buffer()[0] = 0x0;  // Event type
		        get_payload_buffer()[1] = 1; // Device Id
		        get_payload_buffer()[2] = RxHeader2.StdId >> 24; //
		        get_payload_buffer()[3] = RxHeader2.DLC >> 16;
		        get_payload_buffer()[4] = RxHeader2.DLC >> 8;
		        get_payload_buffer()[5] = RxHeader2.DLC >> 0;
		        get_payload_buffer()[6] = RxHeader2.DLC;
		        for(size_t i=0;i<RxHeader2.DLC;i++)
		        {
		            get_payload_buffer()[7+i] = RxData[i];
		        }
		        send( 7 + RxHeader2.DLC );

}

void can_service::serial_read_command()
{

	    static char buf[32] = {0};
        _ssize_t n = HAL_UART_Receive(&huart3, (uint8_t *)buf, sizeof(buf), 1000);

        if (EAGAIN != n)
        {
            char* pbuf = buf;
            do {
                feed(*pbuf++);
            } while(--n);
        }


}


void can_service::handle_packet(const uint8_t* payload, uint8_t n)
{
    // El byte 0 es el código de opcode, el resto el payload. */
	uint8_t opcode = payload[0];

    (opcode < OPCODE_LAST) ? (this->*(opcodes[opcode].fn))(payload + 1, n - 1) : error_code::unknown_opcode;
}


void can_service::send_impl(const uint8_t* buf, uint8_t n)
{

	HAL_UART_Transmit(&huart3, buf, n, 1000);
}



can_service::error_code can_service::cmd_send_message(const uint8_t* payload, uint8_t n)
{
    TxHeader2.StdId= (payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | (payload[4] << 0);
    TxHeader.DLC= payload[5];
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*)&payload[6], &TxMailbox);
    return error_code::success;
}


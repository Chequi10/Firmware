#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#pragma once

#include "main.h"
#include "protocol.h"
#include "cmsis_os.h"

class interface :
    private protocol::packet_encoder,
    private protocol::packet_decoder
{
public:

    interface();
    ~interface() {}

    /*
     * Buffer recibido por UART.
     * Actualmente la recepción se hace en bloques de 12 bytes.
     */
    uint8_t cadena[12];

    void send_imu_state();

    void serial_feed_byte(uint8_t byte);

    /*
     * Recepción UART:
     * entrega los bytes al decoder del protocolo.
     */
    void serial_read_command();

    /*
     * Enviar a la Jetson el estado
     * de altura de los seis encoders.
     *
     * OPCODE 'E'
     */
    void send_encoder_state();

    /*
     * Enviar a la Jetson el estado actual
     * de comando de los seis cuerpos.
     *
     * OPCODE 'F'
     */
    void send_valve_state();
    /*
     * Enviar diagnóstico general hacia la Jetson.
     *
     * OPCODE 'G'
     */
    void send_diagnostic_state();

    /*
     * Enviar estado general de la STM32
     * hacia la Jetson.
     *
     * OPCODE 'H'
     */
    void send_stm32_state();


    /*
     * Dispatcher ejecutado cuando llega
     * un paquete válido.
     */
    void handle_packet(
        const uint8_t* payload,
        uint8_t n
    ) override;

    /*
     * Envío de paquete binario por UART.
     */
    void send_impl(
        const uint8_t* buf,
        uint8_t n
    ) override;

    /*
     * Enviar ACK de configuración hacia Jetson.
     *
     * OPCODE 'L'
     */
    void send_config_ack(
        uint8_t subcommand,
        uint8_t body,
        uint8_t status,
        uint32_t value1,
        uint32_t value2
    );


};

#endif /* INC_INTERFACE_H_ */

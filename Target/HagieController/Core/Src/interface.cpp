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
#include "control_types.h"
#include "imu_types.h"

extern volatile ImuState_t imu_state;
extern UART_HandleTypeDef huart3;
extern volatile uint32_t jetson_heartbeat_count;
extern void setBodyValveCommand(uint8_t body, int16_t command);
extern volatile uint16_t target_height_mm[6];
extern volatile float encoder_height_mm[6];
extern int16_t getBodyValveCommand(uint8_t body);
extern volatile uint32_t axiomatic_rx_dropped;
extern volatile bool jetson_connection_ok;
extern volatile uint32_t jetson_uart_error_count;
extern volatile BodyControlMode_t body_control_mode[6];


interface::interface()
{
}


void interface::serial_read_command() {

	for (size_t i = 0; i < 12; i++) {
		feed(cadena[i]);

	}
}

void interface::send_imu_state()
{
    uint8_t *payload = get_payload_buffer();

    payload[0] = 'I';
    payload[1] = imu_state.valid ? 1 : 0;

    int16_t values[9] =
    {
        static_cast<int16_t>(imu_state.roll_deg * 100.0f),
        static_cast<int16_t>(imu_state.pitch_deg * 100.0f),
        static_cast<int16_t>(imu_state.yaw_deg * 100.0f),

        static_cast<int16_t>(imu_state.gyro_x_dps * 100.0f),
        static_cast<int16_t>(imu_state.gyro_y_dps * 100.0f),
        static_cast<int16_t>(imu_state.gyro_z_dps * 100.0f),

        static_cast<int16_t>(imu_state.accel_x_mps2 * 1000.0f),
        static_cast<int16_t>(imu_state.accel_y_mps2 * 1000.0f),
        static_cast<int16_t>(imu_state.accel_z_mps2 * 1000.0f)
    };

    for (uint8_t i = 0; i < 9; i++)
    {
        uint16_t raw =
            static_cast<uint16_t>(values[i]);

        uint8_t index =
            2 + (i * 2);

        payload[index] =
            static_cast<uint8_t>(
                (raw >> 8) & 0xFF
            );

        payload[index + 1] =
            static_cast<uint8_t>(
                raw & 0xFF
            );
    }

    /*
     * Payload:
     *
     * [0]      = 'I'
     * [1]      = valid
     *
     * [2..3]   = roll       x100
     * [4..5]   = pitch      x100
     * [6..7]   = yaw        x100
     *
     * [8..9]   = gyro_x     x100
     * [10..11] = gyro_y     x100
     * [12..13] = gyro_z     x100
     *
     * [14..15] = accel_x    x1000
     * [16..17] = accel_y    x1000
     * [18..19] = accel_z    x1000
     */

    send(20);
}

void interface::handle_packet(
    const uint8_t *payload,
    uint8_t n)
{
    /*
     * Protección ante paquete vacío.
     */
    if (payload == nullptr || n == 0)
    {
        return;
    }

    /*
     * Primer byte:
     * opcode del comando.
     */
    uint8_t opcode = payload[0];

    switch (opcode)
    {
		case 'A':
		{
			jetson_heartbeat_count++;
			break;
		}

		case 'B':
		{
		    if (n != 4)
		    {
		        break;
		    }

		    uint8_t body = payload[1];

		    if (body >= BODY_COUNT)
		    {
		        break;
		    }

		    int16_t command =
		        static_cast<int16_t>(
		            (static_cast<uint16_t>(payload[2]) << 8) |
		             static_cast<uint16_t>(payload[3])
		        );

		    /*
		     * Un comando directo de válvula
		     * coloca ese cuerpo en modo manual.
		     */
		    body_control_mode[body] = BODY_CONTROL_MANUAL;

		    setBodyValveCommand(body, command);

		    break;
		}

		case 'C':
		{
		    if (n != 1)
		    {
		        break;
		    }

		    for (uint8_t body = 0; body < BODY_COUNT; body++)
		    {
		        body_control_mode[body] = BODY_CONTROL_MANUAL;
		        setBodyValveCommand(body, 0);
		    }

		    break;
		}
        /*
         * OPCODE 'D'
         *
         * Altura objetivo de un cuerpo.
         *
         * Payload:
         * [0] = 'D'
         * [1] = cuerpo 0..5
         * [2] = altura MSB
         * [3] = altura LSB
         */
        case 'D':
        {
            if (n != 4)
            {
                break;
            }

            uint8_t body = payload[1];

            if (body >= BODY_COUNT)
            {
                break;
            }

            uint16_t heightMm =
                static_cast<uint16_t>(
                    (static_cast<uint16_t>(payload[2]) << 8) |
                     static_cast<uint16_t>(payload[3])
                );

            /*
             * Segunda barrera de seguridad.
             *
             * Aunque la Jetson mande una consigna incorrecta,
             * la STM32 no acepta alturas fuera del recorrido
             * permitido para ese cuerpo.
             */
            if (heightMm < BODY_MIN_HEIGHT_MM[body] ||
                heightMm > BODY_MAX_HEIGHT_MM[body])
            {
                /*
                 * Consigna inválida:
                 * detener el cuerpo y salir de AUTO.
                 */
                body_control_mode[body] = BODY_CONTROL_MANUAL;
                setBodyValveCommand(body, 0);

                break;
            }

            /*
             * Consigna válida.
             */
            target_height_mm[body] = heightMm;
            body_control_mode[body] = BODY_CONTROL_AUTO;

            break;

            break;
        }

        default:
        {
            break;
        }
    }
}

void interface::send_encoder_state()
{
    /*
     * Payload:
     *
     * [0]     = 'E'
     *
     * [1..2]  = encoder 1
     * [3..4]  = encoder 2
     * [5..6]  = encoder 3
     * [7..8]  = encoder 4
     * [9..10] = encoder 5
     * [11..12]= encoder 6
     *
     * Alturas expresadas en milímetros
     * como uint16_t, MSB primero.
     */

    uint8_t *payload = get_payload_buffer();

    payload[0] = 'E';

    for (uint8_t i = 0; i < 6; i++)
    {
        float height = encoder_height_mm[i];

        /*
         * Protección básica.
         */
        if (height < 0.0f)
        {
            height = 0.0f;
        }
        else if (height > 65535.0f)
        {
            height = 65535.0f;
        }

        uint16_t heightMm =
            static_cast<uint16_t>(height + 0.5f);

        uint8_t index = 1 + (i * 2);

        payload[index] =
            static_cast<uint8_t>(
                (heightMm >> 8) & 0xFF
            );

        payload[index + 1] =
            static_cast<uint8_t>(
                heightMm & 0xFF
            );
    }

    /*
     * 1 byte opcode +
     * 6 encoders x 2 bytes = 13 bytes.
     *
     * send() agrega automáticamente:
     * PKT!, LEN, CRC16 y '\n'
     */
    send(13);
}

void interface::send_valve_state()
{
    /*
     * OPCODE 'F'
     *
     * Payload:
     *
     * [0]     = 'F'
     *
     * [1..2]  = comando cuerpo 1
     * [3..4]  = comando cuerpo 2
     * [5..6]  = comando cuerpo 3
     * [7..8]  = comando cuerpo 4
     * [9..10] = comando cuerpo 5
     * [11..12]= comando cuerpo 6
     *
     * Cada comando es int16_t:
     *
     * -1000 = bajar máximo
     *     0 = detenido
     * +1000 = subir máximo
     */

    uint8_t *payload = get_payload_buffer();

    payload[0] = 'F';

    for (uint8_t body = 0; body < 6; body++)
    {
        int16_t command =
            getBodyValveCommand(body);

        /*
         * Convertimos manteniendo exactamente
         * los 16 bits del int16_t.
         */
        uint16_t raw =
            static_cast<uint16_t>(command);

        uint8_t index = 1 + (body * 2);

        payload[index] =
            static_cast<uint8_t>(
                (raw >> 8) & 0xFF
            );

        payload[index + 1] =
            static_cast<uint8_t>(
                raw & 0xFF
            );
    }

    /*
     * 1 byte de opcode +
     * 6 comandos x 2 bytes = 13 bytes.
     */
    send(13);
}

void interface::send_stm32_state()
{
    /*
     * OPCODE 'H'
     *
     * Payload:
     *
     * [0]    = 'H'
     * [1]    = estado comunicación Jetson
     *          0 = no válida
     *          1 = válida
     *
     * [2..5] = uptime STM32 en ticks
     *          uint32_t, MSB primero
     *
     * [6]    = cantidad de encoders
     * [7]    = cantidad de cuerpos
     * [8]    = cantidad de módulos Axiomatic
     */

    uint8_t *payload = get_payload_buffer();

    payload[0] = 'H';

    payload[1] =
        jetson_connection_ok ? 1 : 0;

    uint32_t uptime =
        static_cast<uint32_t>(xTaskGetTickCount());

    payload[2] =
        static_cast<uint8_t>(
            (uptime >> 24) & 0xFF
        );

    payload[3] =
        static_cast<uint8_t>(
            (uptime >> 16) & 0xFF
        );

    payload[4] =
        static_cast<uint8_t>(
            (uptime >> 8) & 0xFF
        );

    payload[5] =
        static_cast<uint8_t>(
            uptime & 0xFF
        );

    payload[6] = 6;  // encoders
    payload[7] = 6;  // cuerpos
    payload[8] = 3;  // módulos Axiomatic

    send(9);
}

void interface::send_diagnostic_state()
{
    /*
     * OPCODE 'G'
     *
     * Payload:
     *
     * [0]     = 'G'
     *
     * [1..4]  = axiomatic_rx_dropped
     *           uint32_t, MSB primero
     *
     * [5]     = jetson_connection_ok
     *           0 = desconectada
     *           1 = conectada
     *
     * [6]     = cantidad de módulos
     *           Axiomatic configurados
     */

    uint8_t *payload = get_payload_buffer();

    payload[0] = 'G';

    uint32_t dropped = axiomatic_rx_dropped;

    payload[1] =
        static_cast<uint8_t>((dropped >> 24) & 0xFF);

    payload[2] =
        static_cast<uint8_t>((dropped >> 16) & 0xFF);

    payload[3] =
        static_cast<uint8_t>((dropped >> 8) & 0xFF);

    payload[4] =
        static_cast<uint8_t>(dropped & 0xFF);

    payload[5] =
        jetson_connection_ok ? 1 : 0;

    /*
     * Tenemos preparados 3 módulos Axiomatic.
     */
    payload[6] = 3;

    uint32_t uartErrors = jetson_uart_error_count;

    payload[7] =
        static_cast<uint8_t>((uartErrors >> 24) & 0xFF);

    payload[8] =
        static_cast<uint8_t>((uartErrors >> 16) & 0xFF);

    payload[9] =
        static_cast<uint8_t>((uartErrors >> 8) & 0xFF);

    payload[10] =
        static_cast<uint8_t>(uartErrors & 0xFF);

    send(11);
}

void interface::serial_feed_byte(uint8_t byte)
{
    feed(byte);
}

void interface::send_impl(const uint8_t *buf, uint8_t n)
{


    HAL_UART_Transmit(
        &huart3,
        const_cast<uint8_t*>(buf),
        static_cast<uint16_t>(n),
        HAL_MAX_DELAY
    );


}




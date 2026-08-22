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
#include "jetson_tx.h"
#include "fault_types.h"
#include "control_config.h"

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
extern volatile uint32_t jetson_tx_queue_dropped;
extern volatile uint32_t jetson_tx_dma_errors;
extern volatile uint32_t system_faults;
extern volatile uint32_t body_faults[6];
extern volatile uint32_t jetson_clear_fault_count;
extern volatile TickType_t target_last_update_tick[6];

volatile uint32_t jetson_target_rx_count[BODY_COUNT] = {0};
volatile uint32_t jetson_target_valid_count[BODY_COUNT] = {0};
enum ConfigAckStatus : uint8_t
{
    CONFIG_ACK_OK = 0,
    CONFIG_ACK_INVALID_LENGTH = 1,
    CONFIG_ACK_INVALID_BODY = 2,
    CONFIG_ACK_INVALID_VALUE = 3,
    CONFIG_ACK_UNKNOWN_SUBCOMMAND = 4
};


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
        static_cast<int16_t>(imu_state.gravity_deg * 100.0f),

        static_cast<int16_t>(imu_state.gyro_roll_dps * 100.0f),
        static_cast<int16_t>(imu_state.gyro_pitch_dps * 100.0f),
        static_cast<int16_t>(imu_state.gyro_yaw_dps * 100.0f),

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

	 [2..3]   = roll          x100
	 [4..5]   = pitch         x100
	 [6..7]   = gravity angle x100

     [8..9]   = gyro roll     x100
     [10..11] = gyro pitch    x100
	 [12..13] = gyro yaw      x100
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
            body_control_mode[body] =
                BODY_CONTROL_MANUAL;

            setBodyValveCommand(
                body,
                command
            );

            break;
        }

        case 'C':
        {
            if (n != 1)
            {
                break;
            }

            for (uint8_t body = 0;
                 body < BODY_COUNT;
                 body++)
            {
                body_control_mode[body] =
                    BODY_CONTROL_MANUAL;

                setBodyValveCommand(
                    body,
                    0
                );
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

            uint8_t body =
                payload[1];

            if (body >= BODY_COUNT)
            {
                break;
            }
            /*
             * El paquete D llegó correctamente hasta
             * el parser para este cuerpo.
             */
            jetson_target_rx_count[body]++;

            uint16_t heightMm =
                static_cast<uint16_t>(
                    (static_cast<uint16_t>(
                        payload[2]
                    ) << 8) |
                    static_cast<uint16_t>(
                        payload[3]
                    )
                );

            /*
             * Segunda barrera de seguridad.
             *
             * Aunque la Jetson mande una consigna incorrecta,
             * la STM32 no acepta alturas fuera del recorrido
             * permitido para ese cuerpo.
             */
            if (heightMm <
            		body_control_config.min_height_mm[body] ||
                heightMm >
                    body_control_config.max_height_mm[body])
            {
                /*
                 * Consigna inválida:
                 * detener el cuerpo y salir de AUTO.
                 */
                body_control_mode[body] =
                    BODY_CONTROL_MANUAL;

                setBodyValveCommand(
                    body,
                    0
                );

                break;
            }

            /*
             * El paquete D además pasó la validación.
             */
            jetson_target_valid_count[body]++;

            /*
             * Consigna válida.
             */
            target_height_mm[body] =
                heightMm;

            /*
             * Guardar cuándo llegó la última
             * consigna válida de este cuerpo.
             */
            target_last_update_tick[body] =
                xTaskGetTickCount();

            /*
             * Si este cuerpo había entrado en
             * TARGET_TIMEOUT, una nueva consigna
             * válida recupera automáticamente
             * esa condición.
             */
            body_faults[body] &=
                ~BODY_FAULT_TARGET_TIMEOUT;

            /*
             * Volver / mantener control automático.
             */
            body_control_mode[body] =
                BODY_CONTROL_AUTO;

            break;
        }


        /*
         * OPCODE 'J'
         *
         * Reconocer / borrar la falla
         * NO_MOVEMENT de un cuerpo.
         *
         * Payload:
         * [0] = 'J'
         * [1] = cuerpo 0..5
         */
        case 'J':
        {
            if (n != 2)
            {
                break;
            }

            uint8_t body =
                payload[1];

            if (body >= BODY_COUNT)
            {
                break;
            }

            /*
             * Borrar solamente NO_MOVEMENT.
             */
            body_faults[body] &=
                ~BODY_FAULT_NO_MOVEMENT;

            /*
             * Después de reconocer la falla,
             * dejar el cuerpo detenido y en MANUAL.
             */
            body_control_mode[body] =
                BODY_CONTROL_MANUAL;

            setBodyValveCommand(
                body,
                0
            );

            break;
        }

        /*
         * ========================================================
         * OPCODE 'K'
         *
         * Configuración runtime desde Jetson.
         *
         * payload[0] = 'K'
         * payload[1] = subcomando
         * ========================================================
         */
        case 'K':
        {
            /*
             * Si ni siquiera llegó el subcomando,
             * no podemos identificar qué K responder.
             */
            if (n < 2)
            {
                send_config_ack(
                    0x00,
                    0xFF,
                    CONFIG_ACK_INVALID_LENGTH,
                    0,
                    0
                );

                break;
            }

            uint8_t subcommand =
                payload[1];


            /*
             * ========================================================
             * K 0x01
             * Límites mínimo / máximo de un cuerpo
             * ========================================================
             */
            if (subcommand == 0x01)
            {
                if (n != 7)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_LENGTH,
                        0,
                        0
                    );

                    break;
                }

                uint8_t body =
                    payload[2];

                if (body >= BODY_COUNT)
                {
                    send_config_ack(
                        subcommand,
                        body,
                        CONFIG_ACK_INVALID_BODY,
                        0,
                        0
                    );

                    break;
                }

                uint16_t minHeight =
                    static_cast<uint16_t>(
                        (static_cast<uint16_t>(
                            payload[3]
                        ) << 8) |
                        static_cast<uint16_t>(
                            payload[4]
                        )
                    );

                uint16_t maxHeight =
                    static_cast<uint16_t>(
                        (static_cast<uint16_t>(
                            payload[5]
                        ) << 8) |
                        static_cast<uint16_t>(
                            payload[6]
                        )
                    );

                if (minHeight >= maxHeight ||
                    maxHeight > 2000)
                {
                    send_config_ack(
                        subcommand,
                        body,
                        CONFIG_ACK_INVALID_VALUE,
                        minHeight,
                        maxHeight
                    );

                    break;
                }

                body_control_config.min_height_mm[body] =
                    minHeight;

                body_control_config.max_height_mm[body] =
                    maxHeight;

                /*
                 * Devolver exactamente los valores
                 * que quedaron aplicados.
                 */
                send_config_ack(
                    subcommand,
                    body,
                    CONFIG_ACK_OK,
                    body_control_config.min_height_mm[body],
                    body_control_config.max_height_mm[body]
                );

                break;
            }


            /*
             * ========================================================
             * K 0x02
             * Umbral de comando de movimiento
             * ========================================================
             */
            if (subcommand == 0x02)
            {
                if (n != 4)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_LENGTH,
                        0,
                        0
                    );

                    break;
                }

                uint16_t threshold =
                    static_cast<uint16_t>(
                        (static_cast<uint16_t>(
                            payload[2]
                        ) << 8) |
                        static_cast<uint16_t>(
                            payload[3]
                        )
                    );

                if (threshold == 0 ||
                    threshold > 1000)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_VALUE,
                        threshold,
                        0
                    );

                    break;
                }

                body_control_config.move_command_threshold =
                    static_cast<int16_t>(
                        threshold
                    );

                send_config_ack(
                    subcommand,
                    0xFF,
                    CONFIG_ACK_OK,
                    static_cast<uint32_t>(
                        body_control_config.move_command_threshold
                    ),
                    0
                );

                break;
            }


            /*
             * ========================================================
             * K 0x03
             * Movimiento mínimo
             *
             * value1 del ACK queda expresado x100.
             * ========================================================
             */
            if (subcommand == 0x03)
            {
                if (n != 4)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_LENGTH,
                        0,
                        0
                    );

                    break;
                }

                uint16_t rawMovement =
                    static_cast<uint16_t>(
                        (static_cast<uint16_t>(
                            payload[2]
                        ) << 8) |
                        static_cast<uint16_t>(
                            payload[3]
                        )
                    );

                if (rawMovement == 0)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_VALUE,
                        rawMovement,
                        0
                    );

                    break;
                }

                body_control_config.min_body_movement_mm =
                    static_cast<float>(
                        rawMovement
                    ) / 100.0f;

                /*
                 * Devolvemos el valor entero original x100
                 * para evitar diferencias de float.
                 */
                send_config_ack(
                    subcommand,
                    0xFF,
                    CONFIG_ACK_OK,
                    rawMovement,
                    0
                );

                break;
            }


            /*
             * ========================================================
             * K 0x04
             * Timeout NO_MOVEMENT
             * ========================================================
             */
            if (subcommand == 0x04)
            {
                if (n != 6)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_LENGTH,
                        0,
                        0
                    );

                    break;
                }

                uint32_t timeoutMs =
                    (static_cast<uint32_t>(
                        payload[2]
                    ) << 24) |
                    (static_cast<uint32_t>(
                        payload[3]
                    ) << 16) |
                    (static_cast<uint32_t>(
                        payload[4]
                    ) << 8) |
                    static_cast<uint32_t>(
                        payload[5]
                    );

                if (timeoutMs < 100 ||
                    timeoutMs > 60000)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_VALUE,
                        timeoutMs,
                        0
                    );

                    break;
                }

                body_control_config.no_movement_timeout_ms =
                    timeoutMs;

                send_config_ack(
                    subcommand,
                    0xFF,
                    CONFIG_ACK_OK,
                    body_control_config.no_movement_timeout_ms,
                    0
                );

                break;
            }


            /*
             * ========================================================
             * K 0x05
             * Timeout consigna AUTO
             * ========================================================
             */
            if (subcommand == 0x05)
            {
                if (n != 6)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_LENGTH,
                        0,
                        0
                    );

                    break;
                }

                uint32_t timeoutMs =
                    (static_cast<uint32_t>(
                        payload[2]
                    ) << 24) |
                    (static_cast<uint32_t>(
                        payload[3]
                    ) << 16) |
                    (static_cast<uint32_t>(
                        payload[4]
                    ) << 8) |
                    static_cast<uint32_t>(
                        payload[5]
                    );

                if (timeoutMs < 100 ||
                    timeoutMs > 60000)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_VALUE,
                        timeoutMs,
                        0
                    );

                    break;
                }

                body_control_config.target_timeout_ms =
                    timeoutMs;

                send_config_ack(
                    subcommand,
                    0xFF,
                    CONFIG_ACK_OK,
                    body_control_config.target_timeout_ms,
                    0
                );

                break;
            }


            /*
             * ========================================================
             * K 0x06
             * Sentido encoder
             * ========================================================
             */
            if (subcommand == 0x06)
            {
                if (n != 4)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_LENGTH,
                        0,
                        0
                    );

                    break;
                }

                uint8_t body =
                    payload[2];

                uint8_t direction =
                    payload[3];

                if (body >= BODY_COUNT)
                {
                    send_config_ack(
                        subcommand,
                        body,
                        CONFIG_ACK_INVALID_BODY,
                        direction,
                        0
                    );

                    break;
                }

                if (direction > 1)
                {
                    send_config_ack(
                        subcommand,
                        body,
                        CONFIG_ACK_INVALID_VALUE,
                        direction,
                        0
                    );

                    break;
                }

                body_control_config.encoder_direction[body] =
                    (direction == 0)
                        ? 1
                        : -1;

                /*
                 * En el protocolo:
                 * 0 = normal
                 * 1 = invertido
                 *
                 * Por eso devolvemos direction,
                 * no el +1 / -1 interno.
                 */
                send_config_ack(
                    subcommand,
                    body,
                    CONFIG_ACK_OK,
                    direction,
                    0
                );

                break;
            }


            /*
             * ========================================================
             * K 0x07
             * Escala encoder
             *
             * value1 del ACK queda expresado x100000.
             * ========================================================
             */
            if (subcommand == 0x07)
            {
                if (n != 7)
                {
                    send_config_ack(
                        subcommand,
                        0xFF,
                        CONFIG_ACK_INVALID_LENGTH,
                        0,
                        0
                    );

                    break;
                }

                uint8_t body =
                    payload[2];

                if (body >= BODY_COUNT)
                {
                    send_config_ack(
                        subcommand,
                        body,
                        CONFIG_ACK_INVALID_BODY,
                        0,
                        0
                    );

                    break;
                }

                uint32_t rawScale =
                    (static_cast<uint32_t>(
                        payload[3]
                    ) << 24) |
                    (static_cast<uint32_t>(
                        payload[4]
                    ) << 16) |
                    (static_cast<uint32_t>(
                        payload[5]
                    ) << 8) |
                    static_cast<uint32_t>(
                        payload[6]
                    );

                if (rawScale == 0)
                {
                    send_config_ack(
                        subcommand,
                        body,
                        CONFIG_ACK_INVALID_VALUE,
                        rawScale,
                        0
                    );

                    break;
                }

                body_control_config
                    .encoder_scale_mm_per_pulse[body] =
                    static_cast<float>(
                        rawScale
                    ) / 100000.0f;

                send_config_ack(
                    subcommand,
                    body,
                    CONFIG_ACK_OK,
                    rawScale,
                    0
                );

                break;
            }


            /*
             * ========================================================
             * Subcomando desconocido
             * ========================================================
             */
            send_config_ack(
                subcommand,
                0xFF,
                CONFIG_ACK_UNKNOWN_SUBCOMMAND,
                0,
                0
            );

            break;
        }


        default:
        {
            break;
        }
    }
}

void interface::send_config_ack(
    uint8_t subcommand,
    uint8_t body,
    uint8_t status,
    uint32_t value1,
    uint32_t value2)
{
    uint8_t *payload =
        get_payload_buffer();

    /*
     * OPCODE 'L'
     *
     * [0]     = 'L'
     * [1]     = subcomando K original
     * [2]     = body
     *           0..5  = cuerpo
     *           0xFF  = parámetro global
     *
     * [3]     = status
     *
     * [4..7]  = value1 uint32_t MSB primero
     * [8..11] = value2 uint32_t MSB primero
     */

    payload[0] = 'L';
    payload[1] = subcommand;
    payload[2] = body;
    payload[3] = status;

    payload[4] =
        static_cast<uint8_t>(
            (value1 >> 24) & 0xFF
        );

    payload[5] =
        static_cast<uint8_t>(
            (value1 >> 16) & 0xFF
        );

    payload[6] =
        static_cast<uint8_t>(
            (value1 >> 8) & 0xFF
        );

    payload[7] =
        static_cast<uint8_t>(
            value1 & 0xFF
        );

    payload[8] =
        static_cast<uint8_t>(
            (value2 >> 24) & 0xFF
        );

    payload[9] =
        static_cast<uint8_t>(
            (value2 >> 16) & 0xFF
        );

    payload[10] =
        static_cast<uint8_t>(
            (value2 >> 8) & 0xFF
        );

    payload[11] =
        static_cast<uint8_t>(
            value2 & 0xFF
        );

    send(12);
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

    uint32_t txDropped = jetson_tx_queue_dropped;

    payload[11] =
        static_cast<uint8_t>((txDropped >> 24) & 0xFF);

    payload[12] =
        static_cast<uint8_t>((txDropped >> 16) & 0xFF);

    payload[13] =
        static_cast<uint8_t>((txDropped >> 8) & 0xFF);

    payload[14] =
        static_cast<uint8_t>(txDropped & 0xFF);


    uint32_t txDmaErrors = jetson_tx_dma_errors;

    payload[15] =
        static_cast<uint8_t>((txDmaErrors >> 24) & 0xFF);

    payload[16] =
        static_cast<uint8_t>((txDmaErrors >> 16) & 0xFF);

    payload[17] =
        static_cast<uint8_t>((txDmaErrors >> 8) & 0xFF);

    payload[18] =
        static_cast<uint8_t>(txDmaErrors & 0xFF);

    uint32_t systemFaults = system_faults;

    payload[19] =
        static_cast<uint8_t>((systemFaults >> 24) & 0xFF);
    payload[20] =
        static_cast<uint8_t>((systemFaults >> 16) & 0xFF);
    payload[21] =
        static_cast<uint8_t>((systemFaults >> 8) & 0xFF);
    payload[22] =
        static_cast<uint8_t>(systemFaults & 0xFF);


    for (uint8_t body = 0; body < 6; body++)
    {
        uint32_t faults = body_faults[body];

        uint8_t index =
            23 + (body * 4);

        payload[index] =
            static_cast<uint8_t>((faults >> 24) & 0xFF);

        payload[index + 1] =
            static_cast<uint8_t>((faults >> 16) & 0xFF);

        payload[index + 2] =
            static_cast<uint8_t>((faults >> 8) & 0xFF);

        payload[index + 3] =
            static_cast<uint8_t>(faults & 0xFF);
    }

    send(47);
}

void interface::serial_feed_byte(uint8_t byte)
{
    feed(byte);
}

void interface::send_impl(
    const uint8_t *buf,
    uint8_t n)
{
    if (buf == nullptr || n == 0)
    {
        return;
    }

    if (n > JETSON_TX_BUFFER_SIZE)
    {
        return;
    }

    JetsonTxMessage_t message;

    message.length = n;

    memcpy(
        message.data,
        buf,
        n
    );

    /*
     * No bloquear al productor.
     *
     * La tarea Task_jetson_serial_tx
     * se encargará de transmitir por DMA.
     */
    if (xQueueSend(
            JetsonTxQueueHandle,
            &message,
            0) != pdPASS)
    {
        jetson_tx_queue_dropped++;

        /*
         * La cola TX se llenó y se perdió
         * un paquete hacia la Jetson.
         */
        system_faults |= SYSTEM_FAULT_UART_TX;
    }
}




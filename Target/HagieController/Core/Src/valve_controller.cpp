#include "valve_controller.h"

ValveController::ValveController(
    CAN_HandleTypeDef *canHandle,
    uint8_t moduleAddress)
    : hcan(canHandle),
      address(moduleAddress),
      outputs{0, 0, 0, 0, 0, 0, 0, 0},
      txHeader{},
      txMailbox(0)
{
    /*
     * Este identificador es provisorio.
     *
     * El ID J1939 definitivo se configurará
     * cuando definamos los PGN y direcciones
     * en Axiomatic Electronic Assistant.
     */
    txHeader.IDE = CAN_ID_EXT;
    txHeader.ExtId = 0;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;
}

void ValveController::setOutput(
    uint8_t output,
    uint16_t command)
{
    if (output >= OUTPUT_COUNT)
    {
        return;
    }

    /*
     * Escala provisoria:
     *
     * 0    = salida desactivada
     * 1000 = orden máxima
     */
    if (command > 1000)
    {
        command = 1000;
    }

    outputs[output] = command;
}

void ValveController::disableOutput(uint8_t output)
{
    if (output >= OUTPUT_COUNT)
    {
        return;
    }

    outputs[output] = 0;
}

void ValveController::disableAll()
{
    for (uint8_t i = 0; i < OUTPUT_COUNT; i++)
    {
        outputs[i] = 0;
    }
}

uint16_t ValveController::getOutput(uint8_t output) const
{
    if (output >= OUTPUT_COUNT)
    {
        return 0;
    }

    return outputs[output];
}

uint8_t ValveController::getModuleAddress() const
{
    return address;
}

HAL_StatusTypeDef ValveController::send()
{
    /*
     * Formato propuesto para AX020800:
     *
     * Byte 0 -> OUT1
     * Byte 1 -> OUT2
     * Byte 2 -> OUT3
     * Byte 3 -> OUT4
     * Byte 4 -> OUT5
     * Byte 5 -> OUT6
     * Byte 6 -> OUT7
     * Byte 7 -> OUT8
     *
     * Escala:
     * 0..1000 (GUI) -> 0..100 (CAN)
     *
     * IMPORTANTE:
     * El mapeo debe verificarse en
     * Axiomatic Electronic Assistant.
     */

    uint8_t data[8] = {};

    for (uint8_t i = 0; i < OUTPUT_COUNT; i++)
    {
        uint16_t command = outputs[i];

        if (command > 1000)
        {
            command = 1000;
        }

        data[i] =
            static_cast<uint8_t>((command + 5) / 10);
    }

    /*
     * PGN 0xFF00.
     * SA 0x21: dirección prevista para STM32.
     *
     * El formato todavía no está validado
     * con el AX020800.
     */
    txHeader.ExtId = 0x18FF0021UL;

    /*
     * Seguridad: no transmitir comandos
     * hasta validar configuración y
     * habilitación del AX020800.
     */
    return HAL_CAN_AddTxMessage(
        hcan,
        &txHeader,
        data,
        &txMailbox
    );
}

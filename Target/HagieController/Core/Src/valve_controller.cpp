#include "valve_controller.h"

ValveController::ValveController(
    CAN_HandleTypeDef *canHandle,
    uint8_t moduleAddress)
    : hcan(canHandle),
      address(moduleAddress),
      outputs{0, 0, 0, 0},
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
    uint8_t data[8];

    /*
     * Dos bytes por salida:
     *
     * Bytes 0-1 → salida 1
     * Bytes 2-3 → salida 2
     * Bytes 4-5 → salida 3
     * Bytes 6-7 → salida 4
     *
     * Formato provisional little-endian.
     */
    for (uint8_t i = 0; i < OUTPUT_COUNT; i++)
    {
        data[i * 2] =
            static_cast<uint8_t>(outputs[i] & 0xFF);

        data[i * 2 + 1] =
            static_cast<uint8_t>((outputs[i] >> 8) & 0xFF);
    }

    /*
     * Identificador CAN provisional.
     *
     * No debe usarse con las válvulas hasta
     * reemplazarlo por el PGN J1939 real.
     */
    txHeader.ExtId =
        0x18FF0000UL |
        static_cast<uint32_t>(address);

    return HAL_CAN_AddTxMessage(
        hcan,
        &txHeader,
        data,
        &txMailbox
    );
}

#include "imu_can.h"


ImuCan::ImuCan(
    CAN_HandleTypeDef *canHandle,
    volatile ImuState_t *state)
    : hcan(canHandle),
      imuState(state)
{
}


static uint32_t extractJ1939Pgn(uint32_t canId)
{
    uint8_t dataPage =
        static_cast<uint8_t>(
            (canId >> 24) & 0x01
        );

    uint8_t pduFormat =
        static_cast<uint8_t>(
            (canId >> 16) & 0xFF
        );

    uint8_t pduSpecific =
        static_cast<uint8_t>(
            (canId >> 8) & 0xFF
        );

    uint32_t pgn;

    /*
     * PDU1:
     * PF < 240
     * PS representa dirección destino
     * y NO forma parte del PGN.
     */
    if (pduFormat < 240)
    {
        pgn =
            (static_cast<uint32_t>(dataPage) << 16) |
            (static_cast<uint32_t>(pduFormat) << 8);
    }
    else
    {
        /*
         * PDU2:
         * PF >= 240
         * PS sí forma parte del PGN.
         */
        pgn =
            (static_cast<uint32_t>(dataPage) << 16) |
            (static_cast<uint32_t>(pduFormat) << 8) |
             static_cast<uint32_t>(pduSpecific);
    }

    return pgn;
}


void ImuCan::processMessage(
    uint32_t canId,
    const uint8_t *data,
    uint8_t dlc)
{
    if (imuState == nullptr ||
        data == nullptr ||
        dlc == 0)
    {
        return;
    }

    uint32_t pgn =
        extractJ1939Pgn(canId);

    if (pgn != 61459U)
    {
        return;
    }

    /*
     * Dirección de origen J1939.
     * Es el byte menos significativo
     * del identificador extendido.
     */
    uint8_t sourceAddress =
        static_cast<uint8_t>(
            canId & 0xFF
        );

    /*
     * Por ahora solo la extraemos.
     * Más adelante la podemos guardar
     * para diagnóstico o filtrar una IMU
     * específica por dirección.
     */
    (void)sourceAddress;

    /*
     * Próximo paso:
     * decodificar el payload del AX060900
     * cuando tengamos la tabla exacta.
     */
}

#include "FreeRTOS.h"
#include "task.h"
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
        dlc < 4)
    {
        return;
    }

    uint32_t pgn = extractJ1939Pgn(canId);

    /*
     * AX060900
     * PGN 0xF013: Pitch y Roll
     */
    if (pgn != 0xF013U)
    {
        return;
    }

    /*
     * Valores de 16 bits sin signo.
     * Orden little-endian.
     */
    uint16_t rawPitch =
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);

    uint16_t rawRoll =
        static_cast<uint16_t>(data[2]) |
        (static_cast<uint16_t>(data[3]) << 8);

    /*
     * Resolución: 0.002 grados/bit
     * Offset: -64 grados
     */
    imuState->pitch_deg =
        static_cast<float>(rawPitch) * 0.002f - 64.0f;

    imuState->roll_deg =
        static_cast<float>(rawRoll) * 0.002f - 64.0f;

    imuState->timestamp_ms =
        xTaskGetTickCount();

    imuState->valid = true;
}

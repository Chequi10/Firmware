#ifndef INC_VALVE_CONTROLLER_H_
#define INC_VALVE_CONTROLLER_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

class ValveController
{
public:
    static constexpr uint8_t OUTPUT_COUNT = 4;

    ValveController(
        CAN_HandleTypeDef *canHandle,
        uint8_t moduleAddress
    );

    void setOutput(uint8_t output, uint16_t command);
    void disableOutput(uint8_t output);
    void disableAll();

    uint16_t getOutput(uint8_t output) const;
    uint8_t getModuleAddress() const;

    HAL_StatusTypeDef send();

private:
    CAN_HandleTypeDef *hcan;
    uint8_t address;

    uint16_t outputs[OUTPUT_COUNT];

    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
};

#endif /* INC_VALVE_CONTROLLER_H_ */

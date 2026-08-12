#ifndef IMU_CAN_H
#define IMU_CAN_H

#include "main.h"
#include "imu_types.h"

class ImuCan
{
public:
	ImuCan(
	    CAN_HandleTypeDef *canHandle,
	    volatile ImuState_t *state
	);

    void processMessage(
        uint32_t canId,
        const uint8_t *data,
        uint8_t dlc
    );

private:
    CAN_HandleTypeDef *hcan;
    volatile ImuState_t *imuState;
};

#endif /* IMU_CAN_H */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
#include <stdint.h>

class Encoder
{
public:
    Encoder(TIM_HandleTypeDef *timer, uint8_t timerBits);

    void start();
    void update();
    void reset();

    int32_t getDelta() const;
    int64_t getPosition() const;
    int8_t getDirection() const;

    void setCalibration(
        int64_t countsLow,
        int64_t countsHigh,
        float heightLowMm,
        float heightHighMm
    );

    float getHeightMm() const;
    bool isCalibrated() const;

private:
    TIM_HandleTypeDef *htim;
    uint8_t timerBits;

    uint32_t previousCounter;
    int32_t delta;
    int64_t position;
    int8_t direction;

    bool calibrated;

    int64_t countsLow;
    int64_t countsHigh;

    float heightLowMm;
    float heightHighMm;
    float millimetersPerCount;
};

#endif /* INC_ENCODER_H_ */

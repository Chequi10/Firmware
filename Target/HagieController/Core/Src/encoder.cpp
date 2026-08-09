#include "encoder.h"

Encoder::Encoder(TIM_HandleTypeDef *timer, uint8_t bits)
    : htim(timer),
      timerBits(bits),
      previousCounter(0),
      delta(0),
      position(0),
      direction(0),
      calibrated(false),
      countsLow(0),
      countsHigh(0),
      heightLowMm(0.0f),
      heightHighMm(0.0f),
      millimetersPerCount(0.0f)
{
}

void Encoder::start()
{
    __HAL_TIM_SET_COUNTER(htim, 0);

    if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }

    previousCounter = __HAL_TIM_GET_COUNTER(htim);
}

void Encoder::update()
{
    uint32_t currentCounter = __HAL_TIM_GET_COUNTER(htim);

    if (timerBits == 16)
    {
        delta = static_cast<int16_t>(
            static_cast<uint16_t>(currentCounter) -
            static_cast<uint16_t>(previousCounter)
        );
    }
    else
    {
        delta = static_cast<int32_t>(
            currentCounter - previousCounter
        );
    }

    previousCounter = currentCounter;
    position += delta;

    if (delta > 0)
    {
        direction = 1;
    }
    else if (delta < 0)
    {
        direction = -1;
    }
    else
    {
        direction = 0;
    }
}

void Encoder::reset()
{
    __HAL_TIM_SET_COUNTER(htim, 0);

    previousCounter = 0;
    delta = 0;
    position = 0;
    direction = 0;
}

int32_t Encoder::getDelta() const
{
    return delta;
}

int64_t Encoder::getPosition() const
{
    return position;
}

int8_t Encoder::getDirection() const
{
    return direction;
}

void Encoder::setCalibration(
    int64_t newCountsLow,
    int64_t newCountsHigh,
    float newHeightLowMm,
    float newHeightHighMm)
{
    countsLow = newCountsLow;
    countsHigh = newCountsHigh;

    heightLowMm = newHeightLowMm;
    heightHighMm = newHeightHighMm;

    int64_t countsDifference = countsHigh - countsLow;

    if (countsDifference == 0)
    {
        calibrated = false;
        millimetersPerCount = 0.0f;
        return;
    }

    millimetersPerCount =
        (heightHighMm - heightLowMm) /
        static_cast<float>(countsDifference);

    calibrated = true;
}

float Encoder::getHeightMm() const
{
    if (!calibrated)
    {
        return 0.0f;
    }

    return heightLowMm +
           static_cast<float>(position - countsLow) *
           millimetersPerCount;
}

bool Encoder::isCalibrated() const
{
    return calibrated;
}

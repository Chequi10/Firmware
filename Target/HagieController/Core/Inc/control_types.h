#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <stdint.h>

#define BODY_COUNT 6

static constexpr uint16_t BODY_MIN_HEIGHT_MM[BODY_COUNT] =
{
    50, 50, 50, 50, 50, 50
};

static constexpr uint16_t BODY_MAX_HEIGHT_MM[BODY_COUNT] =
{
    700, 700, 700, 700, 700, 700
};

typedef enum
{
    BODY_CONTROL_MANUAL = 0,
    BODY_CONTROL_AUTO   = 1
} BodyControlMode_t;

#endif

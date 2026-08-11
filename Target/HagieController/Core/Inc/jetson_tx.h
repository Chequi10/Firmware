#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

#define JETSON_TX_BUFFER_SIZE 128

typedef struct
{
    uint16_t length;
    uint8_t data[JETSON_TX_BUFFER_SIZE];

} JetsonTxMessage_t;

extern QueueHandle_t JetsonTxQueueHandle;

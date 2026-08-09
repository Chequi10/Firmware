#ifndef INC_JETSON_WATCHDOG_H_
#define INC_JETSON_WATCHDOG_H_

#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>

extern volatile TickType_t jetson_last_valid_packet_tick;
extern volatile bool jetson_connection_ok;

extern const TickType_t JETSON_WATCHDOG_TIMEOUT;

#endif /* INC_JETSON_WATCHDOG_H_ */

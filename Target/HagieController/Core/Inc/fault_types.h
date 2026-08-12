#ifndef FAULT_TYPES_H
#define FAULT_TYPES_H

#include <stdint.h>

/*
 * ============================================================
 * Fallas por cuerpo
 * ============================================================
 *
 * Cada bit representa una falla distinta.
 * Un cuerpo puede tener varias fallas al mismo tiempo.
 */

typedef enum
{
    BODY_FAULT_NONE            = 0,

    BODY_FAULT_ENCODER_TIMEOUT = (1UL << 0),
    BODY_FAULT_ENCODER_RANGE   = (1UL << 1),
    BODY_FAULT_NO_MOVEMENT     = (1UL << 2),

    BODY_FAULT_MIN_LIMIT       = (1UL << 3),
    BODY_FAULT_MAX_LIMIT       = (1UL << 4),

    BODY_FAULT_TARGET_TIMEOUT  = (1UL << 5),

    BODY_FAULT_VALVE_ERROR     = (1UL << 6)

} BodyFault_t;


/*
 * ============================================================
 * Fallas globales del sistema
 * ============================================================
 */

typedef enum
{
    SYSTEM_FAULT_NONE           = 0,

    SYSTEM_FAULT_JETSON_TIMEOUT = (1UL << 0),
    SYSTEM_FAULT_IMU_TIMEOUT    = (1UL << 1),

    SYSTEM_FAULT_UART_RX        = (1UL << 2),
    SYSTEM_FAULT_UART_TX        = (1UL << 3),

    SYSTEM_FAULT_CAN            = (1UL << 4)

} SystemFault_t;

#endif /* FAULT_TYPES_H */

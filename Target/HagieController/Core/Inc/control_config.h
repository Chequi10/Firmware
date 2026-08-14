#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include <stdint.h>

#include "control_types.h"

/*
 * ============================================================
 * Configuración runtime del control de los cuerpos
 * ============================================================
 *
 * Estos valores pueden ser modificados desde la Jetson
 * mediante el opcode 'K'.
 *
 * Los valores iniciales son los mismos que actualmente
 * están fijos en el firmware.
 */

typedef struct
{
    /*
     * Límites individuales de cada cuerpo.
     */
    uint16_t min_height_mm[BODY_COUNT];
    uint16_t max_height_mm[BODY_COUNT];

    /*
     * Umbral de comando a partir del cual consideramos
     * que se está solicitando movimiento.
     */
    int16_t move_command_threshold;

    /*
     * Movimiento mínimo que debe observarse para
     * considerar que el cuerpo realmente se movió.
     */
    float min_body_movement_mm;

    /*
     * Timeouts expresados en milisegundos.
     *
     * Se guardan en ms y se convierten a ticks de
     * FreeRTOS solamente cuando se utilizan.
     */
    uint32_t no_movement_timeout_ms;
    uint32_t target_timeout_ms;

    /*
     * Preparados para la configuración futura
     * del encoder mediante opcode K.
     */
    int8_t encoder_direction[BODY_COUNT];

    /*
     * Escala en mm/pulso.
     */
    float encoder_scale_mm_per_pulse[BODY_COUNT];

} BodyControlConfig;


/*
 * Única configuración runtime utilizada por STM32.
 */
extern BodyControlConfig body_control_config;


/*
 * Restaura todos los valores por defecto.
 */
void BodyControlConfig_InitDefaults(void);

#endif

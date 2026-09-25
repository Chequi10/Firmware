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

#define MAX_ENCODER_CALIBRATION_POINTS 15

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
     * ========================================================
     * Gestión de demanda hidráulica
     * ========================================================
     *
     * hydraulic_management_mode:
     *
     * 0 = NORMAL
     *     Comportamiento original.
     *     No se modifica ningún comando de válvula.
     *
     * 1 = BÁSICO
     *     Limita la cantidad de cuerpos que pueden
     *     realizar simultáneamente una demanda fuerte.
     */
    uint8_t hydraulic_management_mode;


    /*
     * A partir de este valor absoluto de comando
     * consideramos que existe una demanda hidráulica fuerte.
     *
     * Rango del comando:
     *
     * 0 ... 1000
     */
    int16_t hydraulic_high_command_threshold;


    /*
     * Cantidad máxima de cuerpos que pueden mantener
     * simultáneamente una demanda fuerte sin reducción.
     */
    uint8_t hydraulic_max_high_demand_bodies;


    /*
     * Factor aplicado a las demandas fuertes secundarias.
     *
     * Se almacena como porcentaje entero para evitar
     * problemas de representación en el protocolo.
     *
     * Ejemplo:
     *
     * 40 = 40 %
     * 100 = sin reducción
     */
    uint8_t hydraulic_secondary_percent;

    /*
     * ========================================================
     * Sintonía control de altura
     * ========================================================
     *
     * Kp, Ki y Kd del controlador.
     *
     * Inicialmente:
     * Kp = 5.0
     * Ki = 0.0
     * Kd = 0.0
     *
     * Con Ki y Kd en cero, el comportamiento sigue siendo
     * proporcional como hasta ahora.
     */
    float height_control_kp;
    float height_control_ki;
    float height_control_kd;

    /*
     * Banda muerta alrededor del objetivo.
     */
    float height_control_deadband_mm;

    /*
     * ========================================================
     * Compensación hidráulica individual por cuerpo
     * ========================================================
     *
     * Porcentaje aplicado al comando PID final.
     *
     * Ejemplo:
     *   0   = sin compensación
     *  +10  = aumenta 10 %
     *  -10  = reduce 10 %
     *
     * Se separa subida y bajada porque el comportamiento
     * hidráulico puede ser distinto en cada sentido.
     */
    int8_t height_up_compensation_percent[BODY_COUNT];
    int8_t height_down_compensation_percent[BODY_COUNT];

    /*
     * Preparados para la configuración futura
     * del encoder mediante opcode K.
     */
    int8_t encoder_direction[BODY_COUNT];

    /*
     * Escala en mm/pulso.
     */
    float encoder_scale_mm_per_pulse[BODY_COUNT];
    /*
     * Tabla de calibración:
     * posición relativa del encoder -> altura real.
     */
    uint8_t encoder_calibration_count[BODY_COUNT];

    int32_t encoder_calibration_position
        [BODY_COUNT]
        [MAX_ENCODER_CALIBRATION_POINTS];

    uint16_t encoder_calibration_height_mm
        [BODY_COUNT]
        [MAX_ENCODER_CALIBRATION_POINTS];

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

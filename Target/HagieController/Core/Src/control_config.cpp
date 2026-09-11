#include "control_config.h"


BodyControlConfig body_control_config;


void BodyControlConfig_InitDefaults(void)
{
    for (uint8_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        body_control_config.min_height_mm[body] =
            50;

        body_control_config.max_height_mm[body] =
            700;

        body_control_config.encoder_direction[body] =
            1;

        body_control_config.encoder_scale_mm_per_pulse[body] =
            1.0f;
    }


    body_control_config.move_command_threshold =
        100;

    body_control_config.min_body_movement_mm =
        2.0f;

    body_control_config.no_movement_timeout_ms =
        1000;

    body_control_config.target_timeout_ms =
        1000;

    /*
     * ========================================================
     * Gestión hidráulica
     * ========================================================
     *
     * Por seguridad arrancamos en NORMAL.
     *
     * Esto conserva exactamente el comportamiento
     * original del control.
     */
    body_control_config.hydraulic_management_mode =
        0;

    body_control_config.hydraulic_high_command_threshold =
        700;

    body_control_config.hydraulic_max_high_demand_bodies =
        2;

    body_control_config.hydraulic_secondary_percent =
        40;
}

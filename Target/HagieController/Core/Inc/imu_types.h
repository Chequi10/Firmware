#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    /*
     * Ángulos entregados por AX060900.
     */
    float roll_deg;
    float pitch_deg;
    float gravity_deg;

    /*
     * Velocidades angulares.
     */
    float gyro_roll_dps;
    float gyro_pitch_dps;
    float gyro_yaw_dps;

    /*
     * Aceleraciones en los tres ejes.
     */
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;

    /*
     * Estado de validez.
     */
    bool valid;

    /*
     * Momento de la última actualización.
     */
    uint32_t timestamp_ms;

} ImuState_t;

#endif /* IMU_TYPES_H */

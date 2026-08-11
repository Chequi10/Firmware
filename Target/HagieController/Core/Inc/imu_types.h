#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    /*
     * Orientación
     */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    /*
     * Velocidad angular
     */
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    /*
     * Aceleración lineal
     */
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;

    /*
     * Estado
     */
    bool valid;

    /*
     * Momento de la última actualización.
     * Lo vamos a usar después para detectar
     * si la IMU dejó de entregar datos.
     */
    uint32_t timestamp_ms;

} ImuState_t;

#endif /* IMU_TYPES_H */

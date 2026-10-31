/** @file drv_gy511.h
 *
 * @brief GY-511 (LSM303DLHC) accelerometer on I2C0, through the kernel I2C
 * driver "iica". Call only from the IMU task, the owner of I2C0.
 */

#ifndef DRV_GY511_H
#define DRV_GY511_H

#include <stdint.h>
#include "../app.h"

ER drv_gy511_init(void); /* 100 Hz, +-2 g, high resolution. */
ER drv_gy511_read(int32_t * p_x_mg, int32_t * p_y_mg, int32_t * p_z_mg);

#endif /* DRV_GY511_H */

/*** end of file ***/

/*
 *      drv_gy511.h
 *      GY-511 (LSM303DLHC) accelerometer on I2C0, through the kernel I2C
 *      driver "iica".  Call only from imu_task, the owner of I2C0.
 */

#ifndef DRV_GY511_H
#define DRV_GY511_H

#include "../app.h"

IMPORT ER gy511_init(void);                /* 100 Hz, +-2 g, high res */
IMPORT ER gy511_read(W * x, W * y, W * z); /* acceleration in mg */

#endif /* DRV_GY511_H */

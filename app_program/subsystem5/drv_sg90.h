/** @file drv_sg90.h
 *
 * @brief SG90 servo that pans the ultrasonic sensor, on GP12 (PWM slice
 * 6 A). Call only from the scan task.
 */

#ifndef DRV_SG90_H
#define DRV_SG90_H

#include <stdint.h>

void drv_sg90_init(void);
void drv_sg90_set(uint32_t angle_deg);

#endif /* DRV_SG90_H */

/*** end of file ***/

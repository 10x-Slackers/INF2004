/** @file drv_hcsr04.h
 *
 * @brief HC-SR04 ultrasonic range sensor: TRIG on GP0, ECHO (level
 * shifted) on GP1. Call only from the scan task, which owns the sensor.
 */

#ifndef DRV_HCSR04_H
#define DRV_HCSR04_H

#include <stdbool.h>
#include <stdint.h>

bool drv_hcsr04_init(void);
bool drv_hcsr04_measure(uint32_t * p_mm);

#endif /* DRV_HCSR04_H */

/*** end of file ***/

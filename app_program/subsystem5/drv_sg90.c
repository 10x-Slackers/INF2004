/** @file drv_sg90.c
 *
 * @brief SG90 servo on GP12, PWM slice 6 A at 50 Hz. usermain() has
 * already taken the PWM block out of reset. GP13 shares the slice, so it
 * would share this frequency.
 */

#include "drv_sg90.h"
#include "scan_profile.h"
#include "../app.h"

#define DRV_SG90_PIN    12u
#define DRV_SG90_DIV    (125u << 4) /* 8.4 fixed point: 1 MHz counter. */
#define DRV_SG90_TOP    19999u      /* 20 ms period. */
#define DRV_SG90_MIN_US 1200u       /* Pulse at 45 degrees (right). */
#define DRV_SG90_MID_US 1700u       /* At 90 degrees (ahead). */
#define DRV_SG90_MAX_US 2200u       /* At 135 degrees (left). */
#define DRV_SG90_MID    90u
#define DRV_SG90_HALF   45u /* Degrees from either end to the middle. */

/*!
 * @brief Start the 50 Hz PWM with the servo pointing ahead.
 */
void drv_sg90_init(void)
{
    out_w(PWM_CH6_BASE + PWM_CHx_DIV, DRV_SG90_DIV);
    (void)pwm_set_wrap(DRV_SG90_PIN, DRV_SG90_TOP);
    drv_sg90_set(DRV_SG90_MID);
    (void)pwm_set_pin(DRV_SG90_PIN);
    (void)pwm_set_enabled(DRV_SG90_PIN, TRUE);
}

/*!
 * @brief Point the servo, 45..135 degrees. Each half of the travel has
 * its own pulse range, so the middle can be calibrated on its own.
 */
void drv_sg90_set(uint32_t angle_deg)
{
    uint32_t pulse_us = DRV_SG90_MID_US;

    if (angle_deg < DRV_SG90_MID)
    {
        pulse_us = DRV_SG90_MIN_US + (((angle_deg - SCAN_PROFILE_MIN_DEG) *
                                       (DRV_SG90_MID_US - DRV_SG90_MIN_US)) /
                                      DRV_SG90_HALF);
    }
    else
    {
        pulse_us = DRV_SG90_MID_US + (((angle_deg - DRV_SG90_MID) *
                                       (DRV_SG90_MAX_US - DRV_SG90_MID_US)) /
                                      DRV_SG90_HALF);
    }
    (void)pwm_set_cc(DRV_SG90_PIN, pulse_us);
}

/*** end of file ***/

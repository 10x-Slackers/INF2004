/** @file drv_motion.h
 *
 * @brief Motor PWM and wheel encoder driver for the Robo Pico.
 *
 * Left wheel: motor M1 (GP8, GP9), encoder GP16, GP17 (Grove 4).
 * Right wheel: motor M2 (GP10, GP11), encoder GP2, GP3 (Grove 2).
 * The host test replaces this module with a simulated robot.
 */

#ifndef DRV_MOTION_H
#define DRV_MOTION_H

#include <stdbool.h>
#include <stdint.h>

/* Encoder counts, written only by the encoder edge callback. All fields
 * wrap. */
typedef struct
{
    uint16_t left;
    uint16_t right;
    uint8_t skipped; /* Times a phase was missed between two reads. */
} drv_motion_counts_t;

bool drv_motion_init(void);
void drv_motion_write(int32_t left_duty, int32_t right_duty);
void drv_motion_read(drv_motion_counts_t * p_counts);
uint32_t drv_motion_time_us(void);
void drv_motion_lock(void);
void drv_motion_unlock(void);

#endif /* DRV_MOTION_H */

/*** end of file ***/

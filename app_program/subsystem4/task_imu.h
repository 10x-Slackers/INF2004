/** @file task_imu.h
 *
 * @brief What only subsystem 4 uses of the IMU task: its thresholds, the
 * raw reading and the task entry. Other subsystems use subsystem4.h.
 */

#ifndef TASK_IMU_H
#define TASK_IMU_H

#include "subsystem4.h"

#define S4_IMU_MS 10u  /* Sample period, from a cyclic handler. */
#define S4_CAL_N  200u /* Start-up calibration samples (2 s, car still). */
#define S4_CAL_MG 100  /* A sample this far from the mean restarts it. */

#define S4_TRACK_MM    114  /* Distance between the drive wheel centres. */
#define S4_HUMP_ON_MM  3    /* Right wheel this high: on a hump... */
#define S4_HUMP_MIN_MS 200u /* ...for at least this long. */
#define S4_HUMP_OFF_MM 1    /* Back below this: the hump is over. */
#define S4_ACCEL_MG    50   /* Forward push beyond this: accelerating... */
#define S4_ACCEL_MS    100u /* ...for at least this long. */

#define S4_BUZZER_PIN 22u   /* Robo Pico buzzer; 18 on the Maker Pi Pico. */
#define S4_BEEP_HZ    4000u /* Beep when calibration is done. */
#define S4_BEEP_MS    200u
#define S4_ALARM_MS   1000u /* Beep on a fault. */

#define S4_TILT_MAX  250  /* Pitch or roll beyond 25.0 degrees... */
#define S4_TILT_MS   500u /* ...for this long: excessive tilt. */
#define S4_UPSIDE_MS 200u /* Z below 0 mg this long: upside down. */
#define S4_LIFT_MG   300  /* Z this far above 1 g... */
#define S4_LIFT_MS   50u  /* ...this long: lifted (quick lifts only). */
#define S4_FAIL_MS   50u  /* Reads failing this long: sensor fault... */
#define S4_FROZEN_MS 500u /* ...or no axis changing for this long. */

#define S4_AXIS_MAX_MG 2047 /* Largest value an s4_accel_t axis holds. */

/* Latest calibrated reading, packed to its range:
 * each axis is -2048..2047 mg (the sensor's 12-bit range at +-2 g); count
 * is a wrapping 8-bit sample counter. 36 + 8 bits, stored in 8 bytes. */
typedef struct
{
    signed int x : 12;
    signed int y : 12;
    unsigned int count : 8;
    signed int z : 12;
} s4_accel_t;

extern ID gh_s4_mutex; /* Guards the reading, tilt, humps, motion. */

void s4_get_accel(s4_accel_t * p_accel); /* Copied under gh_s4_mutex. */
void s4_imu_task(INT stacd, void * p_exinf);

#endif /* TASK_IMU_H */

/*** end of file ***/

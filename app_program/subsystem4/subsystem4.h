/** @file subsystem4.h
 *
 * @brief Subsystem 4 (IMU and terrain). Other subsystems read the latest
 * tilt, humps and motion class with the s4_get_*() calls. Faults: app_fault.h.
 *
 * The code follows BARR-C:2018, with the team's deviations listed in app.h
 * and these:
 * - 5.3.a: x, y, z, pitch, roll and the IMU task's filter state are signed
 *   bit-fields, packed to the minimum bits (docs/DATA_REPRESENTATION.md).
 * - 7.1.e: struct fields x, y, z and on keep their short names.
 * - 4.2.a, 4.3.c, 6.2.d: task_imu.c has no header of its own; this file
 *   declares its functions and gh_s4_mutex, shared with subsystem4.c.
 *
 * Abbreviations (rule 1.5): s4 subsystem 4, imu inertial measurement unit,
 * mg milli-g, mm millimetres, ms milliseconds, hz hertz, cal calibrated,
 * cls class (a C++ keyword), accel acceleration, decideg 0.1 degree,
 * pri task priority.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"

#define S4_PRI_IMU 18  /* Priority band 18-19 (TEAM.md). */
#define S4_IMU_MS  10  /* Sample period, from a cyclic handler. */
#define S4_CAL_N   200 /* Start-up calibration samples (2 s, car still). */
#define S4_CAL_MG  100 /* A sample this far from the mean restarts it. */

#define S4_TRACK_MM    114 /* Distance between the drive wheel centres. */
#define S4_HUMP_ON_MM  3   /* Right wheel this high: on a hump... */
#define S4_HUMP_MIN_MS 200 /* ...for at least this long. */
#define S4_HUMP_OFF_MM 1   /* Back below this: the hump is over. */
#define S4_ACCEL_MG    50  /* Forward push beyond this: accelerating... */
#define S4_ACCEL_MS    100 /* ...for at least this long. */

#define S4_BUZZER_PIN 22u   /* Robo Pico buzzer; 18 on the Maker Pi Pico. */
#define S4_BEEP_HZ    4000u /* Beep when calibration is done. */
#define S4_BEEP_MS    200
#define S4_ALARM_MS   1000 /* Beep on a fault. */

#define S4_TILT_MAX  250 /* Pitch or roll beyond 25.0 degrees... */
#define S4_TILT_MS   500 /* ...for this long: excessive tilt. */
#define S4_UPSIDE_MS 200 /* Z below 0 mg this long: upside down. */
#define S4_LIFT_MG   300 /* Z this far above 1 g... */
#define S4_LIFT_MS   50  /* ...this long: lifted (quick lifts only). */

/* Largest values the packed fields below hold. */
#define S4_AXIS_MAX_MG 2047
#define S4_ANGLE_MAX   900 /* 90.0 degrees. */
#define S4_HUMP_MAX_MM 63

/* Private to subsystem 4. Latest calibrated reading, packed to its range:
 * each axis is -2048..2047 mg (the sensor's 12-bit range at +-2 g); count
 * is a wrapping 8-bit sample counter. 36 + 8 bits, stored in 8 bytes. */
typedef struct
{
    signed int x : 12;
    signed int y : 12;
    unsigned int count : 8;
    signed int z : 12;
} s4_accel_t;

/* Tilt relative to the start surface in 0.1 degree, -900..900: pitch is
 * nose up positive, roll is left side up positive. cal is 1 once
 * start-up calibration is done; until then pitch and roll are 0.
 * 11 + 11 + 1 bits, stored in 4 bytes. */
typedef struct
{
    signed int pitch : 11;
    signed int roll : 11;
    unsigned int cal : 1;
} s4_tilt_t;

/* Humps under the right wheel, in mm: peak of the last one, highest of
 * the run, how many so far (wraps), and whether the car is on one now.
 * 6 + 6 + 4 + 1 bits, stored in 4 bytes. */
typedef struct
{
    unsigned int peak : 6;
    unsigned int max : 6;
    unsigned int count : 4;
    unsigned int on : 1;
} s4_hump_t;

/* Current motion class, highest first: climbing or descending a hump,
 * accelerating or braking, steady (parked or cruising: telling them apart needs
 * encoder speed). */
typedef enum
{
    S4_STEADY,
    S4_ACCELERATING,
    S4_BRAKING,
    S4_CLIMBING,
    S4_DESCENDING
} s4_motion_class_t;

/* An s4_motion_class_t in 3 bits, stored in 4 bytes. */
typedef struct
{
    unsigned int cls : 3;
} s4_motion_t;

extern ID gh_s4_mutex; /* Guards the reading, tilt, humps, motion. */

/* Each copies its record under gh_s4_mutex. */
void s4_get_accel(s4_accel_t * p_accel);
void s4_get_tilt(s4_tilt_t * p_tilt);
void s4_get_hump(s4_hump_t * p_hump);
void s4_get_motion(s4_motion_t * p_motion);
void s4_reset_hump_max(void); /* At each run's start. */
void s4_imu_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/

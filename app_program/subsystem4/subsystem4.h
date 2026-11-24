/** @file subsystem4.h
 *
 * @brief Subsystem 4 (IMU and terrain). Other subsystems read the latest
 * tilt, humps and motion class with the s4_get_*() calls. Faults: app_fault.h.
 *
 * The code follows BARR-C:2018, with the team's deviations listed in app.h
 * and these:
 * - 5.3.a: x, y, z, pitch, roll and the IMU task's filter state are signed
 *   bit-fields, packed to the minimum bits (docs/DATA_REPRESENTATION.md).
 * - 7.1.e: struct fields x, y and z keep their short names.
 * - 4.3.c, 6.2.d: this file declares the functions of task_imu.c that other
 *   subsystems call; task_imu.h holds what only subsystem 4 uses.
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

#define S4_PRI_IMU 18 /* Priority band 18-19 (TEAM.md). */

/* Largest values the packed fields below hold. */
#define S4_ANGLE_MAX   900 /* 90.0 degrees. */
#define S4_HUMP_MAX_MM 63

/* Tilt relative to the start surface in 0.1 degree, -900..900: pitch is
 * nose up positive, roll is left side up positive. b_cal is set once
 * calibration is done; until then pitch and roll are 0.
 * 11 + 11 + 1 bits, stored in 4 bytes. */
typedef struct
{
    signed int pitch : 11;
    signed int roll : 11;
    bool b_cal : 1;
} s4_tilt_t;

/* Humps under the right wheel, in mm: peak of the last one, highest of
 * the run, how many so far (wraps), and whether the car is on one now.
 * 6 + 6 + 4 + 1 bits, stored in 4 bytes. */
typedef struct
{
    unsigned int peak : 6;
    unsigned int max : 6;
    unsigned int count : 4;
    bool b_on : 1;
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

/* Each copies its record under the subsystem's mutex. */
void s4_get_tilt(s4_tilt_t * p_tilt);
void s4_get_hump(s4_hump_t * p_hump);
void s4_get_motion(s4_motion_t * p_motion);
void s4_reset_hump_max(void); /* At each run's start. */
void s4_reset_sensor(void);   /* On a Reset: clears the sensor fault. */

#endif /* SUBSYSTEM4_H */

/*** end of file ***/

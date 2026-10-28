/*
 *      subsystem4.h
 *      Subsystem 4 -- IMU and terrain.  Student ID: <student ID>.
 *
 *      Public interface of subsystem 4.  Put here only what OTHER subsystems
 *      may use (object IDs, message formats, priorities); keep everything
 *      else LOCAL in the .c files.  Other subsystems include it as
 *      "../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_IMU 18  /* priority band 18-19 (TEAM.md) */
#define S4_IMU_MS  10  /* sample period, from a cyclic handler */
#define S4_CAL_N   200 /* start-up calibration samples (2 s, car still) */
#define S4_CAL_MG  100 /* a sample this far from the mean restarts it */

#define S4_TRACK_MM    114 /* distance between the drive wheel centres */
#define S4_HUMP_ON_MM  3   /* right wheel this high: on a hump */
#define S4_HUMP_MIN_MS 200 /* ...for at least this long */
#define S4_HUMP_OFF_MM 1   /* back below this: the hump is over */

#define S4_BUZZER_PIN 18   /* Maker Pi Pico buzzer; the Robo Pico's is 22 */
#define S4_BEEP_HZ    4000 /* beep when calibration is done */
#define S4_BEEP_MS    200
#define S4_ALARM_MS   1000 /* beep on a fault */

#define S4_IMPACT_MG 800 /* jump from the smoothed X or Y: impact */
#define S4_TILT_MAX  250 /* pitch or roll beyond 25.0 degrees... */
#define S4_TILT_MS   500 /* ...for this long: excessive tilt */

/* Private to subsystem 4.  Latest calibrated reading, packed to its range:
   each axis is -2048..2047 mg (the sensor's 12-bit range at +-2 g); count
   is a wrapping 8-bit sample counter.  36 + 8 bits, stored in 8 bytes. */
typedef struct
{
    signed int x : 12;
    signed int y : 12;
    unsigned int count : 8;
    signed int z : 12;
} S4_ACCEL;

/* Tilt relative to the start surface in 0.1 degree, -900..900: pitch is
   nose up positive, roll is left side up positive.  cal is 1 once
   start-up calibration is done; until then pitch and roll are 0.
   11 + 11 + 1 bits, stored in 4 bytes. */
typedef struct
{
    signed int pitch : 11;
    signed int roll : 11;
    unsigned int cal : 1;
} S4_TILT;

/* Faults, each latched until reset: a sudden impact, excessive tilt.
   2 bits, stored in 4 bytes.  They move into the team's shared fault
   bitmask once that is agreed. */
typedef struct
{
    unsigned int impact : 1;
    unsigned int tilt : 1;
} S4_FAULT;

/* Humps under the right wheel, in mm: peak of the last one, highest of
   the run, how many so far (wraps), and whether the car is on one now.
   6 + 6 + 4 + 1 bits, stored in 4 bytes. */
typedef struct
{
    unsigned int peak : 6;
    unsigned int max : 6;
    unsigned int count : 4;
    unsigned int on : 1;
} S4_HUMP;

IMPORT ID s4_mtx_accel; /* guards the reading, tilt, faults and humps */
IMPORT void s4_get_accel(S4_ACCEL * out); /* copy it under the mutex */
IMPORT void s4_get_tilt(S4_TILT * out);   /* copy it under the mutex */
IMPORT void s4_get_fault(S4_FAULT * out); /* copy it under the mutex */
IMPORT void s4_get_hump(S4_HUMP * out);   /* copy it under the mutex */
IMPORT void imu_task(INT stacd, void * exinf);

#endif /* SUBSYSTEM4_H */

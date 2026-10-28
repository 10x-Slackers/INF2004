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

IMPORT ID s4_mtx_accel; /* guards the latest reading and tilt */
IMPORT void s4_get_accel(S4_ACCEL * out); /* copy it under the mutex */
IMPORT void s4_get_tilt(S4_TILT * out);   /* copy it under the mutex */
IMPORT void imu_task(INT stacd, void * exinf);

#endif /* SUBSYSTEM4_H */

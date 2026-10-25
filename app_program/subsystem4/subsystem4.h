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
#define S4_IMU_MS  100 /* sample and print period */

/* Private to subsystem 4.  Latest reading, packed to its range: each axis
   is the sensor's 12-bit value (-2048..2047 mg); count is a wrapping 8-bit
   sample counter.  36 + 8 bits, stored in 8 bytes. */
typedef struct
{
    signed int x : 12;
    signed int y : 12;
    unsigned int count : 8;
    signed int z : 12;
} S4_ACCEL;

IMPORT ID s4_mtx_accel;                   /* guards the latest reading */
IMPORT void s4_get_accel(S4_ACCEL * out); /* copy it under the mutex */
IMPORT void imu_task(INT stacd, void * exinf);

#endif /* SUBSYSTEM4_H */

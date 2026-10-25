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

/* Private to subsystem 4 */
typedef struct
{
    W x, y, z; /* acceleration in mg */
} S4_ACCEL;

IMPORT S4_ACCEL s4_accel; /* latest reading, for the tests */
IMPORT UW s4_reads_ok;    /* successful reads, for the tests */
IMPORT void imu_task(INT stacd, void * exinf);

#endif /* SUBSYSTEM4_H */

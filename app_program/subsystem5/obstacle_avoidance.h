/** @file obstacle_avoidance.h
 *
 * @brief Obstacle avoidance API (interface I6). Any task may call it.
 *
 * Once started, the avoidance task watches the range ahead. When something
 * is within SCAN_PROFILE_BLOCK_MM it stops the car, turns it left and right
 * to measure the clearance each side, profiles the obstacle with the servo
 * and drives round it on the more open side, ending on the original line
 * and heading, then watches again. With no open side it stops and stays
 * stopped (no reverse). Units: docs/OBSTACLE_AVOIDANCE.md.
 */

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <stdbool.h>
#include <stdint.h>

/* status.state */
#define OBSTACLE_AVOIDANCE_IDLE    0u /* Not started, or cancelled. */
#define OBSTACLE_AVOIDANCE_MONITOR 1u /* Watching the range ahead. */
#define OBSTACLE_AVOIDANCE_PROFILE 2u /* Stopped, scanning the obstacle. */
#define OBSTACLE_AVOIDANCE_BYPASS  3u /* Driving round it. */
#define OBSTACLE_AVOIDANCE_STOPPED 4u /* Safe stop, until the next start. */

/* status.faults bits, cleared by obstacle_avoidance_start(). */
#define OBSTACLE_AVOIDANCE_FAULT_SENSOR  1u /* No echo, or no scan result. */
#define OBSTACLE_AVOIDANCE_FAULT_BLOCKED 2u /* Neither side open. */
#define OBSTACLE_AVOIDANCE_FAULT_MOTION  4u /* Move failed, or did not end. */

typedef struct
{
    unsigned int state : 3;
    unsigned int faults : 3;
    unsigned int side : 2;      /* SCAN_PROFILE_LEFT/RIGHT of the bypass. */
    unsigned int step : 3;      /* Bypass step 1..7, 0 outside a bypass. */
    unsigned int bypasses : 4;  /* Bypasses finished, wraps. */
    unsigned int left_mm : 12;  /* Clearance 45 degrees left, last profile. */
    unsigned int right_mm : 12; /* And 45 degrees right. */
} obstacle_avoidance_status_t;

bool obstacle_avoidance_init(void);
void obstacle_avoidance_start(void);
void obstacle_avoidance_cancel(void);
void obstacle_avoidance_get_status(obstacle_avoidance_status_t * p_status);

#endif /* OBSTACLE_AVOIDANCE_H */

/*** end of file ***/

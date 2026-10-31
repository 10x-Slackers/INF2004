/** @file subsystem5.h
 *
 * @brief Subsystem 5 (ultrasonic scanning and obstacle avoidance) start-up
 * and tasks. Other subsystems use ultrasonic_scan.h and
 * obstacle_avoidance.h.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "obstacle_avoidance.h"
#include "ultrasonic_scan.h"

#define S5_PRI_AVOID 11 /* Priority band 11-12 (TEAM.md). */
#define S5_PRI_SCAN  12
#define S5_PRI_BENCH 12 /* TEST=1 only. */

void ultrasonic_scan_task(INT stacd, void * p_exinf);
ID obstacle_avoidance_start_task(void);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/

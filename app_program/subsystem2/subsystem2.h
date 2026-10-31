/** @file subsystem2.h
 *
 * @brief Subsystem 2 (motion control) start-up and tasks. Other
 * subsystems use motion_control.h.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "motion_control.h"

#define S2_PRI_MOTION 6
#define S2_PRI_BENCH  15 /* TEST=1 only. */

uint32_t subsystem2_max_tick_us(void);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/

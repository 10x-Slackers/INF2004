/** @file safety_monitor.h
 *
 * @brief Safety Monitor (I11): latches a stop on an IMU fault, a collision
 * risk, a wheel stall or a stop raised by another task. The latch is motion
 * control's (motion_control_latched()), so nothing can drive the car until
 * safety_monitor_clear(). Runs only in normal builds, like the controller.
 *
 * Abbreviations (rule 1.5): mm millimetres, ms milliseconds, imu inertial
 * measurement unit.
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"

#define SAFETY_MONITOR_PRI 5 /* Shared priority band (TEAM.md). */

/* Causes of a stop, printed on the console. */
#define SAFETY_MONITOR_IMU        1u /* Impact or excessive tilt (I9). */
#define SAFETY_MONITOR_COLLISION  2u /* Something close ahead while driving. */
#define SAFETY_MONITOR_STALL      4u /* Motion FAILED: a wheel stalled. */
#define SAFETY_MONITOR_CONTROLLER 8u /* Raised by the Vehicle Controller. */

ER safety_monitor_start(void);
void safety_monitor_stop(uint32_t cause);
bool safety_monitor_clear(void);

#endif /* SAFETY_MONITOR_H */

/*** end of file ***/

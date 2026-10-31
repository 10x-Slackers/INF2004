/** @file vehicle_controller.h
 *
 * @brief Vehicle Controller: the mission state machine and the arbiter of
 * normal motion. It owns no hardware except the GP20 Start/Stop button and
 * runs only in normal builds (TEST=1 keeps GP20 for the avoidance bench).
 *
 * Abbreviations (rule 1.5): vc vehicle controller, mm millimetres,
 * ms milliseconds, deg degrees.
 */

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../app.h"

#define VC_PRI 10 /* Shared priority band (TEAM.md). */

ER vehicle_controller_start(void);

#endif /* VEHICLE_CONTROLLER_H */

/*** end of file ***/

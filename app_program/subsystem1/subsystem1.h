/** @file subsystem1.h
 *
 * @brief Subsystem 1 (Wi-Fi, command and telemetry). Student ID: 2503014.
 * Other subsystems use only the supervisor's command bits (interface I12,
 * docs/TELEMETRY.md).
 *
 * Abbreviations (rule 1.5): s1 subsystem 1, cmd command, pri task priority,
 * mqtt the MQTT protocol, utk micro T-Kernel.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_TELEMETRY 20 /* Priority band 20-21 (TEAM.md). */

/* Bits of the csc13/cmd byte and of gh_s1_command_flag. */
#define S1_CMD_START 1u /* As GP20 in READY. */
#define S1_CMD_STOP  2u /* Safe stop, in every state but INIT. */
#define S1_CMD_RESET 4u /* As GP20 in SAFE_STOP. */
#define S1_CMD_ALL   (S1_CMD_START | S1_CMD_STOP | S1_CMD_RESET)

/* Event flag: the Wi-Fi service sets the supervisor's command bits, the
 * Vehicle Controller takes them. It exists in every build. */
extern ID gh_s1_command_flag;

#endif /* SUBSYSTEM1_H */

/*** end of file ***/

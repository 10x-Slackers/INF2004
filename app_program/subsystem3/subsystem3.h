/** @file subsystem3.h
 *
 * @brief Subsystem 3 (Line following and barcodes) start-up and tasks.
 * Student ID: 2502564. Other subsystems use line_following.h.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "line_following.h"

#define S3_PRI_LINE  9 /* Priority band 8-9 (TEAM.md). */
#define S3_PRI_BENCH 9 /* TEST=1 only. */

void line_following_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM3_H */

/*** end of file ***/

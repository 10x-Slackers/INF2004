/* Buddy 2 task integration; public movement API is motion_control.h. */
#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "motion_control.h"

#define S2_PRI_MOTION 6
#define S2_PRI_CONSOLE 22
void subsystem2_console_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM2_H */
/* End of file. */

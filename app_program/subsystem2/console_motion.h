/** @file console_motion.h
 *
 * @brief USB console for testing, calibrating and tuning motion control.
 */

#ifndef CONSOLE_MOTION_H
#define CONSOLE_MOTION_H

#include "../app.h"
#include <stdint.h>

/* Name fixed by the weak hook in lib/libtm. Runs in the USB task. */
void tm_console_receive_byte(uint8_t byte);
void console_motion_task(INT stacd, void * p_exinf);

#endif /* CONSOLE_MOTION_H */

/*** end of file ***/

/** @file tmonitor.h
 *
 * @brief Host stand-in for the T-Monitor console: tm_printf() only.
 */

#ifndef TMONITOR_H
#define TMONITOR_H

#include "../tk/tkernel.h"

INT tm_printf(UB const * p_format, ...);

#endif /* TMONITOR_H */

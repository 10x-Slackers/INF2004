/** @file drv_line.h
 *
 * @brief Left and right line IR modules: digital outputs on GP6 and GP7.
 * Call only from the Line Following task, which owns the sensors.
 */

#ifndef DRV_LINE_H
#define DRV_LINE_H

#include <stdint.h>

/* drv_line_read() bits: set while that sensor is over black. */
#define DRV_LINE_LEFT  1u
#define DRV_LINE_RIGHT 2u

void drv_line_init(void);
uint32_t drv_line_read(void);

#endif /* DRV_LINE_H */

/*** end of file ***/

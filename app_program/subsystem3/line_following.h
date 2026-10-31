/** @file line_following.h
 *
 * @brief Line Following API (interface I1). Any task may read the latest
 * line state; the Line Following task samples the sensors every 10 ms.
 *
 * line_following_start() makes the task steer the car along the line
 * with motion_control_drive() until line_following_stop(); a junction
 * does not stop it.
 *
 * The two sensors straddle the 18 mm line: both white means the car is
 * centred (or has lost the line, see last_side), one black means the line
 * has drifted under that sensor, both black means a junction or a marking
 * across the line.
 */

#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include <stdbool.h>
#include <stdint.h>
#include "drv_line.h"

#define LINE_FOLLOWING_PERIOD_MS 10u
#define LINE_FOLLOWING_MAX_MM_S  500u /* line_following_start() limit. */

/* Latest line state. */
typedef struct
{
    unsigned int bits : 2;        /* DRV_LINE_LEFT | DRV_LINE_RIGHT now. */
    unsigned int last_side : 2;   /* Last single sensor to see black. */
    unsigned int b_junction : 1;  /* Both black for 3 samples in a row. */
    unsigned int b_following : 1; /* Steering the car. */
    unsigned int count : 8;       /* Samples taken, wraps. */
    unsigned int sample_ms : 32;  /* Kernel time of the sample, wraps. */
} line_following_t;

bool line_following_init(void);
void line_following_get(line_following_t * p_line);
bool line_following_start(uint32_t speed_mm_s);
void line_following_stop(void);

#endif /* LINE_FOLLOWING_H */

/*** end of file ***/

/** @file ultrasonic_scan.h
 *
 * @brief Ultrasonic scan API (interfaces I7 and I8). Any task may request
 * a scan and read the latest result; the scan task does the work.
 *
 * Request a scan, then poll ultrasonic_scan_get() until count changes.
 * A forward reading takes about 60 ms; a profile takes about 2 s and needs
 * the car to stand still.
 */

#ifndef ULTRASONIC_SCAN_H
#define ULTRASONIC_SCAN_H

#include <stdbool.h>
#include <stdint.h>
#include "scan_profile.h"

/* ultrasonic_scan_request() */
#define ULTRASONIC_SCAN_FORWARD 0u /* One reading straight ahead. */
#define ULTRASONIC_SCAN_PROFILE 1u /* Servo sweep, then a forward reading. */

/* Latest results. */
typedef struct
{
    unsigned int forward_mm : 12; /* 20..4000; 4000 is nothing in range. */
    unsigned int b_fault : 1;     /* Last scan had a ray with no echo. */
    unsigned int count : 4;       /* Scans finished, wraps. */
    unsigned int forward_ms : 32; /* Kernel time of forward_mm, wraps. */
    scan_profile_t profile;       /* Last profile scan. */
} ultrasonic_scan_t;

bool ultrasonic_scan_init(void);
void ultrasonic_scan_request(uint32_t mode);
void ultrasonic_scan_get(ultrasonic_scan_t * p_scan);

#endif /* ULTRASONIC_SCAN_H */

/*** end of file ***/

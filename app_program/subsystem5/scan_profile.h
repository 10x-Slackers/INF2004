/** @file scan_profile.h
 *
 * @brief Obstacle profile from one servo sweep of ultrasonic ranges
 * (closest point and edges), and the bypass side from the clearance on
 * each side. No hardware or kernel calls, so the host test builds it on
 * its own.
 *
 * Angles are servo angles: 80 degrees points right, 90 ahead, 100 left.
 * The mount stops the servo about 15 degrees either side of ahead.
 */

#ifndef SCAN_PROFILE_H
#define SCAN_PROFILE_H

#include <stdint.h>

#define SCAN_PROFILE_MIN_DEG  80u
#define SCAN_PROFILE_MAX_DEG  100u
#define SCAN_PROFILE_STEP_DEG 5u
#define SCAN_PROFILE_RAYS     5u    /* 80..100 degrees in 5 degree steps. */
#define SCAN_PROFILE_MIN_MM   20u   /* Sensor's shortest range. */
#define SCAN_PROFILE_FAR_MM   4000u /* Nothing within the sensor's range. */
#define SCAN_PROFILE_BLOCK_MM 150u  /* A ray this close hits an obstacle. */
#define SCAN_PROFILE_CLEAR_MM 250u  /* A side this open can be passed. */

/* scan_profile_side() */
#define SCAN_PROFILE_NONE  0u
#define SCAN_PROFILE_LEFT  1u
#define SCAN_PROFILE_RIGHT 2u

/* Obstacle profile. The edges are the outermost blocked rays next to the
 * closest one. The width is rough: the arc between the edges at the
 * closest distance, at most about 120 mm when the obstacle fills the
 * sweep. */
typedef struct
{
    unsigned int closest_mm : 12;
    unsigned int closest_deg : 7;
    unsigned int right_deg : 7;
    unsigned int b_obstacle : 1; /* closest_mm <= SCAN_PROFILE_BLOCK_MM. */
    unsigned int left_deg : 7;
    unsigned int width_mm : 7; /* 0 if no obstacle. */
} scan_profile_t;

uint32_t scan_profile_mm(uint32_t echo_us);
uint32_t scan_profile_median(uint32_t first, uint32_t second, uint32_t third);
void scan_profile_build(const uint16_t * p_rays, scan_profile_t * p_profile);
uint32_t scan_profile_side(uint32_t left_mm, uint32_t right_mm);

#endif /* SCAN_PROFILE_H */

/*** end of file ***/

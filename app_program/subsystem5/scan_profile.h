/** @file scan_profile.h
 *
 * @brief Obstacle profile from one sweep of ultrasonic ranges: closest
 * point, obstacle edges and the clearance on each side. No hardware or
 * kernel calls, so the host test builds it on its own.
 *
 * Angles are servo angles: 30 degrees points right, 90 ahead, 150 left.
 */

#ifndef SCAN_PROFILE_H
#define SCAN_PROFILE_H

#include <stdint.h>

#define SCAN_PROFILE_MIN_DEG  30u
#define SCAN_PROFILE_MAX_DEG  150u
#define SCAN_PROFILE_STEP_DEG 5u
#define SCAN_PROFILE_RAYS     25u   /* 30..150 degrees in 5 degree steps. */
#define SCAN_PROFILE_MIN_MM   20u   /* Sensor's shortest range. */
#define SCAN_PROFILE_FAR_MM   4000u /* Nothing within the sensor's range. */
#define SCAN_PROFILE_BLOCK_MM 350u  /* A ray this close hits an obstacle. */
#define SCAN_PROFILE_CLEAR_MM 500u  /* A side this open can be passed. */

/* scan_profile_side() */
#define SCAN_PROFILE_NONE  0u
#define SCAN_PROFILE_LEFT  1u
#define SCAN_PROFILE_RIGHT 2u

/* Obstacle profile. Edges are the outermost blocked rays; a side range is
 * the nearest ray beyond that edge, 0 if no ray was taken there. */
typedef struct
{
    unsigned int closest_mm : 12; /* 0 if no ray was taken. */
    unsigned int closest_deg : 8;
    unsigned int right_deg : 8;
    unsigned int b_obstacle : 1; /* closest_mm <= SCAN_PROFILE_BLOCK_MM. */
    unsigned int left_deg : 8;
    unsigned int right_mm : 12;
    unsigned int left_mm : 12;
} scan_profile_t;

uint32_t scan_profile_mm(uint32_t echo_us);
uint32_t scan_profile_median(uint32_t a, uint32_t b, uint32_t c);
void scan_profile_build(const uint16_t * p_rays, scan_profile_t * p_profile);
uint32_t scan_profile_side(const scan_profile_t * p_profile);

#endif /* SCAN_PROFILE_H */

/*** end of file ***/

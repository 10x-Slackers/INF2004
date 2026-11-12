/** @file scan_profile.h
 *
 * @brief Obstacle profile from one servo sweep of ultrasonic ranges
 * (closest point, edges and the clearance each side), and the bypass side
 * from those clearances. No hardware or kernel calls, so the host test
 * builds it on its own.
 *
 * Angles are servo angles: 45 degrees points right, 90 ahead, 135 left.
 * The mount limits the servo to a little under 60 degrees either side.
 * The coarse scan reads every third ray (every 15 degrees); the fine scan
 * fills in the rays next to the obstacle ahead.
 */

#ifndef SCAN_PROFILE_H
#define SCAN_PROFILE_H

#include <stdint.h>

#define SCAN_PROFILE_MIN_DEG  45u
#define SCAN_PROFILE_MAX_DEG  135u
#define SCAN_PROFILE_STEP_DEG 5u
#define SCAN_PROFILE_RAYS     19u   /* 45..135 degrees in 5 degree steps. */
#define SCAN_PROFILE_AHEAD    9u    /* Ray at 90 degrees. */
#define SCAN_PROFILE_COARSE   3u    /* Coarse rays: every third, 15 degrees. */
#define SCAN_PROFILE_MIN_MM   20u   /* Sensor's shortest range. */
#define SCAN_PROFILE_FAR_MM   4000u /* Nothing within the sensor's range. */
#define SCAN_PROFILE_BLOCK_MM 150u  /* A ray this close hits an obstacle. */
#define SCAN_PROFILE_CLEAR_MM 250u  /* A side this open can be passed. */

/* scan_profile_side() */
#define SCAN_PROFILE_NONE  0u
#define SCAN_PROFILE_LEFT  1u
#define SCAN_PROFILE_RIGHT 2u

/* Obstacle profile. The obstacle is the run of blocked rays around the ray
 * ahead; the edges are its outermost rays. The width is rough: the arc
 * between the edges at the closest distance, at most about 235 mm. The
 * clearances are the outer rays, 45 degrees either side of ahead. */
typedef struct
{
    unsigned int closest_mm : 12;
    unsigned int closest_deg : 8;
    unsigned int right_deg : 8;
    unsigned int b_obstacle : 1; /* Ray ahead <= SCAN_PROFILE_BLOCK_MM. */
    unsigned int left_deg : 8;
    unsigned int width_mm : 8;  /* 0 if no obstacle. */
    unsigned int left_mm : 12;  /* Ray at 135 degrees. */
    unsigned int right_mm : 12; /* Ray at 45 degrees. */
} scan_profile_t;

uint32_t scan_profile_mm(uint32_t echo_us);
uint32_t scan_profile_median(uint32_t first, uint32_t second, uint32_t third);
void scan_profile_build(const uint16_t * p_rays, scan_profile_t * p_profile);
uint32_t scan_profile_side(uint32_t left_mm, uint32_t right_mm);

#endif /* SCAN_PROFILE_H */

/*** end of file ***/

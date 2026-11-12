/** @file scan_profile.c
 *
 * @brief Obstacle profile from one sweep of ultrasonic ranges.
 */

#include "scan_profile.h"
#include <stdbool.h>

/*!
 * @brief Echo pulse width to distance: 58 us per cm there and back.
 *
 * @return mm, at most SCAN_PROFILE_FAR_MM.
 */
uint32_t scan_profile_mm(uint32_t echo_us)
{
    uint32_t const range_mm = ((echo_us * 10u) + 29u) / 58u;

    return (range_mm > SCAN_PROFILE_FAR_MM) ? SCAN_PROFILE_FAR_MM : range_mm;
}

/*!
 * @brief Middle value of three readings, to drop one bad echo.
 */
uint32_t scan_profile_median(uint32_t first, uint32_t second, uint32_t third)
{
    uint32_t const low = (first < second) ? first : second;
    uint32_t const high = (first < second) ? second : first;

    return (third < low) ? low : ((third > high) ? high : third);
}

/*!
 * @brief Build the profile of the obstacle ahead.
 *
 * @param[in]  p_rays    SCAN_PROFILE_RAYS ranges in mm, each ray at
 *                       SCAN_PROFILE_MIN_DEG + 5 * ray degrees. Rays not
 *                       measured read SCAN_PROFILE_FAR_MM.
 * @param[out] p_profile The profile.
 */
void scan_profile_build(const uint16_t * p_rays, scan_profile_t * p_profile)
{
    bool const b_obstacle =
        (p_rays[SCAN_PROFILE_AHEAD] <= SCAN_PROFILE_BLOCK_MM);
    uint32_t closest = SCAN_PROFILE_AHEAD;
    uint32_t right = SCAN_PROFILE_AHEAD;
    uint32_t left = SCAN_PROFILE_AHEAD;
    uint32_t ray = 0u;

    /* The obstacle is the run of blocked rays around the one ahead. */
    while (b_obstacle && (right > 0u) &&
           (p_rays[right - 1u] <= SCAN_PROFILE_BLOCK_MM))
    {
        right--;
    }
    while (b_obstacle && ((left + 1u) < SCAN_PROFILE_RAYS) &&
           (p_rays[left + 1u] <= SCAN_PROFILE_BLOCK_MM))
    {
        left++;
    }
    for (ray = right; ray <= left; ray++)
    {
        closest = (p_rays[ray] < p_rays[closest]) ? ray : closest;
    }

    p_profile->closest_mm = p_rays[closest];
    p_profile->closest_deg =
        SCAN_PROFILE_MIN_DEG + (closest * SCAN_PROFILE_STEP_DEG);
    p_profile->b_obstacle = b_obstacle;
    p_profile->right_deg =
        SCAN_PROFILE_MIN_DEG + (right * SCAN_PROFILE_STEP_DEG);
    p_profile->left_deg = SCAN_PROFILE_MIN_DEG + (left * SCAN_PROFILE_STEP_DEG);

    /* Arc length r * angle, with pi / 180 as 1745 / 100000. */
    p_profile->width_mm = b_obstacle ? ((p_rays[closest] * (left - right) *
                                         SCAN_PROFILE_STEP_DEG * 1745u) /
                                        100000u)
                                     : 0u;
    p_profile->left_mm = p_rays[SCAN_PROFILE_RAYS - 1u];
    p_profile->right_mm = p_rays[0];
}

/*!
 * @brief Side to pass the obstacle on: the more open side with at least
 * SCAN_PROFILE_CLEAR_MM, left on a tie.
 *
 * @return SCAN_PROFILE_LEFT, _RIGHT, or _NONE if neither side is open.
 */
uint32_t scan_profile_side(uint32_t left_mm, uint32_t right_mm)
{
    uint32_t side = SCAN_PROFILE_NONE;

    if ((left_mm >= SCAN_PROFILE_CLEAR_MM) && (left_mm >= right_mm))
    {
        side = SCAN_PROFILE_LEFT;
    }
    else if (right_mm >= SCAN_PROFILE_CLEAR_MM)
    {
        side = SCAN_PROFILE_RIGHT;
    }
    else
    {
        /* Neither side is open: no reverse, the caller stops. */
    }
    return side;
}

/*** end of file ***/

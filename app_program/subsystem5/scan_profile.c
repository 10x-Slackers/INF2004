/** @file scan_profile.c
 *
 * @brief Obstacle profile from one sweep of ultrasonic ranges.
 */

#include "scan_profile.h"

/*!
 * @brief Echo pulse width to distance: 58 us per cm there and back.
 *
 * @return mm, at most SCAN_PROFILE_FAR_MM.
 */
uint32_t scan_profile_mm(uint32_t echo_us)
{
    uint32_t mm = ((echo_us * 10u) + 29u) / 58u;

    return (mm > SCAN_PROFILE_FAR_MM) ? SCAN_PROFILE_FAR_MM : mm;
}

/*!
 * @brief Middle value of three readings, to drop one bad echo.
 */
uint32_t scan_profile_median(uint32_t a, uint32_t b, uint32_t c)
{
    uint32_t low = (a < b) ? a : b;
    uint32_t high = (a < b) ? b : a;

    return (c < low) ? low : ((c > high) ? high : c);
}

/*!
 * @brief Build the profile of the obstacle nearest the car.
 *
 * @param[in]  p_rays    SCAN_PROFILE_RAYS ranges in mm, ray i at
 *                       SCAN_PROFILE_MIN_DEG + 5 i degrees.
 * @param[out] p_profile The profile.
 */
void scan_profile_build(const uint16_t * p_rays, scan_profile_t * p_profile)
{
    uint32_t closest = 0u;
    uint32_t right = 0u;
    uint32_t left = 0u;
    uint32_t i = 0u;

    for (i = 1u; i < SCAN_PROFILE_RAYS; i++)
    {
        closest = (p_rays[i] < p_rays[closest]) ? i : closest;
    }

    /* The obstacle is the run of blocked rays around the closest one. */
    right = closest;
    while ((right > 0u) && (p_rays[right - 1u] <= SCAN_PROFILE_BLOCK_MM))
    {
        right--;
    }
    left = closest;
    while (((left + 1u) < SCAN_PROFILE_RAYS) &&
           (p_rays[left + 1u] <= SCAN_PROFILE_BLOCK_MM))
    {
        left++;
    }

    p_profile->closest_mm = p_rays[closest];
    p_profile->closest_deg =
        SCAN_PROFILE_MIN_DEG + (closest * SCAN_PROFILE_STEP_DEG);
    p_profile->b_obstacle = (p_rays[closest] <= SCAN_PROFILE_BLOCK_MM);
    p_profile->right_deg =
        SCAN_PROFILE_MIN_DEG + (right * SCAN_PROFILE_STEP_DEG);
    p_profile->left_deg = SCAN_PROFILE_MIN_DEG + (left * SCAN_PROFILE_STEP_DEG);

    /* Arc length r * angle, with pi / 180 as 1745 / 100000. */
    p_profile->width_mm = p_profile->b_obstacle
                              ? ((p_rays[closest] * (left - right) *
                                  SCAN_PROFILE_STEP_DEG * 1745u) /
                                 100000u)
                              : 0u;
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

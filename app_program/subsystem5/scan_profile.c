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
 * @param[in]  p_rays    SCAN_PROFILE_RAYS ranges in mm, ray i at 30 + 5 i
 *                       degrees; 0 where no ray was taken.
 * @param[out] p_profile The profile.
 */
void scan_profile_build(const uint16_t * p_rays, scan_profile_t * p_profile)
{
    uint32_t closest = SCAN_PROFILE_RAYS;
    uint32_t right = 0u;
    uint32_t left = 0u;
    uint32_t i = 0u;

    *p_profile = (scan_profile_t){0};
    for (i = 0u; i < SCAN_PROFILE_RAYS; i++)
    {
        if ((0u != p_rays[i]) &&
            ((SCAN_PROFILE_RAYS == closest) || (p_rays[i] < p_rays[closest])))
        {
            closest = i;
        }
    }
    if (SCAN_PROFILE_RAYS == closest)
    {
        return;
    }

    /* The obstacle is the run of blocked rays around the closest one;
     * skipped angles do not end it. */
    right = closest;
    left = closest;
    i = closest;
    while ((i > 0u) && (p_rays[i - 1u] <= SCAN_PROFILE_BLOCK_MM))
    {
        i--;
        right = (0u != p_rays[i]) ? i : right;
    }
    i = closest;
    while (((i + 1u) < SCAN_PROFILE_RAYS) &&
           (p_rays[i + 1u] <= SCAN_PROFILE_BLOCK_MM))
    {
        i++;
        left = (0u != p_rays[i]) ? i : left;
    }

    p_profile->closest_mm = p_rays[closest];
    p_profile->closest_deg =
        SCAN_PROFILE_MIN_DEG + (closest * SCAN_PROFILE_STEP_DEG);
    p_profile->b_obstacle = (p_rays[closest] <= SCAN_PROFILE_BLOCK_MM);
    p_profile->right_deg =
        SCAN_PROFILE_MIN_DEG + (right * SCAN_PROFILE_STEP_DEG);
    p_profile->left_deg = SCAN_PROFILE_MIN_DEG + (left * SCAN_PROFILE_STEP_DEG);
    for (i = 0u; i < SCAN_PROFILE_RAYS; i++)
    {
        if ((0u != p_rays[i]) && (i < right) &&
            ((0u == p_profile->right_mm) || (p_rays[i] < p_profile->right_mm)))
        {
            p_profile->right_mm = p_rays[i];
        }
        if ((0u != p_rays[i]) && (i > left) &&
            ((0u == p_profile->left_mm) || (p_rays[i] < p_profile->left_mm)))
        {
            p_profile->left_mm = p_rays[i];
        }
    }
}

/*!
 * @brief Side to pass the obstacle on: the more open side with at least
 * SCAN_PROFILE_CLEAR_MM, left on a tie.
 *
 * @return SCAN_PROFILE_LEFT, _RIGHT, or _NONE if neither side is open.
 */
uint32_t scan_profile_side(const scan_profile_t * p_profile)
{
    uint32_t side = SCAN_PROFILE_NONE;

    if ((p_profile->left_mm >= SCAN_PROFILE_CLEAR_MM) &&
        (p_profile->left_mm >= p_profile->right_mm))
    {
        side = SCAN_PROFILE_LEFT;
    }
    else if (p_profile->right_mm >= SCAN_PROFILE_CLEAR_MM)
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

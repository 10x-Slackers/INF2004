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
 * @brief Arc length range * angle, with pi / 180 as 1745 / 100000.
 */
uint32_t scan_profile_arc_mm(uint32_t range_mm, uint32_t angle_deg)
{
    return (range_mm * angle_deg * 1745u) / 100000u;
}

/*!
 * @brief Width as an arc, capped to fit 8 bits: only a run over about 70
 * degrees reaches it.
 */
uint32_t scan_profile_width_mm(uint32_t range_mm, uint32_t angle_deg)
{
    uint32_t const width_mm = scan_profile_arc_mm(range_mm, angle_deg);

    return (width_mm > 255u) ? 255u : width_mm;
}

/*!
 * @brief Furthest a ray on the obstacle ahead reads: SCAN_PROFILE_DEPTH_MM
 * past the ray ahead, and at least SCAN_PROFILE_BLOCK_MM.
 *
 * @return mm, or 0 if the ray ahead is not blocked.
 */
uint32_t scan_profile_reach_mm(uint32_t ahead_mm)
{
    uint32_t reach_mm = ahead_mm + SCAN_PROFILE_DEPTH_MM;

    if (ahead_mm > SCAN_PROFILE_BLOCK_MM)
    {
        reach_mm = 0u;
    }
    else if (reach_mm < SCAN_PROFILE_BLOCK_MM)
    {
        reach_mm = SCAN_PROFILE_BLOCK_MM;
    }
    else
    {
        /* Past the ray ahead. */
    }
    return reach_mm;
}

/*!
 * @brief The run of rays within limit_mm around the ray ahead, as its
 * outermost rays. Both are the ray ahead if that is not within it.
 */
static void scan_profile_run(const uint16_t * p_rays, uint32_t limit_mm,
                             uint32_t * p_right, uint32_t * p_left)
{
    *p_right = SCAN_PROFILE_AHEAD;
    *p_left = SCAN_PROFILE_AHEAD;
    if (p_rays[SCAN_PROFILE_AHEAD] <= limit_mm)
    {
        while ((*p_right > 0u) && (p_rays[*p_right - 1u] <= limit_mm))
        {
            (*p_right)--;
        }
        while (((*p_left + 1u) < SCAN_PROFILE_RAYS) &&
               (p_rays[*p_left + 1u] <= limit_mm))
        {
            (*p_left)++;
        }
    }
}

/*!
 * @brief Build the profile of the obstacle ahead.
 *
 * @param[in]  p_rays    SCAN_PROFILE_RAYS ranges in mm, each ray at
 *                       SCAN_PROFILE_DEG(ray) degrees. Rays not
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
    uint32_t wide_right = SCAN_PROFILE_AHEAD;
    uint32_t wide_left = SCAN_PROFILE_AHEAD;
    uint32_t ray = 0u;
    uint32_t span_deg = 0u;

    /* The obstacle is the run of blocked rays around the one ahead; its
     * width is measured across the wider run on the same obstacle. */
    scan_profile_run(p_rays, SCAN_PROFILE_BLOCK_MM, &right, &left);
    scan_profile_run(p_rays, scan_profile_reach_mm(p_rays[SCAN_PROFILE_AHEAD]),
                     &wide_right, &wide_left);
    for (ray = right; ray <= left; ray++)
    {
        closest = (p_rays[ray] < p_rays[closest]) ? ray : closest;
    }

    p_profile->closest_mm = p_rays[closest];
    p_profile->closest_deg = SCAN_PROFILE_DEG(closest);
    p_profile->b_obstacle = b_obstacle;
    p_profile->right_deg = SCAN_PROFILE_DEG(right);
    p_profile->left_deg = SCAN_PROFILE_DEG(left);
    /* Less the beam spread at both edges. With no obstacle the edges
     * meet, so the width is 0. */
    span_deg = SCAN_PROFILE_DEG(wide_left) - SCAN_PROFILE_DEG(wide_right);
    span_deg = (span_deg > (2u * SCAN_PROFILE_EDGE_DEG))
                   ? (span_deg - (2u * SCAN_PROFILE_EDGE_DEG))
                   : 0u;
    p_profile->width_mm = scan_profile_width_mm(p_rays[closest], span_deg);
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

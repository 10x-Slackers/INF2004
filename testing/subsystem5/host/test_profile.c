/** @file test_profile.c
 *
 * @brief Host test of scan_profile.c: echo conversion, median, profile and
 * side choice, and the sizes of subsystem 5's packed records. Build and
 * run as in S5-U04 of testing/subsystem5/TEST_LOG.md.
 */

#include "../../../app_program/subsystem5/obstacle_avoidance.h"
#include "../../../app_program/subsystem5/scan_profile.h"
#include "../../../app_program/subsystem5/ultrasonic_scan.h"
#include <stdbool.h>
#include <stdio.h>

static int g_failures;

/* Print one result and count failures. */
static void test_check(bool b_pass, const char * p_text)
{
    printf("%s  %s\n", b_pass ? "PASS" : "FAIL", p_text);
    if (!b_pass)
    {
        g_failures++;
    }
}

/* Profile with the ray ahead (90 degrees) and its neighbours set: rays
 * at 75..105 degrees, side rays at 45 and 135, the rest open. */
static scan_profile_t test_profile(const uint16_t * p_mid, uint16_t right_mm,
                                   uint16_t left_mm)
{
    uint16_t rays[SCAN_PROFILE_RAYS] = {0};
    scan_profile_t profile = {0};
    uint32_t ray = 0u;

    for (ray = 0u; ray < SCAN_PROFILE_RAYS; ray++)
    {
        rays[ray] = SCAN_PROFILE_FAR_MM;
    }
    for (ray = 0u; ray < 7u; ray++)
    {
        rays[SCAN_PROFILE_AHEAD - 3u + ray] = p_mid[ray];
    }
    rays[0] = right_mm;
    rays[SCAN_PROFILE_RAYS - 1u] = left_mm;
    scan_profile_build(rays, &profile);
    return profile;
}

int main(void)
{
    const uint16_t centred[7] = {800u, 800u, 140u, 100u, 120u, 800u, 800u};
    const uint16_t off_centre[7] = {120u, 140u, 130u, 140u, 800u, 90u, 90u};
    const uint16_t one_ray[7] = {4000u, 4000u, 4000u, 150u,
                                 4000u, 4000u, 4000u};
    const uint16_t open[7] = {4000u, 4000u, 4000u, 4000u, 4000u, 4000u, 4000u};
    scan_profile_t p;
    uint16_t rays[SCAN_PROFILE_RAYS] = {0};
    uint32_t ray = 0u;

    test_check((1000u == scan_profile_mm(5800u)) &&
                   (200u == scan_profile_mm(1160u)) &&
                   (SCAN_PROFILE_FAR_MM == scan_profile_mm(30000u)),
               "echo time to mm; past 4 m reads as 4000 (nothing there)");
    test_check((5u == scan_profile_median(9u, 5u, 1u)) &&
                   (5u == scan_profile_median(5u, 1u, 9u)) &&
                   (5u == scan_profile_median(1u, 9u, 5u)),
               "median of three");

    p = test_profile(centred, 900u, 600u);
    test_check((100u == p.closest_mm) && (90u == p.closest_deg) &&
                   (1u == p.b_obstacle) && (85u == p.right_deg) &&
                   (95u == p.left_deg),
               "centred obstacle: closest point and edges at 85..95");
    test_check(17u == p.width_mm,
               "width: 100 mm x 10 degrees = 17 mm (arc length)");
    test_check((900u == p.right_mm) && (600u == p.left_mm),
               "clearances: rays at 45 and 135 degrees");

    p = test_profile(off_centre, 900u, 600u);
    test_check((120u == p.closest_mm) && (75u == p.closest_deg) &&
                   (75u == p.right_deg) && (90u == p.left_deg),
               "run reaching right; a closer blocked run to the left is "
               "not it");

    for (ray = 0u; ray < SCAN_PROFILE_RAYS; ray++)
    {
        rays[ray] = 200u;
    }
    scan_profile_build(rays, &p);
    test_check((45u == p.right_deg) && (135u == p.left_deg) &&
                   (255u == p.width_mm) && (200u == p.right_mm) &&
                   (200u == p.left_mm),
               "blocked across the sweep: edges at both ends, width "
               "314 mm capped at 255");

    p = test_profile(one_ray, 4000u, 4000u);
    test_check((0u == p.width_mm) && (1u == p.b_obstacle) &&
                   (90u == p.right_deg) && (90u == p.left_deg),
               "single blocked ray: width 0");

    p = test_profile(open, 100u, 100u);
    test_check((0u == p.b_obstacle) && (4000u == p.closest_mm) &&
                   (0u == p.width_mm) && (90u == p.right_deg) &&
                   (90u == p.left_deg),
               "clear ahead: no obstacle, even with blocked side rays");

    test_check((SCAN_PROFILE_LEFT == scan_profile_side(800u, 800u)) &&
                   (SCAN_PROFILE_LEFT == scan_profile_side(900u, 600u)) &&
                   (SCAN_PROFILE_RIGHT == scan_profile_side(600u, 900u)),
               "the more open side, left on a tie");
    test_check((SCAN_PROFILE_RIGHT == scan_profile_side(200u, 250u)) &&
                   (SCAN_PROFILE_LEFT == scan_profile_side(250u, 200u)),
               "one side under 250 mm: the other side");
    test_check(SCAN_PROFILE_NONE == scan_profile_side(249u, 200u),
               "both sides under 250 mm: no side (safe stop)");

    test_check((12u == sizeof(scan_profile_t)) &&
                   (20u == sizeof(ultrasonic_scan_t)) &&
                   (8u == sizeof(obstacle_avoidance_status_t)),
               "packed profile 12, scan 20, avoidance status 8 bytes");

    printf("%s: %d failure(s)\n", (0 == g_failures) ? "ALL PASS" : "FAILED",
           g_failures);
    return (0 == g_failures) ? 0 : 1;
}

/*** end of file ***/

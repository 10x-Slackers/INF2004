/** @file test_profile.c
 *
 * @brief Host test of scan_profile.c: echo conversion, median, profile and
 * side choice, and the sizes of subsystem 5's packed records. Build and
 * run as in S5-U43 of testing/subsystem5/TEST_LOG.md.
 */

#include "../../../app_program/subsystem5/obstacle_avoidance.h"
#include "../../../app_program/subsystem5/scan_profile.h"
#include "../../../app_program/subsystem5/ultrasonic_scan.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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

/* Rays from a list of (angle, mm) pairs; other angles not taken. */
static void test_rays(uint16_t * p_rays, const uint16_t * p_pairs,
                      uint32_t count)
{
    uint32_t i = 0u;

    memset(p_rays, 0, SCAN_PROFILE_RAYS * sizeof(uint16_t));
    for (i = 0u; i < count; i++)
    {
        p_rays[(p_pairs[2u * i] - SCAN_PROFILE_MIN_DEG) /
               SCAN_PROFILE_STEP_DEG] = p_pairs[(2u * i) + 1u];
    }
}

/* Profile of five coarse rays at 30, 60, 90, 120 and 150 degrees. */
static scan_profile_t test_coarse(uint16_t r30, uint16_t r60, uint16_t r90,
                                  uint16_t r120, uint16_t r150)
{
    const uint16_t pairs[] = {30u, r30, 60u, r60, 90u, r90, 120u, r120,
                              150u, r150};
    uint16_t rays[SCAN_PROFILE_RAYS];
    scan_profile_t profile;

    test_rays(rays, pairs, 5u);
    scan_profile_build(rays, &profile);
    return profile;
}

int main(void)
{
    uint16_t rays[SCAN_PROFILE_RAYS];
    scan_profile_t p;
    const uint16_t fine[] = {30u,  800u, 60u,  800u, 75u,  800u, 80u, 300u,
                             85u,  250u, 90u,  200u, 95u,  220u, 100u, 330u,
                             105u, 800u, 120u, 800u, 150u, 800u};
    const uint16_t gap[] = {30u, 800u, 60u, 800u, 90u, 200u, 120u, 300u,
                            150u, 900u};

    test_check((1000u == scan_profile_mm(5800u)) &&
                   (200u == scan_profile_mm(1160u)) &&
                   (SCAN_PROFILE_FAR_MM == scan_profile_mm(30000u)),
               "echo time to mm; past 4 m reads as 4000 (nothing there)");
    test_check((5u == scan_profile_median(9u, 5u, 1u)) &&
                   (5u == scan_profile_median(5u, 1u, 9u)) &&
                   (5u == scan_profile_median(1u, 9u, 5u)),
               "median of three");

    p = test_coarse(800u, 800u, 200u, 800u, 800u);
    test_check((200u == p.closest_mm) && (90u == p.closest_deg) &&
                   (1u == p.b_obstacle) && (90u == p.right_deg) &&
                   (90u == p.left_deg) && (800u == p.right_mm) &&
                   (800u == p.left_mm),
               "centred obstacle: closest point, edges, both sides 800 mm");
    test_check(SCAN_PROFILE_LEFT == scan_profile_side(&p),
               "both sides open: left on a tie");

    p = test_coarse(900u, 800u, 200u, 300u, 300u);
    test_check((150u == p.left_deg) && (0u == p.left_mm) &&
                   (SCAN_PROFILE_RIGHT == scan_profile_side(&p)),
               "blocked to the left edge of the sweep: pass on the right");

    p = test_coarse(400u, 400u, 200u, 400u, 450u);
    test_check(SCAN_PROFILE_NONE == scan_profile_side(&p),
               "both sides under 500 mm: no side (safe stop)");

    p = test_coarse(4000u, 4000u, 4000u, 4000u, 4000u);
    test_check((0u == p.b_obstacle) && (4000u == p.closest_mm),
               "open space: no obstacle");

    p = test_coarse(200u, 800u, 800u, 800u, 800u);
    test_check((30u == p.right_deg) && (0u == p.right_mm) &&
                   (SCAN_PROFILE_LEFT == scan_profile_side(&p)),
               "obstacle at the right end: no rays right of it");

    test_rays(rays, fine, 11u);
    scan_profile_build(rays, &p);
    test_check((200u == p.closest_mm) && (80u == p.right_deg) &&
                   (100u == p.left_deg) && (800u == p.right_mm) &&
                   (800u == p.left_mm),
               "fine rays narrow the edges to 80..100 degrees");

    test_rays(rays, gap, 5u);
    scan_profile_build(rays, &p);
    test_check((120u == p.left_deg) && (900u == p.left_mm),
               "angles not taken do not split the obstacle");

    memset(rays, 0, sizeof(rays));
    scan_profile_build(rays, &p);
    test_check((0u == p.closest_mm) && (0u == p.b_obstacle) &&
                   (SCAN_PROFILE_NONE == scan_profile_side(&p)),
               "no rays: empty profile, no side");

    test_check((8u == sizeof(scan_profile_t)) &&
                   (16u == sizeof(ultrasonic_scan_t)) &&
                   (4u == sizeof(obstacle_avoidance_status_t)),
               "packed profile 8, scan 16, avoidance status 4 bytes");

    printf("%s: %d failure(s)\n", (0 == g_failures) ? "ALL PASS" : "FAILED",
           g_failures);
    return (0 == g_failures) ? 0 : 1;
}

/*** end of file ***/

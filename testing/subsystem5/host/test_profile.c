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

/* Profile of the five rays at 80, 85, 90, 95 and 100 degrees. */
static scan_profile_t test_profile(uint16_t r80, uint16_t r85, uint16_t r90,
                                   uint16_t r95, uint16_t r100)
{
    const uint16_t rays[SCAN_PROFILE_RAYS] = {r80, r85, r90, r95, r100};
    scan_profile_t profile = {0};

    scan_profile_build(rays, &profile);
    return profile;
}

int main(void)
{
    scan_profile_t p;

    test_check((1000u == scan_profile_mm(5800u)) &&
                   (200u == scan_profile_mm(1160u)) &&
                   (SCAN_PROFILE_FAR_MM == scan_profile_mm(30000u)),
               "echo time to mm; past 4 m reads as 4000 (nothing there)");
    test_check((5u == scan_profile_median(9u, 5u, 1u)) &&
                   (5u == scan_profile_median(5u, 1u, 9u)) &&
                   (5u == scan_profile_median(1u, 9u, 5u)),
               "median of three");

    p = test_profile(800u, 300u, 200u, 250u, 800u);
    test_check((200u == p.closest_mm) && (90u == p.closest_deg) &&
                   (1u == p.b_obstacle) && (85u == p.right_deg) &&
                   (95u == p.left_deg),
               "centred obstacle: closest point and edges at 85..95");

    p = test_profile(200u, 800u, 800u, 300u, 300u);
    test_check((200u == p.closest_mm) && (80u == p.closest_deg) &&
                   (80u == p.right_deg) && (80u == p.left_deg),
               "obstacle at the right end; a further blocked run is not it");

    p = test_profile(300u, 300u, 300u, 300u, 250u);
    test_check((100u == p.closest_deg) && (80u == p.right_deg) &&
                   (100u == p.left_deg),
               "obstacle filling the sweep: edges at both ends");

    p = test_profile(4000u, 4000u, 4000u, 4000u, 4000u);
    test_check((0u == p.b_obstacle) && (4000u == p.closest_mm),
               "open space: no obstacle");

    test_check((SCAN_PROFILE_LEFT == scan_profile_side(800u, 800u)) &&
                   (SCAN_PROFILE_LEFT == scan_profile_side(900u, 600u)) &&
                   (SCAN_PROFILE_RIGHT == scan_profile_side(600u, 900u)),
               "the more open side, left on a tie");
    test_check((SCAN_PROFILE_RIGHT == scan_profile_side(300u, 400u)) &&
                   (SCAN_PROFILE_LEFT == scan_profile_side(400u, 300u)),
               "one side under 400 mm: the other side");
    test_check(SCAN_PROFILE_NONE == scan_profile_side(399u, 300u),
               "both sides under 400 mm: no side (safe stop)");

    test_check((8u == sizeof(scan_profile_t)) &&
                   (16u == sizeof(ultrasonic_scan_t)) &&
                   (8u == sizeof(obstacle_avoidance_status_t)),
               "packed profile 8, scan 16, avoidance status 8 bytes");

    printf("%s: %d failure(s)\n", (0 == g_failures) ? "ALL PASS" : "FAILED",
           g_failures);
    return (0 == g_failures) ? 0 : 1;
}

/*** end of file ***/

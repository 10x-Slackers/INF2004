/** @file subsystem4_test.c
 *
 * @brief On-target tests for subsystem 4 (make TEST=1). Record results in
 * testing/subsystem4/TEST_LOG.md.
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST
#define S4_TEST_POLLS      30  /* Wait up to 3 s for calibration... */
#define S4_TEST_POLL_MS    100 /* ...polling every 100 ms. */
#define S4_TEST_WINDOW_MS  500
#define S4_TEST_MIN_READS  3u  /* Readings in the window: some... */
#define S4_TEST_LOW_READS  45u /* ...and one every 10 ms, +-10%. */
#define S4_TEST_TOP_READS  55u
#define S4_TEST_FLAT_MG    200 /* Flat: x and y within this of 0... */
#define S4_TEST_Z_LOW_MG   800 /* ...and z between these. */
#define S4_TEST_Z_TOP_MG   1200
#define S4_TEST_COUNT_MAX  255u /* Largest 8-bit reading count. */
#define S4_TEST_HUMPS_MAX  15u  /* Largest 4-bit hump count. */
#define S4_TEST_ACCEL_SIZE 8u
#define S4_TEST_WORD_SIZE  4u

void subsystem4_test(void)
{
    s4_accel_t before = {0};
    s4_accel_t after = {0};
    s4_accel_t packed = {0};
    s4_tilt_t tilt = {0};
    s4_hump_t hump = {0};
    s4_motion_t motion = {0};
    uint8_t reads = 0u;

    TEST_INIT();

    /* Readings start once start-up calibration (2 s, board still) ends. */
    for (int32_t poll = 0; (poll < S4_TEST_POLLS) && !tilt.cal; poll++)
    {
        (void)tk_dly_tsk(S4_TEST_POLL_MS);
        s4_get_tilt(&tilt);
    }
    TEST_CHECK("S4-U05", tilt.cal, "start-up calibration ends within 3 s");

    s4_get_accel(&before);
    (void)tk_dly_tsk(S4_TEST_WINDOW_MS);
    s4_get_accel(&after);
    reads = (uint8_t)(after.count - before.count);
    TEST_CHECK("S4-U01", reads >= S4_TEST_MIN_READS,
               "GY-511 answers on I2C0 and readings arrive");
    TEST_CHECK("S4-U06",
               (reads >= S4_TEST_LOW_READS) && (reads <= S4_TEST_TOP_READS),
               "a reading every 10 ms (45..55 in 500 ms)");

    /* Board lying flat, component side up: gravity on +Z only. */
    TEST_CHECK("S4-U02",
               (after.z > S4_TEST_Z_LOW_MG) && (after.z < S4_TEST_Z_TOP_MG) &&
                   (after.x > -S4_TEST_FLAT_MG) &&
                   (after.x < S4_TEST_FLAT_MG) &&
                   (after.y > -S4_TEST_FLAT_MG) && (after.y < S4_TEST_FLAT_MG),
               "flat: z ~ +1000 mg, x and y ~ 0");

    /* The packed fields hold the sensor's full range; count wraps. */
    packed.x = -S4_AXIS_MAX_MG - 1;
    packed.y = S4_AXIS_MAX_MG;
    packed.z = -1;
    packed.count = S4_TEST_COUNT_MAX;
    packed.count++;
    TEST_CHECK("S4-U04",
               (S4_TEST_ACCEL_SIZE == sizeof(s4_accel_t)) &&
                   (-S4_AXIS_MAX_MG - 1 == packed.x) &&
                   (S4_AXIS_MAX_MG == packed.y) && (-1 == packed.z) &&
                   (0u == packed.count),
               "packed reading keeps -2048..2047 per axis in 8 bytes");

    /* Board still since power-on: no fault latched. */
    TEST_CHECK("S4-U14", 0u == (app_faults() & APP_FAULT_IMU),
               "no impact or tilt fault on a still board");
    hump.peak = S4_HUMP_MAX_MM;
    hump.count = S4_TEST_HUMPS_MAX;
    hump.count++;
    TEST_CHECK("S4-U13",
               (S4_TEST_WORD_SIZE == sizeof(s4_hump_t)) &&
                   (S4_HUMP_MAX_MM == hump.peak) && (0u == hump.count),
               "packed hump record keeps 0..63 mm and a wrapping count");

    /* Board still since power-on: steady. */
    s4_get_motion(&motion);
    TEST_CHECK("S4-U18",
               (S4_TEST_WORD_SIZE == sizeof(s4_motion_t)) &&
                   (S4_STEADY == motion.cls),
               "motion is steady on a still board");

    TEST_SUMMARY("subsystem4");
}
#endif /* APP_TEST */

/*** end of file ***/

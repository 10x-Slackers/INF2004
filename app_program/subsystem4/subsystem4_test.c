/*
 *      subsystem4_test.c
 *      On-target tests for subsystem 4.  Built only with: make TEST=1
 *      Record results in testing/subsystem4/TEST_LOG.md.  IDs: S4-U01, ...
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
    S4_ACCEL a, b;
    S4_ACCEL t = {0};

    TEST_INIT();

    s4_get_accel(&a);
    tk_dly_tsk(5 * S4_IMU_MS); /* let imu_task take a few readings */
    s4_get_accel(&b);
    TEST_CHECK("S4-U01", (UB)(b.count - a.count) >= 3,
               "GY-511 answers on I2C0 and readings arrive");

    /* Board lying flat, component side up: gravity on +Z only */
    TEST_CHECK("S4-U02",
               b.z > 800 && b.z < 1200 && b.x > -200 && b.x < 200 &&
                   b.y > -200 && b.y < 200,
               "flat: z ~ +1000 mg, x and y ~ 0");

    /* The packed fields hold the sensor's full range; count wraps */
    t.x = -2048;
    t.y = 2047;
    t.z = -1;
    t.count = 255;
    t.count++;
    TEST_CHECK("S4-U04",
               sizeof(S4_ACCEL) == 8 && t.x == -2048 && t.y == 2047 &&
                   t.z == -1 && t.count == 0,
               "packed reading keeps -2048..2047 per axis in 8 bytes");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

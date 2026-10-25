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
    UW before = s4_reads_ok;

    TEST_INIT();

    tk_dly_tsk(5 * S4_IMU_MS); /* let imu_task take a few readings */
    TEST_CHECK("S4-U01", s4_reads_ok >= before + 3,
               "GY-511 answers on I2C0 and readings arrive");

    /* Board lying flat, component side up: gravity on +Z only */
    TEST_CHECK("S4-U02",
               s4_accel.z > 800 && s4_accel.z < 1200 && s4_accel.x > -200 &&
                   s4_accel.x < 200 && s4_accel.y > -200 && s4_accel.y < 200,
               "flat: z ~ +1000 mg, x and y ~ 0");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

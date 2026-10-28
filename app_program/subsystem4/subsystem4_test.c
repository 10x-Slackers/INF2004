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
    S4_TILT tilt;
    S4_FAULT fault;
    S4_HUMP hump = {0};
    INT i;

    TEST_INIT();

    /* Readings start once start-up calibration (2 s, board still) ends */
    for (i = 0; i < 30; i++)
    {
        s4_get_tilt(&tilt);
        if (tilt.cal)
        {
            break;
        }
        tk_dly_tsk(100);
    }
    TEST_CHECK("S4-U05", tilt.cal, "start-up calibration ends within 3 s");

    s4_get_accel(&a);
    tk_dly_tsk(500);
    s4_get_accel(&b);
    TEST_CHECK("S4-U01", (UB)(b.count - a.count) >= 3,
               "GY-511 answers on I2C0 and readings arrive");
    TEST_CHECK("S4-U06",
               (UB)(b.count - a.count) >= 45 && (UB)(b.count - a.count) <= 55,
               "a reading every 10 ms (45..55 in 500 ms)");

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

    /* Board still since power-on: no fault latched */
    s4_get_fault(&fault);
    TEST_CHECK("S4-U14", sizeof(S4_FAULT) == 4 && !fault.impact && !fault.tilt,
               "no impact or tilt fault on a still board");
    hump.peak = 63;
    hump.count = 15;
    hump.count++;
    TEST_CHECK("S4-U13",
               sizeof(S4_HUMP) == 4 && hump.peak == 63 && hump.count == 0,
               "packed hump record keeps 0..63 mm and a wrapping count");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

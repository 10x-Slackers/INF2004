/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3 (make TEST=1). Afterwards the
 * bench task prints the line state every time it changes, for the manual
 * tests in testing/subsystem3/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST
#define S3_TEST_WAIT_MS  100 /* Ten sampling periods. */
#define S3_BENCH_POLL_MS 10

static void subsystem3_bench_task(INT stacd, void * p_exinf);

void subsystem3_test(void)
{
    line_following_t line = {0};
    uint32_t count = 0u;

    TEST_INIT();
    line_following_get(&line);
    count = line.count;
    tk_dly_tsk(S3_TEST_WAIT_MS);
    line_following_get(&line);
    TEST_CHECK("S3-U01", count != line.count,
               "the line sensors are sampled within 100 ms");
    TEST_CHECK("S3-U02", 8u == sizeof(line), "packed line state 8 bytes");
    TEST_SUMMARY("subsystem3");

    /* FP is the kernel's generic task pointer type. */
    (void)app_start_task((FP)subsystem3_bench_task, S3_PRI_BENCH, 0,
                         "s3_bench");
}

/*!
 * @brief Bench task: print the line state on every change.
 */
static void subsystem3_bench_task(INT stacd, void * p_exinf)
{
    line_following_t line = {0};
    line_following_t last = {0};

    (void)stacd;
    (void)p_exinf;
    line_following_get(&last);
    last.bits = ~last.bits; /* Print the first state. */
    for (;;)
    {
        line_following_get(&line);
        if ((line.bits != last.bits) || (line.b_junction != last.b_junction))
        {
            /* (UB *): tm_printf() takes unsigned chars. */
            tm_printf((UB *)"[line] left=%u right=%u junction=%u last=%u\n",
                      line.bits & DRV_LINE_LEFT,
                      (line.bits & DRV_LINE_RIGHT) >> 1, line.b_junction,
                      line.last_side);
            last = line;
        }
        tk_dly_tsk(S3_BENCH_POLL_MS);
    }
}
#endif /* APP_TEST */

/*** end of file ***/

/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3 (make TEST=1). The checks do not
 * move a motor. Afterwards the bench task prints the line state on every
 * change and follows the line after a short GP20 press, for the manual
 * tests in testing/subsystem3/TEST_LOG.md. GP20 is subsystem 5's bench
 * button too: a short press also makes it do one servo scan.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST
#define S3_TEST_WAIT_MS     100 /* Ten sampling periods. */
#define S3_BENCH_POLL_MS    10
#define S3_BENCH_BUTTON     20u  /* Robo Pico button, low when pressed. */
#define S3_BENCH_GO_MS      5000 /* Time to put the car on the line. */
#define S3_BENCH_RUN_MS     15000u
#define S3_BENCH_SPEED_MM_S 200u

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
    TEST_CHECK("S3-U04",
               !line_following_start(0u) &&
                   !line_following_start(LINE_FOLLOWING_MAX_MM_S + 1u),
               "line following rejects speeds out of range");
    line_following_get(&line);
    TEST_CHECK("S3-U05", !line.b_following, "line following boots stopped");
    TEST_SUMMARY("subsystem3");

    /* FP is the kernel's generic task pointer type. */
    (void)app_start_task((FP)subsystem3_bench_task, S3_PRI_BENCH, 0,
                         "s3_bench");
}

/*!
 * @brief Print the line state if it changed since *p_last.
 *
 * @return True if both sensors see a junction.
 */
static bool subsystem3_bench_report(line_following_t * p_last)
{
    line_following_t line = {0};

    line_following_get(&line);
    if ((line.bits != p_last->bits) || (line.b_junction != p_last->b_junction))
    {
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[line] left=%u right=%u junction=%u last=%u "
                        "following=%u\n",
                  line.bits & DRV_LINE_LEFT, (line.bits & DRV_LINE_RIGHT) >> 1,
                  line.b_junction, line.last_side, line.b_following);
        *p_last = line;
    }
    return line.b_junction;
}

/*!
 * @brief Bench task. Prints the line state on every change. A short GP20
 * press starts a run: after 5 s the car follows the line at 200 mm/s
 * until a junction or for 15 s, then stops.
 */
static void subsystem3_bench_task(INT stacd, void * p_exinf)
{
    line_following_t last = {0};
    uint32_t run_ms = 0u;
    bool b_pressed = false;
    bool b_was_pressed = false;

    (void)stacd;
    (void)p_exinf;

    /* Pull-up only (both on is bus keeper): the button grounds the pin. */
    (void)gpio_set_pin(S3_BENCH_BUTTON, GPIO_MODE_IN);
    clr_w(GPIO(S3_BENCH_BUTTON), GPIO_PDE);
    set_w(GPIO(S3_BENCH_BUTTON), GPIO_PUE);
    last.bits = DRV_LINE_LEFT | DRV_LINE_RIGHT;
    last.b_junction = true; /* Unlikely at power-up: print the first state. */
    tm_printf((UB *)"[line] bench: press GP20 briefly to follow the line\n");
    for (;;)
    {
        (void)subsystem3_bench_report(&last);
        b_pressed = (0u == gpio_get_val(S3_BENCH_BUTTON));
        if (b_was_pressed && !b_pressed)
        {
            tm_printf((UB *)"[line] put the car on the line, starting in "
                            "5 s\n");
            tk_dly_tsk(S3_BENCH_GO_MS);
            (void)line_following_start(S3_BENCH_SPEED_MM_S);
            for (run_ms = 0u;
                 (run_ms < S3_BENCH_RUN_MS) && !subsystem3_bench_report(&last);
                 run_ms += S3_BENCH_POLL_MS)
            {
                tk_dly_tsk(S3_BENCH_POLL_MS);
            }
            line_following_stop();
            tm_printf((UB *)"[line] bench done after %u ms\n", run_ms);
        }
        b_was_pressed = b_pressed;
        tk_dly_tsk(S3_BENCH_POLL_MS);
    }
}
#endif /* APP_TEST */

/*** end of file ***/

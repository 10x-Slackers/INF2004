/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3 (make TEST=1). The checks do not
 * move a motor. Afterwards the bench task prints the line state on every
 * change and follows the line after GP21 is held for 1 s, for the manual
 * tests in testing/subsystem3/TEST_LOG.md. It also prints the barcode
 * sensor's level every 0.5 s and every barcode read. A tap under 0.5 s is
 * subsystem 2's bench; a press in between starts neither, so the two never
 * both start, even with their 20 ms polls out of step.
 */

#include "subsystem3.h"
#include "code39.h"
#include "../test_report.h"

#if APP_TEST
#define S3_TEST_WAIT_MS     100 /* Ten sampling periods. */
#define S3_TEST_NARROW      48u /* 3 mm in 1/16 mm. */
#define S3_TEST_WIDE        144u
#define S3_TEST_A           0x109u /* Code 39 patterns, 1 = wide. */
#define S3_TEST_Z           0x0D0u
#define S3_TEST_START       0x094u /* '*' */
#define S3_TEST_FIRST_GAP   9u
#define S3_BENCH_POLL_MS    10 /* Waits 2 kernel ticks: a poll every 20 ms. */
#define S3_BENCH_STEP_MS    20
#define S3_BENCH_BUTTON     21u   /* Robo Pico button, low when pressed. */
#define S3_BENCH_HOLD_MS    1000u /* Subsystem 2 takes taps under 0.5 s. */
#define S3_BENCH_GO_MS      5000  /* Time to put the car on the line. */
#define S3_BENCH_RUN_MS     15000u
#define S3_BENCH_SPEED_MM_S 200u
#define S3_BENCH_RAW_POLLS  25u /* Print the barcode level every 0.5 s. */

static void subsystem3_bench_task(INT stacd, void * p_exinf);
static void subsystem3_symbol(uint16_t * p_widths, uint32_t pattern,
                              bool b_reversed);

void subsystem3_test(void)
{
    line_following_t line = {0};
    barcode_decoder_raw_t raw = {0};
    barcode_decoder_t barcode = {0};
    uint16_t widths[CODE39_ELEMENTS] = {0};
    uint32_t letter = 0u;
    uint32_t count = 0u;
    uint32_t samples = 0u;

    TEST_INIT();
    line_following_get(&line);
    barcode_decoder_get_raw(&raw);
    count = line.count;
    samples = raw.samples;
    tk_dly_tsk(S3_TEST_WAIT_MS);
    line_following_get(&line);
    barcode_decoder_get_raw(&raw);
    TEST_CHECK("S3-U01", count != line.count,
               "the line sensors are sampled within 100 ms");
    /* tk_dly_tsk(100) waits 100 to 110 ms. */
    TEST_CHECK("S3-U07",
               ((uint8_t)(line.count - count) >= 10u) &&
                   ((uint8_t)(line.count - count) <= 12u),
               "a sample every 10 ms (10..12 in 100 ms)");
    TEST_CHECK("S3-U02", 8u == sizeof(line), "packed line state 8 bytes");
    TEST_CHECK("S3-U04",
               !line_following_start(0u) &&
                   !line_following_start(LINE_FOLLOWING_MAX_MM_S + 1u),
               "line following rejects speeds out of range");
    line_following_get(&line);
    TEST_CHECK("S3-U05", !line.b_following, "line following boots stopped");
    /* 2 ksample/s over 100 to 110 ms, give or take one 10 ms read. */
    TEST_CHECK("S3-U08",
               ((uint16_t)(raw.samples - samples) >= 180u) &&
                   ((uint16_t)(raw.samples - samples) <= 240u),
               "barcode ADC at 2 ksample/s (180..240 in 100 ms)");
    TEST_CHECK("S3-U09", 8u == sizeof(raw), "packed raw barcode level 8 bytes");
    subsystem3_symbol(widths, S3_TEST_A, false);
    letter = code39_decode(widths);
    subsystem3_symbol(widths, S3_TEST_A, true);
    TEST_CHECK("S3-U11", ('A' == letter) && ('A' == code39_decode(widths)),
               "decodes *A* read either way");
    subsystem3_symbol(widths, S3_TEST_Z, false);
    letter = code39_decode(widths);
    subsystem3_symbol(widths, S3_TEST_A, false);
    widths[S3_TEST_FIRST_GAP] = S3_TEST_WIDE; /* A wide gap: misread. */
    TEST_CHECK("S3-U12", (0u == letter) && (0u == code39_decode(widths)),
               "rejects *Z* and a misread *A*");
    TEST_CHECK("S3-U13", 8u == sizeof(barcode), "packed barcode 8 bytes");
    TEST_SUMMARY("subsystem3");

    /* FP is the kernel's generic task pointer type. */
    (void)app_start_task((FP)subsystem3_bench_task, S3_PRI_BENCH, 0,
                         "s3_bench");
}

/*!
 * @brief Fill in the widths of *X*, X given by its Code 39 pattern.
 */
static void subsystem3_symbol(uint16_t * p_widths, uint32_t pattern,
                              bool b_reversed)
{
    uint32_t chars[3] = {S3_TEST_START, pattern, S3_TEST_START};
    uint32_t i = 0u;
    uint32_t at = 0u;

    for (i = 0u; i < CODE39_ELEMENTS; i++)
    {
        /* Every 10th element is the narrow gap between characters. */
        at = b_reversed ? (CODE39_ELEMENTS - 1u - i) : i;
        p_widths[at] = ((9u != (i % 10u)) &&
                        (0u != (chars[i / 10u] & (0x100u >> (i % 10u)))))
                           ? S3_TEST_WIDE
                           : S3_TEST_NARROW;
    }
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
 * @brief Bench task. Prints the line state on every change. Releasing GP21
 * after holding it for 1 s starts a run: after 5 s the car follows the
 * line at 200 mm/s until a junction or for 15 s, then stops.
 */
static void subsystem3_bench_task(INT stacd, void * p_exinf)
{
    line_following_t last = {0};
    uint32_t run_ms = 0u;
    uint32_t held_ms = 0u;
    uint32_t polls = 0u;
    barcode_decoder_raw_t raw = {0};
    barcode_decoder_t barcode = {0};
    barcode_decoder_t last_barcode = {0};

    (void)stacd;
    (void)p_exinf;

    app_pull_up(S3_BENCH_BUTTON);
    last.bits = DRV_LINE_LEFT | DRV_LINE_RIGHT;
    last.b_junction = true; /* Unlikely at power-up: print the first state. */
    tm_printf((UB *)"[line] bench: hold GP21 for %u s to follow the line\n",
              S3_BENCH_HOLD_MS / 1000u);
    for (;;)
    {
        (void)subsystem3_bench_report(&last);
        barcode_decoder_get(&barcode);
        if ((barcode.count != last_barcode.count) ||
            (barcode.rejects != last_barcode.rejects))
        {
            /* (UB *): tm_printf() takes unsigned chars. */
            tm_printf((UB *)"[barcode] command=%u count=%u rejects=%u "
                            "at_mm=%u\n",
                      barcode.command, barcode.count, barcode.rejects,
                      barcode.distance_mm);
            last_barcode = barcode;
        }
        if (++polls >= S3_BENCH_RAW_POLLS)
        {
            barcode_decoder_get_raw(&raw);
            /* (UB *): tm_printf() takes unsigned chars. */
            tm_printf((UB *)"[bar] min=%u max=%u\n", raw.min, raw.max);
            polls = 0u;
        }
        if (0u == gpio_get_val(S3_BENCH_BUTTON))
        {
            held_ms += S3_BENCH_STEP_MS;
        }
        else if (held_ms >= S3_BENCH_HOLD_MS)
        {
            held_ms = 0u;
            tm_printf((UB *)"[line] put the car on the line, starting in "
                            "%d s\n",
                      S3_BENCH_GO_MS / 1000);
            tk_dly_tsk(S3_BENCH_GO_MS);
            (void)line_following_start(S3_BENCH_SPEED_MM_S);
            for (run_ms = 0u;
                 (run_ms < S3_BENCH_RUN_MS) && !subsystem3_bench_report(&last);
                 run_ms += S3_BENCH_STEP_MS)
            {
                tk_dly_tsk(S3_BENCH_POLL_MS);
            }
            line_following_stop();
            tm_printf((UB *)"[line] bench done after %u ms\n", run_ms);
        }
        else
        {
            held_ms = 0u; /* Released before 1 s: not a run. */
        }
        tk_dly_tsk(S3_BENCH_POLL_MS);
    }
}
#endif /* APP_TEST */

/*** end of file ***/

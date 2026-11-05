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
#include "../subsystem2/motion_control.h"

#if APP_TEST
#define S3_TEST_WAIT_MS     100   /* Ten sampling periods. */
#define S3_TEST_NARROW      3000u /* 3 mm in um, as the decoder measures. */
#define S3_TEST_WIDE        9000u
#define S3_TEST_A           0x109u /* Code 39 patterns, 1 = wide. */
#define S3_TEST_Z           0x0D0u
#define S3_TEST_BAD         0x1E0u /* 4 wide: no character. */
#define S3_TEST_START       0x094u /* '*' */
#define S3_TEST_FIRST_GAP   9u
#define S3_BENCH_POLL_MS    10 /* Waits 2 kernel ticks: a poll every 20 ms. */
#define S3_BENCH_STEP_MS    20u
#define S3_BENCH_BUTTON     21u   /* Robo Pico button, low when pressed. */
#define S3_BENCH_HOLD_MS    1000u /* Subsystem 2 takes taps under 0.5 s. */
#define S3_BENCH_GO_MS      5000  /* Time to put the car on the line. */
#define S3_BENCH_RUN_MS     15000u
#define S3_BENCH_SPEED_MM_S 150u
#define S3_BENCH_RAW_POLLS  25u /* Print the barcode level every 0.5 s. */
#define S3_BENCH_AXLE_MM   110u /* As the Vehicle Controller's junction move. */
#define S3_BENCH_TURN_DEG  90u
#define S3_BENCH_UTURN_DEG 180u

static void subsystem3_bench_task(INT stacd, void * p_exinf);
static uint32_t subsystem3_bench_barcode(barcode_decoder_t * p_last);
static bool subsystem3_bench_turn(uint32_t command);
static bool subsystem3_bench_move(bool b_started);
static void subsystem3_symbol(uint16_t * p_widths, uint32_t pattern,
                              bool b_reversed);

/*!
 * @brief Run subsystem 3's on-target checks, then start the bench task.
 */
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
    /* tk_dly_tsk(100) waits 100 to 110 ms. The counters wrap; each cast
     * keeps the difference modulo the counter's width. */
    TEST_CHECK("S3-U07",
               ((uint8_t)(line.count - count) >= 10u) &&
                   ((uint8_t)(line.count - count) <= 12u),
               "a sample every 10 ms (10..12 in 100 ms)");
    TEST_CHECK("S3-U02", 8u == sizeof(line), "packed line state 8 bytes");
    TEST_CHECK("S3-U04",
               (!line_following_start(0u)) &&
                   (!line_following_start(LINE_FOLLOWING_MAX_MM_S + 1u)),
               "line following rejects speeds out of range");
    line_following_get(&line);
    TEST_CHECK("S3-U05", !line.b_following, "line following boots stopped");
    /* 2 ksample/s over 100 to 110 ms, give or take one 10 ms read. The
     * casts keep the wrapped difference, as above. */
    TEST_CHECK("S3-U08",
               ((uint16_t)(raw.samples - samples) >= 180u) &&
                   ((uint16_t)(raw.samples - samples) <= 240u),
               "barcode ADC position 2 ksample/s (180..240 in 100 ms)");
    TEST_CHECK("S3-U09", 8u == sizeof(raw), "packed raw barcode level 8 bytes");
    subsystem3_symbol(widths, S3_TEST_A, false);
    letter = code39_decode(widths);
    subsystem3_symbol(widths, S3_TEST_A, true);
    TEST_CHECK("S3-U11", ('A' == letter) && ('A' == code39_decode(widths)),
               "decodes *A* read either way");
    subsystem3_symbol(widths, S3_TEST_Z, false);
    TEST_CHECK("S3-U20", 'Z' == code39_decode(widths), "decodes *Z*");
    subsystem3_symbol(widths, S3_TEST_BAD, false);
    letter = code39_decode(widths);
    subsystem3_symbol(widths, S3_TEST_A, false);
    widths[S3_TEST_FIRST_GAP] = S3_TEST_WIDE; /* A wide gap: misread. */
    TEST_CHECK("S3-U12", (0u == letter) && (0u == code39_decode(widths)),
               "rejects a 4-wide letter and a misread *A*");
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
    uint32_t const chars[3] = {S3_TEST_START, pattern, S3_TEST_START};
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
 * @brief Print the line state, with the odometer, if it changed since
 * *p_last. The odometer shows how far the car runs with both sensors
 * white, to set LINE_FOLLOWING_LOST_MM.
 *
 * @return True if the line was lost. A junction is only printed.
 */
static bool subsystem3_bench_report(line_following_t * p_last)
{
    line_following_t line = {0};
    motion_control_status_t motion = {0};

    line_following_get(&line);
    if ((line.bits != p_last->bits) ||
        (line.b_junction != p_last->b_junction) ||
        (line.b_searching != p_last->b_searching) ||
        (line.b_lost != p_last->b_lost))
    {
        motion_control_get_status(&motion);
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[line] left=%u right=%u junction=%u last=%u "
                        "following=%u searching=%u lost=%u mm=%u deg=%u\n",
                  line.bits & DRV_LINE_LEFT, (line.bits & DRV_LINE_RIGHT) >> 1,
                  line.b_junction, line.last_side, line.b_following,
                  line.b_searching, line.b_lost, motion.distance_mm,
                  motion.heading_deg);
        *p_last = line;
    }
    return line.b_lost;
}

/*!
 * @brief Print a barcode read or rejected since *p_last.
 *
 * @return The command of a new valid barcode, else BARCODE_DECODER_NONE.
 */
static uint32_t subsystem3_bench_barcode(barcode_decoder_t * p_last)
{
    barcode_decoder_t barcode = {0};
    uint32_t command = BARCODE_DECODER_NONE;

    barcode_decoder_get(&barcode);
    if ((barcode.count != p_last->count) ||
        (barcode.rejects != p_last->rejects))
    {
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[barcode] command=%u count=%u rejects=%u at_mm=%u\n",
                  barcode.command, barcode.count, barcode.rejects,
                  barcode.distance_mm);
        command = (barcode.count != p_last->count) ? barcode.command
                                                   : BARCODE_DECODER_NONE;
        *p_last = barcode;
    }
    return command;
}

/*!
 * @brief Wait for a motion command to finish.
 *
 * @param[in] b_started The command was accepted.
 * @return True if it completed.
 */
static bool subsystem3_bench_move(bool b_started)
{
    motion_control_status_t motion = {0};

    motion_control_get_status(&motion);
    while (b_started && (MOTION_CONTROL_RUNNING == motion.state))
    {
        tk_dly_tsk(S3_BENCH_POLL_MS);
        motion_control_get_status(&motion);
    }
    return b_started && (MOTION_CONTROL_COMPLETE == motion.state);
}

/*!
 * @brief At a junction, do as the Vehicle Controller does: roll the axle
 * onto the junction, turn for the command (none for straight on), then
 * follow the line again.
 *
 * @param[in] command BARCODE_DECODER_... other than NONE.
 * @return False if a move failed; the car is then stopped.
 */
static bool subsystem3_bench_turn(uint32_t command)
{
    bool b_ok = false;

    /* (UB *): tm_printf() takes unsigned chars. */
    tm_printf((UB *)"[line] junction: command %u\n", command);
    line_following_stop();
    b_ok = subsystem3_bench_move(motion_control_move_forward(S3_BENCH_AXLE_MM));
    if (b_ok && (BARCODE_DECODER_LEFT == command))
    {
        b_ok =
            subsystem3_bench_move(motion_control_turn_left(S3_BENCH_TURN_DEG));
    }
    else if (b_ok && (BARCODE_DECODER_RIGHT == command))
    {
        b_ok =
            subsystem3_bench_move(motion_control_turn_right(S3_BENCH_TURN_DEG));
    }
    else if (b_ok && (BARCODE_DECODER_UTURN == command))
    {
        b_ok =
            subsystem3_bench_move(motion_control_turn_left(S3_BENCH_UTURN_DEG));
    }
    else
    {
        /* Straight on, or the roll forward failed. */
    }
    if (b_ok)
    {
        (void)line_following_start(S3_BENCH_SPEED_MM_S);
    }
    else
    {
        motion_control_stop();
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[line] junction move failed\n");
    }
    return b_ok;
}

/*!
 * @brief Bench task. Prints the line state on every change. Releasing GP21
 * after holding it for 1 s starts a run: after 5 s the car follows the
 * line at 150 mm/s until a lost line or 15 s, then stops. At the first
 * junction after a valid barcode it turns as the Vehicle Controller does;
 * it drives straight over other junctions.
 */
static void subsystem3_bench_task(INT stacd, void * p_exinf)
{
    line_following_t last = {0};
    uint32_t run_ms = 0u;
    uint32_t held_ms = 0u;
    uint32_t polls = 0u;
    uint32_t pending = BARCODE_DECODER_NONE; /* Command for the junction. */
    uint32_t command = BARCODE_DECODER_NONE;
    bool b_was_junction = false;
    bool b_stop = false;
    barcode_decoder_raw_t raw = {0};
    barcode_decoder_t last_barcode = {0};

    (void)stacd;
    (void)p_exinf;

    app_pull_up(S3_BENCH_BUTTON);
    last.bits = DRV_LINE_LEFT | DRV_LINE_RIGHT;
    last.b_junction = true; /* Unlikely at power-up: print the first state. */
    /* (UB *): tm_printf() takes unsigned chars. */
    tm_printf((UB *)"[line] bench: hold GP21 for %u s to follow the line\n",
              S3_BENCH_HOLD_MS / 1000u);
    for (;;)
    {
        (void)subsystem3_bench_report(&last);
        (void)subsystem3_bench_barcode(&last_barcode);
        polls++;
        if (polls >= S3_BENCH_RAW_POLLS)
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
            /* (UB *): tm_printf() takes unsigned chars. */
            tm_printf((UB *)"[line] put the car on the line, starting in "
                            "%d s\n",
                      S3_BENCH_GO_MS / 1000);
            tk_dly_tsk(S3_BENCH_GO_MS);
            (void)line_following_start(S3_BENCH_SPEED_MM_S);
            pending = BARCODE_DECODER_NONE;
            b_stop = false;
            for (run_ms = 0u; (run_ms < S3_BENCH_RUN_MS) && (!b_stop);
                 run_ms += S3_BENCH_STEP_MS)
            {
                b_was_junction = last.b_junction;
                b_stop = subsystem3_bench_report(&last);
                command = subsystem3_bench_barcode(&last_barcode);
                pending = (BARCODE_DECODER_NONE != command) ? command : pending;
                if ((!b_stop) && last.b_junction && (!b_was_junction) &&
                    (BARCODE_DECODER_NONE != pending))
                {
                    b_stop = !subsystem3_bench_turn(pending);
                    pending = BARCODE_DECODER_NONE;
                }
                tk_dly_tsk(S3_BENCH_POLL_MS);
            }
            line_following_stop();
            /* (UB *): tm_printf() takes unsigned chars. */
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

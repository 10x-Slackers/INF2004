/** @file subsystem2_test.c
 *
 * @brief On-target tests for subsystem 2 (make TEST=1). The checks do not
 * move a motor. Afterwards the bench task waits for a tap of the GP21
 * button, under 0.5 s (a hold of 1 s or more is subsystem 3's), then runs a
 * fixed move sequence for the manual tests in
 * testing/subsystem2/TEST_LOG.md.
 */

#include "subsystem2.h"
#include "drv_motion.h"
#include "../test_report.h"

#if APP_TEST
#define S2_TEST_WAIT_MS 200 /* 20 control periods. */
#define S2_TEST_TICK_US 1000u
#define S2_BENCH_BUTTON 21u  /* Robo Pico button, low when pressed. */
#define S2_BENCH_GO_MS  5000 /* Time to put the car down after a press. */
#define S2_BENCH_TAP_POLLS                                                     \
    25u                         /* Under 0.5 s; from 1 s it is subsystem 3's.  \
                                 */
#define S2_BENCH_PAUSE_MS  3000 /* Still after each step, to measure it. */
#define S2_BENCH_DRIVE_MS  2000
#define S2_BENCH_POLL_MS   10 /* Waits 2 kernel ticks: a line every 20 ms. */
#define S2_BENCH_STATUS_MS 1000
#define S2_BENCH_MM_S      100
#define S2_BENCH_FAST_MM_S 200u
#define S2_BENCH_MOVE_MM   300u
#define S2_BENCH_TURN_DEG  90u
#define S2_BENCH_UTURN_DEG 180u

static void subsystem2_bench_task(INT stacd, void * p_exinf);

void subsystem2_test(void)
{
    motion_control_status_t status = {0};
    uint8_t packet[MOTION_CONTROL_PACKET_BYTES] = {0};

    TEST_INIT();
    tk_dly_tsk(S2_TEST_WAIT_MS);
    motion_control_get_status(&status);
    TEST_CHECK("S2-U16",
               (MOTION_CONTROL_IDLE == status.state) &&
                   (MOTION_CONTROL_CMD_NONE == status.command),
               "boots idle with the motors braked");
    TEST_CHECK(
        "S2-U17",
        !motion_control_move_forward(0u) &&
            !motion_control_move_forward(MOTION_CONTROL_MAX_DISTANCE_MM + 1u) &&
            !motion_control_turn_left(MOTION_CONTROL_MAX_ANGLE_DEG + 1u) &&
            !motion_control_drive(MOTION_CONTROL_MAX_SPEED_MM_S + 1, 0),
        "out-of-range commands are rejected");
    motion_control_get_status(&status);
    TEST_CHECK("S2-U18", MOTION_CONTROL_IDLE == status.state,
               "a rejected command does not start the motors");
    motion_control_pack(&status, packet);
    TEST_CHECK("S2-U19",
               (MOTION_CONTROL_PACKET_BYTES == sizeof(status)) &&
                   (0u == packet[0]),
               "12-byte packed status and telemetry");
    TEST_CHECK("S2-U20",
               (subsystem2_max_tick_us() > 0u) &&
                   (subsystem2_max_tick_us() < S2_TEST_TICK_US),
               "control step runs and takes under 1 ms");
    (void)motion_control_latch(true);
    TEST_CHECK("S2-U28",
               motion_control_latched() && !motion_control_move_forward(100u) &&
                   !motion_control_turn_left(90u) &&
                   !motion_control_drive(100, 100),
               "a latched safety stop refuses every command");
    (void)motion_control_latch(false);
    TEST_CHECK("S2-U29", motion_control_drive(0, 0),
               "commands are accepted again after the latch is released");
    motion_control_stop();
    TEST_SUMMARY("subsystem2");

    /* FP is the kernel's generic task pointer type. */
    (void)app_start_task((FP)subsystem2_bench_task, S2_PRI_BENCH, TP_PRC1,
                         "s2_bench");
}

/*!
 * @brief Print the motion status on one line.
 */
static void subsystem2_bench_report(void)
{
    motion_control_status_t status = {0};

    motion_control_get_status(&status);
    /* (UB *): tm_printf() takes unsigned chars. */
    tm_printf((UB *)"[motion] state=%u cmd=%u faults=%u heading=%u "
                    "dist_mm=%u counts=%u,%u speed=%d,%d max_tick_us=%u\n",
              status.state, status.command, app_faults() & APP_FAULT_MOTION,
              status.heading_deg, status.distance_mm, status.left_count,
              status.right_count, status.left_speed, status.right_speed,
              subsystem2_max_tick_us());
}

/*!
 * @brief Print the wheel speeds every 20 ms until the command ends, then
 * the status. With b_drive, renew drive(left, right) for up to 2 s and
 * stop after the status, so a stall still shows in it.
 */
static void subsystem2_bench_run(bool b_drive, int32_t left, int32_t right)
{
    motion_control_status_t status = {0};
    SYSTIM now = {0};
    uint32_t start_ms = 0u;

    (void)tk_get_otm(&now);
    start_ms = now.lo;
    do
    {
        if (b_drive)
        {
            (void)motion_control_drive(left, right);
        }
        tk_dly_tsk(S2_BENCH_POLL_MS);
        motion_control_get_status(&status);
        (void)tk_get_otm(&now);
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[trace] %u %d %d\n", now.lo, status.left_speed,
                  status.right_speed);
    } while ((MOTION_CONTROL_RUNNING == status.state) &&
             (!b_drive || ((now.lo - start_ms) < S2_BENCH_DRIVE_MS)));
    subsystem2_bench_report();
    if (b_drive)
    {
        motion_control_stop();
    }
    tk_dly_tsk(S2_BENCH_PAUSE_MS);
}

/*!
 * @brief Bench task: after a GP21 tap, the fixed move sequence with a
 * pause after each step, then the status every 1 s.
 */
static void subsystem2_bench_task(INT stacd, void * p_exinf)
{
    uint32_t polls = S2_BENCH_TAP_POLLS;

    (void)stacd;
    (void)p_exinf;

    app_pull_up(S2_BENCH_BUTTON);
    tm_printf((UB *)"[motion] bench: tap GP21 to start\n");
    while (polls >= S2_BENCH_TAP_POLLS)
    {
        polls = 0u;
        tk_dly_tsk(S2_BENCH_POLL_MS); /* Let a release bounce settle. */
        while (0u != gpio_get_val(S2_BENCH_BUTTON))
        {
            tk_dly_tsk(S2_BENCH_POLL_MS);
        }
        while (0u == gpio_get_val(S2_BENCH_BUTTON))
        {
            tk_dly_tsk(S2_BENCH_POLL_MS);
            polls++;
        }
    }
    tk_dly_tsk(S2_BENCH_GO_MS);
    tm_printf((UB *)"[motion] bench: drive +-100, forward/backward 300, "
                    "left 90, right 90, left 180, forward 300 at 200, "
                    "left 180, forward 300\n");
    subsystem2_bench_run(true, S2_BENCH_MM_S, S2_BENCH_MM_S);
    subsystem2_bench_run(true, -S2_BENCH_MM_S, -S2_BENCH_MM_S);
    (void)motion_control_move_forward(S2_BENCH_MOVE_MM);
    subsystem2_bench_run(false, 0, 0);
    (void)motion_control_move_backward(S2_BENCH_MOVE_MM);
    subsystem2_bench_run(false, 0, 0);
    (void)motion_control_turn_left(S2_BENCH_TURN_DEG);
    subsystem2_bench_run(false, 0, 0);
    (void)motion_control_turn_right(S2_BENCH_TURN_DEG);
    subsystem2_bench_run(false, 0, 0);
    (void)motion_control_turn_left(S2_BENCH_UTURN_DEG);
    subsystem2_bench_run(false, 0, 0);
    (void)motion_control_set_speed(S2_BENCH_FAST_MM_S);
    (void)motion_control_move_forward(S2_BENCH_MOVE_MM);
    subsystem2_bench_run(false, 0, 0);
    (void)motion_control_set_speed((uint32_t)S2_BENCH_MM_S);
    (void)motion_control_turn_left(S2_BENCH_UTURN_DEG); /* Back to start. */
    subsystem2_bench_run(false, 0, 0);
    (void)motion_control_move_forward(S2_BENCH_MOVE_MM);
    subsystem2_bench_run(false, 0, 0);
    tm_printf((UB *)"[motion] bench done\n");
    for (;;)
    {
        subsystem2_bench_report();
        tk_dly_tsk(S2_BENCH_STATUS_MS);
    }
}
#endif

/*** end of file ***/

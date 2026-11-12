/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5 (make TEST=1). The checks do not
 * move a motor. Afterwards the bench task waits for the GP20 button: each
 * short press prints a servo profile scan, a press held for 1 s starts
 * avoidance and drives forward, for the manual tests in
 * testing/subsystem5/TEST_LOG.md.
 */

#include "subsystem5.h"
#include "../subsystem2/motion_control.h"
#include "../test_report.h"

#if APP_TEST
#define S5_TEST_SCAN_MS   500  /* A forward reading takes about 60 ms. */
#define S5_BENCH_BUTTON   20u  /* Robo Pico button, low when pressed. */
#define S5_BENCH_GO_MS    5000 /* Time to put the car down after a press. */
#define S5_BENCH_POLL_MS  20u
#define S5_BENCH_HOLD_MS  1000u /* A press this long starts the drive. */
#define S5_BENCH_DRIVE_MM 1500u
#define S5_BENCH_LOG      32u /* Records kept from the last run. */

/* One status change of a run, kept to print again over USB afterwards. */
typedef struct
{
    ultrasonic_scan_t scan;
    obstacle_avoidance_status_t status;
    uint32_t faults;
    uint32_t motion;
    uint32_t ms; /* Since the run started. */
} s5_bench_record_t;

static s5_bench_record_t g_log[S5_BENCH_LOG]; /* Bench task only. */
static uint32_t g_logged;

static void subsystem5_bench_task(INT stacd, void * p_exinf);

/*!
 * @brief Run subsystem 5's on-target tests, then start the bench task.
 */
void subsystem5_test(void)
{
    obstacle_avoidance_status_t status = {0};
    ultrasonic_scan_t scan = {0};
    uint32_t request = 0u;

    TEST_INIT();
    obstacle_avoidance_get_status(&status);
    TEST_CHECK("S5-U40", OBSTACLE_AVOIDANCE_IDLE == status.state,
               "avoidance boots idle");
    request = ultrasonic_scan_request(ULTRASONIC_SCAN_FORWARD);
    tk_dly_tsk(S5_TEST_SCAN_MS);
    ultrasonic_scan_get(&scan);
    TEST_CHECK("S5-U41", request == scan.request,
               "a forward scan finishes within 500 ms");
    /* (UB *): tm_printf() takes unsigned chars. */
    tm_printf((UB *)"[scan] forward_mm=%u fault=%u\n", scan.forward_mm,
              (0u != (app_faults() & APP_FAULT_NO_ECHO)));
    TEST_CHECK("S5-U42",
               (8u == sizeof(status)) && (20u == sizeof(scan)) &&
                   (12u == sizeof(scan_profile_t)),
               "packed status 8, scan 20, profile 12 bytes");
    TEST_SUMMARY("subsystem5");

    /* FP is the kernel's generic task pointer type. */
    (void)app_start_task((FP)subsystem5_bench_task, S5_PRI_BENCH, 0u,
                         "s5_bench");
}

/*!
 * @brief Wait for a GP20 press and release.
 *
 * @return True if the press was held for S5_BENCH_HOLD_MS.
 */
static bool subsystem5_bench_press(void)
{
    uint32_t held = 0u;

    while (0u != gpio_get_val(S5_BENCH_BUTTON))
    {
        tk_dly_tsk(S5_BENCH_POLL_MS);
    }
    while (0u == gpio_get_val(S5_BENCH_BUTTON))
    {
        tk_dly_tsk(S5_BENCH_POLL_MS);
        held += S5_BENCH_POLL_MS;
    }
    return (held >= S5_BENCH_HOLD_MS);
}

/*!
 * @brief Print one record: the scan, and the avoidance status with the
 * motion state, faults and time into the run, on two lines.
 */
static void subsystem5_bench_print(const s5_bench_record_t * p_rec)
{
    /* (UB *): tm_printf() takes unsigned chars. */
    tm_printf((UB *)"[scan] forward_mm=%u fault=%u closest=%u@%u "
                    "edges=%u..%u width=%u obstacle=%u clear=%u/%u\n",
              p_rec->scan.forward_mm,
              (0u != (p_rec->faults & APP_FAULT_NO_ECHO)),
              p_rec->scan.profile.closest_mm, p_rec->scan.profile.closest_deg,
              p_rec->scan.profile.right_deg, p_rec->scan.profile.left_deg,
              p_rec->scan.profile.width_mm, p_rec->scan.profile.b_obstacle,
              p_rec->scan.profile.right_mm, p_rec->scan.profile.left_mm);
    tm_printf((UB *)"[avoid] t=%u ms state=%u faults=0x%x side=%u step=%u "
                    "bypasses=%u right_mm=%u left_mm=%u motion=%u\n",
              p_rec->ms, p_rec->status.state, p_rec->faults & APP_FAULT_AVOID,
              p_rec->status.side, p_rec->status.step, p_rec->status.bypasses,
              p_rec->status.right_mm, p_rec->status.left_mm, p_rec->motion);
}

/*!
 * @brief Print the latest scan and status. During a run, also keep them in
 * the run log while there is room.
 *
 * @param[in] start_ms Kernel time the run started.
 * @param[in] b_run    True during a run.
 */
static void subsystem5_bench_report(uint32_t start_ms, bool b_run)
{
    s5_bench_record_t rec = {0};
    motion_control_status_t motion = {0};
    SYSTIM now = {0};

    ultrasonic_scan_get(&rec.scan);
    obstacle_avoidance_get_status(&rec.status);
    motion_control_get_status(&motion);
    (void)tk_get_otm(&now);
    rec.faults = app_faults();
    rec.motion = motion.state;
    rec.ms = b_run ? (now.lo - start_ms) : 0u;
    if (b_run && (g_logged < S5_BENCH_LOG))
    {
        g_log[g_logged] = rec;
        g_logged++;
    }
    subsystem5_bench_print(&rec);
}

/*!
 * @brief Bench task. Each short GP20 press: the last run's log again (for
 * a run made with USB unplugged), then one servo profile scan, car still.
 * A held press: after 5 s, start avoidance and drive forward 1.5 m,
 * printing and logging the status on every change until the drive ends, a
 * bypass finishes or avoidance stops. Then it waits for GP20 again.
 */
static void subsystem5_bench_task(INT stacd, void * p_exinf)
{
    obstacle_avoidance_status_t status = {0};
    obstacle_avoidance_status_t last = {0};
    motion_control_status_t motion = {0};
    ultrasonic_scan_t scan = {0};
    SYSTIM start = {0};
    uint32_t bypasses = 0u;
    uint32_t request = 0u;
    uint32_t shown = 0u;

    (void)stacd;
    (void)p_exinf;

    app_pull_up(S5_BENCH_BUTTON);

    for (;;)
    {
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[avoid] bench: press GP20 for a profile scan, hold "
                        "it %u s to avoid while driving 1500 mm\n",
                  S5_BENCH_HOLD_MS / 1000u);
        while (!subsystem5_bench_press())
        {
            tm_printf((UB *)"[avoid] last run: %u records\n", g_logged);
            for (shown = 0u; shown < g_logged; shown++)
            {
                subsystem5_bench_print(&g_log[shown]);
            }
            request = ultrasonic_scan_request(ULTRASONIC_SCAN_PROFILE);
            ultrasonic_scan_get(&scan);
            while (request != scan.request)
            {
                tk_dly_tsk(S5_BENCH_POLL_MS);
                ultrasonic_scan_get(&scan);
            }
            subsystem5_bench_report(0u, false);
        }
        tk_dly_tsk(S5_BENCH_GO_MS);
        g_logged = 0u;
        (void)tk_get_otm(&start);
        obstacle_avoidance_get_status(&last);
        bypasses = last.bypasses;
        obstacle_avoidance_start();
        (void)motion_control_move_forward(S5_BENCH_DRIVE_MM);
        do
        {
            tk_dly_tsk(S5_BENCH_POLL_MS);
            obstacle_avoidance_get_status(&status);
            motion_control_get_status(&motion);
            if ((status.state != last.state) || (status.step != last.step))
            {
                subsystem5_bench_report(start.lo, true);
                last = status;
            }
        } while ((bypasses == status.bypasses) &&
                 (OBSTACLE_AVOIDANCE_STOPPED != status.state) &&
                 ((OBSTACLE_AVOIDANCE_MONITOR != status.state) ||
                  (MOTION_CONTROL_RUNNING == motion.state)));
        obstacle_avoidance_cancel();
        subsystem5_bench_report(start.lo, true);
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[avoid] bench done\n");
    }
}
#endif /* APP_TEST */

/*** end of file ***/

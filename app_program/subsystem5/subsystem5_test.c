/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5 (make TEST=1). The checks do not
 * move a motor. Afterwards the bench task waits for the GP20 button: the
 * first press prints a profile scan, the second starts avoidance and
 * drives forward, for the manual tests in testing/subsystem5/TEST_LOG.md.
 */

#include "subsystem5.h"
#include "../subsystem2/motion_control.h"
#include "../test_report.h"

#if APP_TEST
#define S5_TEST_SCAN_MS   500  /* A forward reading takes about 60 ms. */
#define S5_BENCH_BUTTON   20u  /* Robo Pico button, low when pressed. */
#define S5_BENCH_GO_MS    5000 /* Time to put the car down after a press. */
#define S5_BENCH_POLL_MS  20
#define S5_BENCH_DRIVE_MM 1500u

static void subsystem5_bench_task(INT stacd, void * p_exinf);

void subsystem5_test(void)
{
    obstacle_avoidance_status_t status = {0};
    ultrasonic_scan_t scan = {0};
    uint32_t count = 0u;

    TEST_INIT();
    obstacle_avoidance_get_status(&status);
    TEST_CHECK("S5-U40", OBSTACLE_AVOIDANCE_IDLE == status.state,
               "avoidance boots idle");
    ultrasonic_scan_get(&scan);
    count = scan.count;
    ultrasonic_scan_request(ULTRASONIC_SCAN_FORWARD);
    tk_dly_tsk(S5_TEST_SCAN_MS);
    ultrasonic_scan_get(&scan);
    TEST_CHECK("S5-U41", count != scan.count,
               "a forward scan finishes within 500 ms");
    /* (UB *): tm_printf() takes unsigned chars. */
    tm_printf((UB *)"[scan] forward_mm=%u fault=%u\n", scan.forward_mm,
              scan.b_fault);
    TEST_CHECK("S5-U42",
               (4u == sizeof(status)) && (16u == sizeof(scan)) &&
                   (8u == sizeof(scan_profile_t)),
               "packed status 4, scan 16, profile 8 bytes");
    TEST_SUMMARY("subsystem5");

    /* FP is the kernel's generic task pointer type. */
    (void)app_start_task((FP)subsystem5_bench_task, S5_PRI_BENCH, 0,
                         "s5_bench");
}

/*!
 * @brief Wait for a GP20 press and release.
 */
static void subsystem5_bench_press(void)
{
    while (0u != gpio_get_val(S5_BENCH_BUTTON))
    {
        tk_dly_tsk(S5_BENCH_POLL_MS);
    }
    while (0u == gpio_get_val(S5_BENCH_BUTTON))
    {
        tk_dly_tsk(S5_BENCH_POLL_MS);
    }
}

/*!
 * @brief Print the latest scan and avoidance status on two lines.
 */
static void subsystem5_bench_report(void)
{
    obstacle_avoidance_status_t status = {0};
    ultrasonic_scan_t scan = {0};

    ultrasonic_scan_get(&scan);
    obstacle_avoidance_get_status(&status);
    /* (UB *): tm_printf() takes unsigned chars. */
    tm_printf((UB *)"[scan] forward_mm=%u fault=%u closest=%u@%u "
                    "edges=%u..%u right_mm=%u left_mm=%u obstacle=%u\n",
              scan.forward_mm, scan.b_fault, scan.profile.closest_mm,
              scan.profile.closest_deg, scan.profile.right_deg,
              scan.profile.left_deg, scan.profile.right_mm,
              scan.profile.left_mm, scan.profile.b_obstacle);
    tm_printf((UB *)"[avoid] state=%u faults=%u side=%u step=%u "
                    "bypasses=%u\n",
              status.state, status.faults, status.side, status.step,
              status.bypasses);
}

/*!
 * @brief Bench task. First GP20 press: one profile scan, car still. Second
 * press: after 5 s, start avoidance and drive forward 1.5 m, printing the
 * status on every change until the drive ends, a bypass finishes or
 * avoidance stops.
 */
static void subsystem5_bench_task(INT stacd, void * p_exinf)
{
    obstacle_avoidance_status_t status = {0};
    obstacle_avoidance_status_t last = {0};
    motion_control_status_t motion = {0};
    ultrasonic_scan_t scan = {0};
    uint32_t count = 0u;

    (void)stacd;
    (void)p_exinf;

    /* Pull-up only (both on is bus keeper): the button grounds the pin. */
    (void)gpio_set_pin(S5_BENCH_BUTTON, GPIO_MODE_IN);
    clr_w(GPIO(S5_BENCH_BUTTON), GPIO_PDE);
    set_w(GPIO(S5_BENCH_BUTTON), GPIO_PUE);

    tm_printf((UB *)"[avoid] bench: press GP20 for a profile scan\n");
    subsystem5_bench_press();
    ultrasonic_scan_get(&scan);
    count = scan.count;
    ultrasonic_scan_request(ULTRASONIC_SCAN_PROFILE);
    while (count == scan.count)
    {
        tk_dly_tsk(S5_BENCH_POLL_MS);
        ultrasonic_scan_get(&scan);
    }
    subsystem5_bench_report();

    tm_printf((UB *)"[avoid] bench: press GP20 to avoid while driving "
                    "1500 mm\n");
    subsystem5_bench_press();
    tk_dly_tsk(S5_BENCH_GO_MS);
    obstacle_avoidance_start();
    (void)motion_control_move_forward(S5_BENCH_DRIVE_MM);
    do
    {
        tk_dly_tsk(S5_BENCH_POLL_MS);
        obstacle_avoidance_get_status(&status);
        motion_control_get_status(&motion);
        if ((status.state != last.state) || (status.step != last.step))
        {
            subsystem5_bench_report();
            last = status;
        }
    } while ((0u == status.bypasses) &&
             (OBSTACLE_AVOIDANCE_STOPPED != status.state) &&
             ((OBSTACLE_AVOIDANCE_MONITOR != status.state) ||
              (MOTION_CONTROL_RUNNING == motion.state)));
    obstacle_avoidance_cancel();
    subsystem5_bench_report();
    tm_printf((UB *)"[avoid] bench done\n");
}
#endif

/*** end of file ***/

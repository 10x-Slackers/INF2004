#include "subsystem5.h"
#include "../test_report.h"

/* Temporary stationary-desk test; excluded from normal and host builds. */
#define S5_RUN_HARDWARE_BENCH_TEST 1
#if APP_TEST && S5_RUN_HARDWARE_BENCH_TEST && !defined(S5_HOST_TEST)
#include <sys/sysdef.h>
static void s5_hardware_bench_test(void);
#endif

#if APP_TEST
static S5_SCAN example(void)
{
    const unsigned int angles[S5_COARSE_COUNT] = S5_COARSE_ANGLES;
    S5_SCAN scan = {0};
    scan.mode = S5_COARSE;
    scan.count = S5_COARSE_COUNT;
    for (unsigned int i = 0; i < scan.count; i++)
    {
        scan.samples[i].angle_deg = angles[i];
        scan.samples[i].distance_mm = i == 2 ? 200 : 800;
        scan.samples[i].valid = 1;
        scan.samples[i].stamp_ms = 1000;
    }
    return scan;
}

void subsystem5_test(void)
{
    TEST_INIT();
    S5_SCAN scan = example();
    S5_PROFILE profile;
    S5_CONTROL control = {0};
    S5_SAMPLE sample = s5_echo_sample(5800, 90, 1000);
    struct
    {
        unsigned int distance, angle, valid, stamp;
    } reference;
    TEST_CHECK("S5-U01",
               sizeof(S5_SAMPLE) == 8 && sizeof(S5_PROFILE) == 16 &&
                   sizeof(S5_SCAN) == 100 && sizeof(S5_CONTROL) == 32 &&
                   sizeof(S5_HEALTH) == 36,
               "packed layout sizes");
    tm_printf((UB *)"[S5 sizes] sample=%u reference=%u profile=%u scan=%u "
                    "control=%u health=%u\n",
              (unsigned int)sizeof(sample), (unsigned int)sizeof(reference),
              (unsigned int)sizeof(profile), (unsigned int)sizeof(scan),
              (unsigned int)sizeof(control), (unsigned int)sizeof(S5_HEALTH));
    TEST_CHECK("S5-U02", sample.valid && sample.distance_mm == 1000,
               "5800 us converts to 1000 mm");
    S5_SAMPLE invalid = s5_echo_sample(0, 90, 1000);
    TEST_CHECK("S5-U03",
               !invalid.valid && !s5_forward_clear(&invalid, 90, 1000),
               "no echo is invalid");
    s5_make_profile(&scan, 1100, &profile);
    TEST_CHECK("S5-U04",
               profile.closest_mm == 200 && profile.closest_angle == 90,
               "closest point");
    TEST_CHECK("S5-U05",
               profile.detected && profile.valid && profile.left_edge == 90 &&
                   profile.right_edge == 90 && !profile.width_valid,
               "centred sector, width unknown");
    profile.right_mm = 400;
    TEST_CHECK("S5-U06", s5_choose(&profile, 1100, 1) == S5_BYPASS_LEFT,
               "left-only clearance");
    profile.right_mm = 800;
    profile.left_mm = 400;
    TEST_CHECK("S5-U07", s5_choose(&profile, 1100, 1) == S5_BYPASS_RIGHT,
               "right-only clearance");
    profile.right_mm = 400;
    TEST_CHECK("S5-U08", s5_choose(&profile, 1100, 1) == S5_SAFE_STOP,
               "both blocked, no blind reverse");
    for (unsigned int i = 0; i < scan.count; i++)
    {
        scan.samples[i].valid = 0;
    }
    s5_make_profile(&scan, 1100, &profile);
    control.state = S5_PROFILING;
    control.motion_stopped = 1;
    control.pending = 1;
    s5_transition(&control, S5_SCAN_DONE, &scan, 1100);
    TEST_CHECK("S5-U09",
               !profile.valid && control.scan_retries == 1 &&
                   control.action != S5_CONTINUE && control.pending,
               "all invalid retries, never clear");
    TEST_CHECK("S5-U10",
               s5_forward_clear(&sample, 90, 1200) &&
                   !s5_forward_clear(&sample, 90, 1201) &&
                   !s5_forward_clear(&sample, 120, 1100),
               "age boundary and sideways rejection");
    s5_transition(&control, S5_SCAN_DONE, &scan, 1200);
    s5_transition(&control, S5_SCAN_DONE, &scan, 1300);
    TEST_CHECK("S5-U11",
               control.scan_retries == S5_MAX_SCAN_RETRIES &&
                   control.state == S5_STOPPED &&
                   control.action == S5_SAFE_STOP,
               "two scan retries then latched stop");
    control.faults = S5_NO_ECHO | S5_SERVO_ERROR | S5_RECOVERY_FAULT;
    TEST_CHECK("S5-U12", control.faults == 97 && control.scan_retries == 2,
               "seven-bit fault mask");
    S5_SAMPLE values[S5_FILTER_COUNT] = {s5_echo_sample(5800, 90, 1000),
                                         s5_echo_sample(1160, 90, 1060),
                                         s5_echo_sample(0, 90, 1120)};
    TEST_CHECK("S5-U13", s5_filter(values).distance_mm == 200,
               "filter ignores invalid, conservative pair");
    sample.stamp_ms = UINT32_MAX - 99U;
    TEST_CHECK("S5-U14", s5_forward_clear(&sample, 90, 100),
               "timestamp wrap handled by subtraction");
    control = (S5_CONTROL){0};
    s5_transition(&control, S5_ACTIVATE, 0, 0);
    TEST_CHECK("S5-U15", control.state == S5_WAIT_STOP && !control.pending,
               "scan requires stop confirmation");
    s5_transition(&control, S5_MOTION_STOPPED, 0, 10);
    TEST_CHECK("S5-U16", control.state == S5_PROFILING && control.pending,
               "confirmed stop starts coarse scan");
    s5_transition(&control, S5_CANCEL, 0, 20);
    s5_transition(&control, S5_ACTIVATE, 0, 30);
    TEST_CHECK("S5-U17", control.state == S5_STOPPED && control.pending,
               "cancel latches, outstanding result must drain");
    control = (S5_CONTROL){0};
    control.state = S5_LINE_RECOVERY;
    for (unsigned int i = 0; i < 3; i++)
    {
        s5_transition(&control, S5_RECOVERY_FAILED, 0, 100 + i);
    }
    TEST_CHECK("S5-U18",
               control.recovery_retries == 2 && control.state == S5_STOPPED &&
                   control.faults & S5_RECOVERY_FAULT,
               "recovery bounded at two retries");
    scan = example();
    s5_make_profile(&scan, 1100, &profile);
    TEST_CHECK("S5-U19",
               s5_choose(&profile, 1100, 1) == S5_BYPASS_LEFT &&
                   s5_choose(&profile, 1100, 0) == S5_SAFE_STOP,
               "deterministic tie and calibration gate");
    scan.samples[0].valid = 0;
    s5_make_profile(&scan, 1100, &profile);
    TEST_CHECK("S5-U20",
               !profile.right_valid &&
                   s5_choose(&profile, 1100, 1) == S5_SAFE_STOP,
               "unknown side cannot make a safe profile");
    control = (S5_CONTROL){0};
    s5_transition(&control, S5_ACTIVATE, 0, 100);
    s5_transition(&control, S5_TICK, 0, 100 + S5_WAIT_STOP_MS + 1);
    TEST_CHECK("S5-U21",
               control.state == S5_STOPPED && control.faults & S5_SCAN_TIMEOUT,
               "missing stop bounded");

    scan = example();
    scan.mode = S5_FINE;
    scan.samples[5] = scan.samples[2];
    scan.samples[5].distance_mm = 150;
    scan.samples[5].stamp_ms = 1500;
    scan.count = 6;
    s5_make_profile(&scan, 1600, &profile);
    TEST_CHECK("S5-U22",
               profile.closest_mm == 150 && profile.valid_count == 5 &&
                   profile.stamp_ms == 1000,
               "fine duplicate replaces coarse; oldest timestamp retained");
    TEST_CHECK("S5-U23",
               s5_choose(&profile, 1000 + S5_PROFILE_MAX_AGE_MS + 1, 1) ==
                   S5_SAFE_STOP,
               "stale profile stops");
    sample = s5_echo_sample(580, 90, 100);
    scan = (S5_SCAN){0};
    scan.mode = S5_FORWARD;
    scan.count = 1;
    scan.samples[0] = sample;
    control = (S5_CONTROL){0};
    control.pending = 1;
    s5_transition(&control, S5_SCAN_DONE, &scan, 100);
    TEST_CHECK("S5-U24",
               control.state == S5_STOPPED && control.action == S5_SAFE_STOP,
               "critical forward range stops");
    s5_transition(&control, S5_ACTIVATE, 0, 110);
    TEST_CHECK("S5-U25", control.state == S5_STOPPED,
               "activation cannot unlatch");
    s5_transition(&control, S5_MOTION_STOPPED, 0, 120);
    s5_transition(&control, S5_RESET, 0, 130);
    TEST_CHECK("S5-U26",
               control.state == S5_MONITOR && !control.active &&
                   !control.forward.valid,
               "reset needs new activation and range");
    TEST_CHECK("S5-U27",
               !s5_echo_sample(100, 90, 0).valid &&
                   s5_echo_sample(116, 90, 0).distance_mm == 20 &&
                   s5_echo_sample(23200, 90, 0).distance_mm == 4000 &&
                   !s5_echo_sample(24000, 90, 0).valid,
               "sensor useful range boundaries");
    scan = example();
    scan.samples[0].valid = 0;
    scan.samples[1].valid = 0;
    scan.samples[2].valid = 0;
    TEST_CHECK("S5-U28", !s5_filter(scan.samples).valid,
               "all-invalid filter remains invalid");

    scan = example();
    scan.mode = S5_FINE;
    scan.samples[0].distance_mm = 400;
    scan.samples[4].distance_mm = 400;
    control = (S5_CONTROL){0};
    control.state = S5_PROFILING;
    control.motion_stopped = 1;
    control.pending = 1;
    s5_transition(&control, S5_SCAN_DONE, &scan, 1100);
    unsigned int kept_profile =
        control.state == S5_STOPPED && control.profile.valid;
    s5_transition(&control, S5_CANCEL, 0, 1200);
    TEST_CHECK("S5-U29", kept_profile && !control.profile.valid,
               "planner stop preserves profile; cancellation invalidates");
    TEST_SUMMARY("subsystem5");

#if S5_RUN_HARDWARE_BENCH_TEST && !defined(S5_HOST_TEST)
    s5_hardware_bench_test();
#endif
}
#endif

/* ---------- Temporary on-target hardware bench: no direct drivers ----------
 */
#if APP_TEST && S5_RUN_HARDWARE_BENCH_TEST && !defined(S5_HOST_TEST)
#define S5_BENCH_POLL_MS 20
#define S5_BENCH_MAX_MS                                                        \
    ((S5_MAX_SCAN_RETRIES + 3U) * S5_SCAN_MAX_MS + S5_WAIT_STOP_MS)

static const char * s5_bench_state(unsigned int state)
{
    static const char * const names[] = {
        "MONITOR",     "WAIT_STOP",     "PROFILING", "PLAN",
        "WAIT_MOTION", "LINE_RECOVERY", "STOPPED"};
    return state <= S5_STOPPED ? names[state] : "UNKNOWN";
}

static const char * s5_bench_action(unsigned int action)
{
    static const char * const names[] = {
        "NONE",         "CONTINUE", "STOP_AND_SCAN", "BYPASS_LEFT",
        "BYPASS_RIGHT", "RECOVERY", "SAFE_STOP"};
    return action <= S5_SAFE_STOP ? names[action] : "UNKNOWN";
}

static void s5_hardware_bench_test(void)
{
    S5_HEALTH health;
    uint32_t started = s5_now_ms();
    unsigned int busy_seen = 0;
    unsigned int last_mode = S5_FORWARD;
    unsigned int last_retry = 0;
    ER er;

    tm_printf((UB *)"[S5-HW] === BUDDY 5 HARDWARE TEST ===\n");
    s5_get_health(&health);
    uint32_t previous_stamp = health.control.forward.stamp_ms;
    uint32_t timer_before = in_w(TIMER_TIMERAWL);
    er = s5_request_scan(S5_FORWARD);
    if (er < E_OK)
    {
        tm_printf((UB *)"[S5-HW] ERROR: forward request failed (%d).\n", er);
        goto abort_bench;
    }
    tm_printf((UB *)"[S5-HW] Real FORWARD scan requested.\n");
    while (s5_now_ms() - started < S5_BENCH_MAX_MS)
    {
        tk_dly_tsk(S5_BENCH_POLL_MS);
        s5_get_health(&health);
        busy_seen |= health.control.pending || health.scan_state;
        if (!health.control.pending && !health.scan_state &&
            (busy_seen || health.control.forward.stamp_ms != previous_stamp ||
             health.control.state == S5_STOPPED))
        {
            break;
        }
    }
    if (s5_now_ms() - started >= S5_BENCH_MAX_MS)
    {
        goto timeout;
    }
    tm_printf((UB *)"[S5-HW] TIMER delta_us=%u elapsed_ms=%u\n",
              (unsigned int)(in_w(TIMER_TIMERAWL) - timer_before),
              (unsigned int)(s5_now_ms() - started));
    tm_printf((UB *)"[S5-HW] FORWARD valid=%u distance_mm=%u servo_angle=%u "
                    "state=%s action=%s faults=0x%02x\n",
              health.control.forward.valid, health.control.forward.distance_mm,
              health.servo_angle, s5_bench_state(health.control.state),
              s5_bench_action(health.control.action), health.control.faults);
    if (health.control.state != S5_WAIT_STOP)
    {
        tm_printf((UB *)"[S5-HW] No detailed scan requested by production "
                        "logic.\n");
        goto report;
    }

    tm_printf((UB *)"[S5-HW] BENCH-TEST ONLY: simulating MOTION_STOPPED; "
                    "assembly is stationary on desk, Buddy 2 unavailable.\n");
    er = s5_command(S5_MOTION_STOPPED);
    if (er < E_OK)
    {
        tm_printf((UB *)"[S5-HW] ERROR: stop confirmation failed (%d).\n", er);
        goto abort_bench;
    }
    while (s5_now_ms() - started < S5_BENCH_MAX_MS)
    {
        tk_dly_tsk(S5_BENCH_POLL_MS);
        s5_get_health(&health);
        if (health.control.state == S5_PROFILING &&
            (health.control.mode != last_mode ||
             health.control.scan_retries != last_retry))
        {
            last_mode = health.control.mode;
            last_retry = health.control.scan_retries;
            tm_printf((UB *)"[S5-HW] %s SCAN (retry=%u).\n",
                      last_mode == S5_COARSE ? "COARSE" : "FINE", last_retry);
        }
        if (!health.control.pending && !health.scan_state &&
            (health.control.state == S5_STOPPED ||
             health.control.state == S5_MONITOR ||
             health.control.state == S5_WAIT_MOTION))
        {
            if (!S5_BYPASS_CALIBRATED && health.control.profile.valid &&
                health.control.profile.detected && last_mode == S5_FINE &&
                health.control.action == S5_SAFE_STOP)
            {
                tm_printf((UB *)"[S5-HW] Profile completed; bypass remains "
                                "calibration-gated.\n");
            }
            else
            {
                tm_printf((UB *)"[S5-HW] Profiling/planning finished.\n");
            }
            goto report;
        }
    }

timeout:
    tm_printf((UB *)"[S5-HW] TIMEOUT: overall bench deadline reached.\n");
abort_bench:
    er = s5_command(S5_CANCEL);
    tm_printf((UB *)"[S5-HW] CANCEL enqueue result=%d; no further bench "
                    "requests.\n",
              er);
    s5_get_health(&health);
report:
    tm_printf((UB *)"[S5-HW] PROFILE detected=%u valid=%u closest_mm=%u "
                    "closest_angle=%u\n",
              health.control.profile.detected, health.control.profile.valid,
              health.control.profile.closest_mm,
              health.control.profile.closest_angle);
    tm_printf(
        (UB *)"[S5-HW] EDGES left=%u right=%u left_mm=%u left_valid=%u "
              "right_mm=%u right_valid=%u\n",
        health.control.profile.left_edge, health.control.profile.right_edge,
        health.control.profile.left_mm, health.control.profile.left_valid,
        health.control.profile.right_mm, health.control.profile.right_valid);
    tm_printf((UB *)"[S5-HW] RESULT valid_samples=%u state=%s action=%s "
                    "faults=0x%02x profile_faults=0x%02x pending=%u "
                    "scan_state=%u servo_angle=%u\n",
              health.control.profile.valid_count,
              s5_bench_state(health.control.state),
              s5_bench_action(health.control.action), health.control.faults,
              health.control.profile.faults, health.control.pending,
              health.scan_state, health.servo_angle);
    tm_printf((UB *)"[S5-HW] === HARDWARE TEST COMPLETE ===\n");
}
#endif

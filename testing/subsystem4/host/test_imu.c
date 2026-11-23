/** @file test_imu.c
 *
 * @brief Host test of task_imu.c: calibration, humps, tilt, the
 * upside-down fault that ends a demo run and the sensor fault, fed with
 * synthetic samples.
 *
 * Includes task_imu.c itself to reach its private sample functions. The
 * kernel, console and board calls come from stub/ and the stubs below.
 * One sample is one 10 ms period. Build and run as in S4-U21 of
 * testing/subsystem4/TEST_LOG.md.
 */

#include "../../../app_program/subsystem4/task_imu.c"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI       3.14159265358979
#define TEST_G_MG     1000
#define TEST_UPSIDE_N (S4_UPSIDE_MS / S4_IMU_MS) /* Samples to latch. */
#define TEST_FAIL_N   (S4_FAIL_MS / S4_IMU_MS)
#define TEST_FROZEN_N (S4_FROZEN_MS / S4_IMU_MS)

ID gh_s4_mutex;
static uint32_t g_faults; /* Stands in for the kernel event flag. */
static int g_failures;

/* Stubs: the IMU task's kernel, console, board and driver calls. */
ER tk_loc_mtx(ID mtxid, TMO tmout)
{
    (void)mtxid;
    (void)tmout;
    return E_OK;
}

ER tk_unl_mtx(ID mtxid)
{
    (void)mtxid;
    return E_OK;
}

ER tk_slp_tsk(TMO tmout)
{
    (void)tmout;
    return E_OK;
}

ID tk_get_tid(void)
{
    return 1;
}

INT tm_printf(UB const * p_format, ...)
{
    (void)p_format;
    return 0;
}

ER pwm_set_pin(UW no)
{
    (void)no;
    return E_OK;
}

ER pwm_set_wrap(UW no, UW wrap)
{
    (void)no;
    (void)wrap;
    return E_OK;
}

ER pwm_set_cc(UW no, UW cc)
{
    (void)no;
    (void)cc;
    return E_OK;
}

ER pwm_set_enabled(UW no, BOOL enable)
{
    (void)no;
    (void)enable;
    return E_OK;
}

void app_check(ER result, const char * p_what)
{
    (void)result;
    (void)p_what;
}

ER app_start_cyclic(ID h_task, RELTIM period_ms)
{
    (void)h_task;
    (void)period_ms;
    return E_OK;
}

ER drv_gy511_init(void)
{
    return E_OK;
}

ER drv_gy511_read(int32_t * p_x_mg, int32_t * p_y_mg, int32_t * p_z_mg)
{
    *p_x_mg = 0;
    *p_y_mg = 0;
    *p_z_mg = TEST_G_MG;
    return E_OK;
}

void app_fault_set(uint32_t bits)
{
    g_faults |= bits;
}

void app_fault_clear(uint32_t bits)
{
    g_faults &= ~bits;
}

uint32_t app_faults(void)
{
    return g_faults;
}

/* Print one result and count failures. */
static void test_check(bool b_pass, const char * p_text)
{
    printf("%s  %s\n", b_pass ? "PASS" : "FAIL", p_text);
    if (!b_pass)
    {
        g_failures++;
    }
}

/* Fresh task state, calibrated flat (offsets 0). */
static void test_reset(void)
{
    int i;

    memset(&g_imu, 0, sizeof(g_imu));
    memset(&g_accel, 0, sizeof(g_accel));
    memset(&g_tilt, 0, sizeof(g_tilt));
    memset(&g_hump, 0, sizeof(g_hump));
    memset(&g_motion, 0, sizeof(g_motion));
    g_faults = 0u;
    for (i = 0; i < S4_CAL_N; i++)
    {
        imu_calibrate(0, 0, TEST_G_MG);
    }
}

/* Feed n samples as good reads, every other one 1 mg lower on Y: a real
 * sensor's noise, without which the reading would count as frozen. */
static void test_feed(int32_t x_mg, int32_t y_mg, int32_t z_mg, int n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        imu_step(true, x_mg, y_mg - (i & 1), z_mg);
    }
}

/* n wake-ups in a row: failed reads, or the same good reading. */
static void test_steps(bool b_ready, int n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        imu_step(b_ready, 0, 0, TEST_G_MG);
    }
}

/* Feed n samples of the car rolled right side up by deg (a hump under
 * the right wheel, or a sideways tilt): gravity moves from Z into -X. */
static void test_roll(double deg, int n)
{
    double const rad = deg * TEST_PI / 180.0;

    test_feed((int32_t)(-TEST_G_MG * sin(rad)), 0,
              (int32_t)(TEST_G_MG * cos(rad)), n);
}

int main(void)
{
    int i;
    int first = -1;
    bool b_early = false;
    bool b_beeping = false;

    /* The motion class lost "impact": five classes, still 3 bits. */
    test_check(((sizeof(g_motion_name) / sizeof(g_motion_name[0])) ==
                (S4_DESCENDING + 1u)) &&
                   (S4_DESCENDING < 8u),
               "five motion classes with names, fit the 3-bit field");

    test_reset();
    test_check(S4_CAL_N == g_imu.cal_count, "calibrated after S4_CAL_N");
    test_feed(0, 0, TEST_G_MG, 100);
    test_check(0u == g_faults, "flat for 1 s: no fault");

    /* A 30 mm hump under the right wheel: roll asin(30 / 114) = 15.3. */
    test_reset();
    test_roll(15.3, 100);
    test_feed(0, 0, TEST_G_MG, 100);
    test_check((0u == g_faults) && (1u == g_hump.count) &&
                   (g_hump.peak >= 28u) && (g_hump.peak <= 30u),
               "30 mm hump: measured 28..30 mm, no fault");

    /* Bumps: Z down to -500 mg for one sample, then for 150 ms. */
    test_reset();
    test_feed(0, 0, -500, 1);
    test_feed(0, 0, TEST_G_MG, 10);
    test_feed(0, 0, -500, TEST_UPSIDE_N - 5);
    test_feed(0, 0, TEST_G_MG, 10);
    test_check(0u == (g_faults & APP_FAULT_UPSIDE_DOWN),
               "Z below 0 for 10 ms and 150 ms: no upside-down fault");

    /* Held at 60 degrees, not inverted: tilt, but not upside down. */
    test_reset();
    test_roll(60.0, 100);
    test_check((0u != (g_faults & APP_FAULT_TILT)) &&
                   (0u == (g_faults & APP_FAULT_UPSIDE_DOWN)),
               "60 degrees for 1 s: tilt fault, no upside-down fault");

    /* Quick lift: flipped at once. Latches on the 20th sample (200 ms). */
    test_reset();
    test_feed(0, 0, -TEST_G_MG, TEST_UPSIDE_N - 1);
    b_early = (0u != (g_faults & APP_FAULT_UPSIDE_DOWN));
    test_feed(0, 0, -TEST_G_MG, 1);
    test_check((!b_early) && (0u != (g_faults & APP_FAULT_UPSIDE_DOWN)),
               "flipped at once: upside-down fault at 200 ms, not before");
    test_check((0 == g_tilt.pitch) && (0 == g_tilt.roll),
               "fully inverted reads 0 pitch and roll (why tilt misses it)");

    /* Slow lift: turned over nose first across 2 s, then held. Z goes
     * below 0 past 90 degrees, at sample 100. */
    test_reset();
    for (i = 0; i <= 200; i++)
    {
        double const rad = TEST_PI * i / 200.0;

        imu_sample(0, (int32_t)(-TEST_G_MG * sin(rad)),
                   (int32_t)(TEST_G_MG * cos(rad)));
        if ((first < 0) && (0u != (g_faults & APP_FAULT_UPSIDE_DOWN)))
        {
            first = i;
        }
    }
    test_feed(0, 0, -TEST_G_MG, 50);
    printf("INFO  slow lift: upside down latched at sample %d\n", first);
    test_check((first > 100) && (first <= (100 + TEST_UPSIDE_N + 1)),
               "turned over across 2 s: upside down within 200 ms of 90");

    /* Latched until reset: righting the car does not clear it. */
    test_feed(0, 0, TEST_G_MG, 100);
    test_check(0u != (g_faults & APP_FAULT_UPSIDE_DOWN),
               "upside-down fault stays latched after righting the car");

    /* Rolled over the right side across 1 s: the right wheel rises as on
     * a hump, but no hump may be recorded. */
    test_reset();
    for (i = 0; i <= 100; i++)
    {
        test_roll(180.0 * i / 100.0, 1);
    }
    test_feed(0, 0, -TEST_G_MG, 50);
    for (i = 100; i >= 0; i--)
    {
        test_roll(180.0 * i / 100.0, 1); /* Righted the same way. */
    }
    test_feed(0, 0, TEST_G_MG, 100);
    test_check((0u != (g_faults & APP_FAULT_UPSIDE_DOWN)) &&
                   (0u == g_hump.count) && (0u == g_hump.max),
               "rolled over sideways and back: no hump recorded");

    /* A 10 ms jolt (Z -500 mg) coming down a 30 mm hump: still measured. */
    test_reset();
    test_roll(15.3, 50);
    test_roll(5.0, 10);
    test_feed((int32_t)(-TEST_G_MG * sin(5.0 * TEST_PI / 180.0)), 0, -500, 1);
    test_roll(5.0, 5);
    test_feed(0, 0, TEST_G_MG, 100);
    test_check((1u == g_hump.count) && (g_hump.peak >= 28u) &&
                   (g_hump.peak <= 30u),
               "10 ms jolt on a hump: still measured 28..30 mm");

    /* Quick lift: Z 500 mg over 1 g for 100 ms. Set on the 5th sample
     * (50 ms), not before; cleared once Z settles; nothing latched. */
    test_reset();
    test_feed(0, 0, TEST_G_MG + 500, (S4_LIFT_MS / S4_IMU_MS) - 1);
    b_early = (0u != (g_faults & APP_FAULT_LIFTED));
    test_feed(0, 0, TEST_G_MG + 500, 1);
    test_check((!b_early) && (APP_FAULT_LIFTED == g_faults),
               "quick lift: lifted fault at 50 ms, not before, alone");
    test_feed(0, 0, TEST_G_MG + 500, 5);
    test_feed(0, 0, TEST_G_MG, 1);
    test_check(APP_FAULT_LIFTED == g_faults,
               "Z settled: lifted fault held while it beeps");
    test_feed(0, 0, TEST_G_MG, S4_ALARM_MS / S4_IMU_MS);
    test_check(0u == g_faults, "beep over: lifted fault cleared");

    /* Not lifts: a 40 ms knock at the sensor's limit, a slow lift (Z 150
     * mg over 1 g for 1 s), and Z just at S4_LIFT_MG for 1 s. */
    test_reset();
    test_feed(0, 0, 2047, (S4_LIFT_MS / S4_IMU_MS) - 1);
    test_feed(0, 0, TEST_G_MG, 10);
    test_feed(0, 0, TEST_G_MG + 150, 100);
    test_feed(0, 0, TEST_G_MG + S4_LIFT_MG, 100);
    test_check(0u == g_faults, "40 ms knock, slow lift, 300 mg: no fault");

    /* Picked up mid-tilt, as by hand at the end of a run: a 30 mm "hump"
     * is under way when a quick lift sets the lifted fault; then righted. */
    test_reset();
    test_roll(16.0, 30);
    test_feed((int32_t)(-TEST_G_MG * sin(16.0 * TEST_PI / 180.0)), 0,
              TEST_G_MG + 500, (S4_LIFT_MS / S4_IMU_MS) + 1);
    test_roll(16.0, 10);
    test_feed(0, 0, TEST_G_MG, 100);
    test_check((0u == g_hump.count) && (0u == g_hump.max),
               "lifted mid-tilt, then righted: no hump recorded");

    /* Sensor unplugged: 4 failed reads, a good one, 4 more, then a 5th in
     * a row (50 ms). The fault stays when the reads work again, and the
     * beep ends though no sample arrives to time it. */
    test_reset();
    test_steps(false, TEST_FAIL_N - 1);
    test_feed(0, 0, TEST_G_MG, 1);
    test_steps(false, TEST_FAIL_N - 1);
    b_early = (0u != g_faults);
    test_steps(false, 1);
    test_check((!b_early) && (APP_FAULT_IMU_SENSOR == g_faults),
               "5 failed reads in a row: sensor fault, not at 4 + 4");
    test_steps(false, S4_ALARM_MS / S4_IMU_MS);
    b_beeping = (0 != g_imu.beep_left);
    test_feed(0, 0, TEST_G_MG, 100);
    test_check((!b_beeping) && (APP_FAULT_IMU_SENSOR == g_faults),
               "beep ends while reads fail; fault stays once they work");

    /* Reset with the sensor still out: faults again, never calibrates. */
    memset(&g_imu, 0, sizeof(g_imu));
    g_faults = 0u;
    test_steps(false, TEST_FAIL_N);
    test_check((APP_FAULT_IMU_SENSOR == g_faults) && (0 == g_imu.cal_count),
               "no sensor at power-on: sensor fault, not calibrated");

    /* Frozen: the first reading, 49 unchanged, 1 mg on one axis, then 49
     * unchanged again and a 50th (500 ms). */
    test_reset();
    test_steps(true, TEST_FROZEN_N);
    imu_step(true, 1, 0, TEST_G_MG);
    imu_step(true, 0, 0, TEST_G_MG);
    test_steps(true, TEST_FROZEN_N - 1);
    b_early = (0u != g_faults);
    test_steps(true, 1);
    test_check((!b_early) && (APP_FAULT_IMU_SENSOR == g_faults),
               "50 unchanged readings in a row: sensor fault, not at 49 + 49");

    /* Frozen from power-on: faults, and its readings never calibrate. */
    memset(&g_imu, 0, sizeof(g_imu));
    g_faults = 0u;
    test_steps(true, S4_CAL_N + 1);
    test_check((APP_FAULT_IMU_SENSOR == g_faults) &&
                   (g_imu.cal_count < S4_CAL_N),
               "frozen at power-on: sensor fault, not calibrated");

    printf("%s: %d failure(s)\n", (0 == g_failures) ? "ALL PASS" : "FAILED",
           g_failures);
    return (0 == g_failures) ? 0 : 1;
}

/*** end of file ***/

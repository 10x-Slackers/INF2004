/** @file test_faults.c
 *
 * @brief Host test of subsystem 3's sensor fault checks: a line sensor
 * stuck on black, the time limit on the lost-line search, and barcode ADC
 * samples that stop.
 *
 * Runs the real Line Following and Barcode Decoder tasks. The kernel calls
 * come from stub/ and the stubs below; tk_slp_tsk() is one 10 ms wake-up
 * and leaves the task when the wake-ups asked for are used up, so a task's
 * local variables start again on every run. The car is a model: each
 * wake-up moves the odometer by the wheel speeds last commanded, and the
 * heading never moves. Build and run as in S3-U22 of
 * testing/subsystem3/TEST_LOG.md.
 */

#include "../../../app_program/subsystem3/subsystem3.h"
#include "../../../app_program/subsystem3/drv_barcode.h"
#include "../../../app_program/subsystem2/motion_control.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_SPEED_MM_S 150u /* As the Vehicle Controller follows. */

static jmp_buf g_task_exit;
static uint32_t g_wakes;    /* Wake-ups left before the task is left. */
static uint32_t g_bits;     /* What the line sensors read. */
static uint32_t g_adc;      /* Samples per barcode read. */
static int32_t g_left_mm_s; /* Wheel speeds last commanded. */
static int32_t g_right_mm_s;
static uint32_t g_um;     /* Odometer. */
static uint32_t g_faults; /* Stands in for the kernel event flag. */
static int g_failures;

/* ---- Stubs: kernel, faults, drivers and the car ----------------------- */

ID tk_cre_mtx(const T_CMTX * p_mutex)
{
    (void)p_mutex;
    return 1;
}

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

ER tk_get_otm(SYSTIM * p_time)
{
    (void)p_time;
    return E_OK;
}

/* One 10 ms wake-up: the car has moved at the speeds last commanded. */
ER tk_slp_tsk(TMO tmout)
{
    (void)tmout;
    if (0u == g_wakes)
    {
        longjmp(g_task_exit, 1);
    }
    g_wakes--;
    /* (|left| + |right|) / 2 mm/s for 10 ms, in um. */
    g_um += (uint32_t)(abs(g_left_mm_s) + abs(g_right_mm_s)) * 5u;
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

void drv_line_init(void)
{
}

uint32_t drv_line_read(void)
{
    return g_bits;
}

void drv_barcode_init(void)
{
}

uint32_t drv_barcode_read(uint16_t * p_samples)
{
    uint32_t sample = 0u;

    for (sample = 0u; sample < g_adc; sample++)
    {
        p_samples[sample] = 0u; /* White. */
    }
    return g_adc;
}

bool motion_control_drive(int32_t left_mm_s, int32_t right_mm_s)
{
    g_left_mm_s = left_mm_s;
    g_right_mm_s = right_mm_s;
    return true;
}

void motion_control_stop(void)
{
    g_left_mm_s = 0;
    g_right_mm_s = 0;
}

void motion_control_get_status(motion_control_status_t * p_status)
{
    p_status->distance_mm = g_um / 1000u;
    p_status->heading_deg = 0u; /* Never moves: no sweep ever ends. */
}

uint32_t motion_control_distance_um(void)
{
    return g_um;
}

/* ---- Helpers ----------------------------------------------------------- */

/* Print one result and count failures. */
static void test_check(bool b_pass, const char * p_text)
{
    printf("%s  %s\n", b_pass ? "PASS" : "FAIL", p_text);
    if (!b_pass)
    {
        g_failures++;
    }
}

/* Run the Line Following task for some wake-ups with the sensors at bits. */
static void test_line(uint32_t bits, uint32_t wakes)
{
    g_bits = bits;
    g_wakes = wakes;
    if (0 == setjmp(g_task_exit))
    {
        line_following_task(0, NULL);
    }
}

/* Run the Barcode Decoder task for some wake-ups of adc samples each. */
static void test_barcode(uint32_t adc, uint32_t wakes)
{
    g_adc = adc;
    g_wakes = wakes;
    if (0 == setjmp(g_task_exit))
    {
        barcode_decoder_task(0, NULL);
    }
}

/* Start following. The task has always sampled since following stopped (a
 * restart needs a button press or a finished move), so one wake-up first. */
static void test_start(void)
{
    test_line(g_bits, 1u);
    (void)line_following_start(TEST_SPEED_MM_S);
}

/* Follow for mm with the sensors at bits (1.5 mm per wake-up, however the
 * car steers). */
static void test_follow(uint32_t bits, uint32_t mm)
{
    test_line(bits, (mm * 2u) / 3u);
}

/* True if line following gave up with the line sensor fault. */
static bool test_stuck(void)
{
    line_following_t line = {0};

    line_following_get(&line);
    return (0u != line.b_lost) && (0u == line.b_following) &&
           (0u != (g_faults & APP_FAULT_LINE_SENSOR)) && (0 == g_left_mm_s) &&
           (0 == g_right_mm_s);
}

/* ---- Tests ------------------------------------------------------------- */

/* A sensor black for over 300 mm stops the car; normal driving does not. */
static void test_stuck_sensor(void)
{
    line_following_t line = {0};
    uint32_t lap = 0u;

    test_start();
    test_follow(DRV_LINE_LEFT, 297u);
    test_check(0u == g_faults, "left black for 297 mm: no fault yet");
    test_follow(DRV_LINE_LEFT, 6u);
    test_check(test_stuck(), "left black past 300 mm: braked, lost, fault");

    test_start();
    line_following_get(&line);
    test_check((0u == g_faults) && (0u == line.b_lost),
               "the next start clears the fault and b_lost");

    /* The other sensor crossing lines does not hide a stuck one, and its
     * black counts from the first crossing (last_side is still left). */
    test_follow(0u, 30u);
    for (lap = 0u; (lap < 4u) && (0u == g_faults); lap++)
    {
        test_follow(DRV_LINE_RIGHT | DRV_LINE_LEFT, 60u);
        test_follow(DRV_LINE_RIGHT, 60u);
    }
    test_check(test_stuck() && (3u == lap),
               "right stuck, left crossing lines: fault past 300 mm");

    /* The longest runs measured (120 mm), back to back through junctions,
     * with no white between: neither sensor is black for 300 mm. */
    test_start();
    for (lap = 0u; lap < 10u; lap++)
    {
        test_follow(DRV_LINE_LEFT, 120u);
        test_follow(DRV_LINE_LEFT | DRV_LINE_RIGHT, 30u);
        test_follow(DRV_LINE_RIGHT, 120u);
        test_follow(DRV_LINE_LEFT | DRV_LINE_RIGHT, 30u);
    }
    line_following_get(&line);
    test_check((0u == g_faults) && (0u != line.b_following),
               "120 mm runs on alternate sides through junctions: no fault");
    test_follow(DRV_LINE_LEFT | DRV_LINE_RIGHT, 310u);
    test_check(test_stuck(), "both black past 300 mm: fault");
}

/* A search whose heading never reaches the end of the sweep fails at 6 s. */
static void test_search_limit(void)
{
    line_following_t line = {0};
    uint32_t wakes = 0u;

    test_start();
    test_follow(0u, 90u);
    line_following_get(&line);
    test_check(0u != line.b_searching, "all white for 90 mm: searching");
    for (wakes = 0u; (wakes < 1000u) && (0u != line.b_searching); wakes++)
    {
        test_line(0u, 1u);
        line_following_get(&line);
    }
    test_check((wakes > 590u) && (wakes <= 600u) && (0u != line.b_lost) &&
                   (0u == line.b_following) && (0 == g_left_mm_s) &&
                   (0 == g_right_mm_s) && (0u == g_faults),
               "the heading never moves: braked and lost after 6 s");
}

/* Under 100 samples in 100 ms sets the barcode fault; 100 clears it. */
static void test_barcode_samples(void)
{
    test_barcode(0u, 1u);
    test_barcode(20u, 9u);
    test_check(0u == g_faults, "first read empty, then 2 per ms: no fault");
    test_barcode(0u, 10u);
    test_check(APP_FAULT_BARCODE_SENSOR == g_faults,
               "no samples for 100 ms: fault");
    test_barcode(9u, 10u);
    test_check(APP_FAULT_BARCODE_SENSOR == g_faults,
               "90 samples in 100 ms: still the fault");
    test_barcode(10u, 10u);
    test_check(0u == g_faults, "100 samples in 100 ms: fault cleared");
}

int main(void)
{
    (void)line_following_init();
    (void)barcode_decoder_init();
    test_stuck_sensor();
    test_search_limit();
    test_barcode_samples();
    printf("%s\n", (0 == g_failures) ? "ALL PASS" : "FAILED");
    return (0 == g_failures) ? 0 : 1;
}

/*** end of file ***/

/** @file test_motion.c
 *
 * @brief Host test of motion_control.c against a simulated robot.
 *
 * Replaces drv_motion.c with a model: each wheel's speed follows its duty
 * with a lag, the right motor is 15% weaker, and wheels coast briefly after
 * braking. Geometry matches the firmware (65 mm wheels, 2537 edges/rev,
 * 113.5 mm track), so heading and distance are checked against the model's
 * true motion. Build and run as in S2-U21 of testing/subsystem2/TEST_LOG.md.
 */

#include "../../../app_program/subsystem2/drv_motion.h"
#include "../../../app_program/subsystem2/motion_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TEST_PI        3.14159265358979
#define TEST_EDGE_MM   (TEST_PI * 65.0 / 2537.0)
#define TEST_TRACK_MM  113.5
#define TEST_TICKS_MAX 3000u

typedef struct
{
    double speed_mm_s; /* True wheel speed. */
    double edges;      /* True edges travelled, fractional. */
    double gain;       /* mm/s per permille above the dead band. */
    int32_t duty;
    bool b_blocked;
} test_wheel_t;

static test_wheel_t g_wheel[2];
static uint8_t g_skipped;
static uint32_t g_count_origin;
static int g_failures;

/* Print one result and count failures. */
static void test_check(bool b_pass, const char * p_text)
{
    printf("%s  %s\n", b_pass ? "PASS" : "FAIL", p_text);
    if (!b_pass)
    {
        g_failures++;
    }
}

/* ---- Fake driver ------------------------------------------------------- */

void drv_motion_write(int32_t left_duty, int32_t right_duty)
{
    g_wheel[0].duty = left_duty;
    g_wheel[1].duty = right_duty;
}

void drv_motion_read(drv_motion_counts_t * p_counts)
{
    p_counts->left =
        (uint16_t)(g_count_origin + (int32_t)floor(g_wheel[0].edges));
    p_counts->right =
        (uint16_t)(g_count_origin + (int32_t)floor(g_wheel[1].edges));
    p_counts->skipped = g_skipped;
}

void drv_motion_lock(void)
{
}

void drv_motion_unlock(void)
{
}

/* ---- Robot model ------------------------------------------------------- */

static void test_reset(uint32_t count_origin)
{
    memset(g_wheel, 0, sizeof(g_wheel));
    g_wheel[0].gain = 0.6;
    g_wheel[1].gain = 0.6 * 0.85; /* Weaker right motor. */
    g_skipped = 0u;
    g_count_origin = count_origin;
    motion_control_init();
}

/* Advance the model by 1 ms steps for one control period. */
static void test_physics(void)
{
    uint32_t step = 0u;
    uint32_t wheel = 0u;
    double target = 0.0;
    double lag_ms = 0.0;
    double magnitude = 0.0;

    for (step = 0u; step < 10u; step++)
    {
        for (wheel = 0u; wheel < 2u; wheel++)
        {
            test_wheel_t * p_wheel = &g_wheel[wheel];
            magnitude = abs(p_wheel->duty) - 80.0; /* Static friction. */
            target = (magnitude > 0.0) ? ((p_wheel->duty > 0) ? 1.0 : -1.0) *
                                             magnitude * p_wheel->gain
                                       : 0.0;
            lag_ms = (0 == p_wheel->duty) ? 15.0 : 40.0; /* Braking coasts. */
            p_wheel->speed_mm_s += (target - p_wheel->speed_mm_s) / lag_ms;
            if (p_wheel->b_blocked)
            {
                p_wheel->speed_mm_s = 0.0;
            }
            p_wheel->edges += p_wheel->speed_mm_s * 0.001 / TEST_EDGE_MM;
        }
    }
}

/* Run until the state leaves RUNNING; returns the ticks taken. */
static uint32_t test_run(motion_control_status_t * p_status)
{
    uint32_t ticks = 0u;

    do
    {
        test_physics();
        motion_control_tick(10u);
        motion_control_get_status(p_status);
        ticks++;
    } while ((MOTION_CONTROL_RUNNING == p_status->state) &&
             (ticks < TEST_TICKS_MAX));

    /* Let the wheels coast to rest, then take the final reading. */
    for (uint32_t settle = 0u; settle < 20u; settle++)
    {
        test_physics();
        motion_control_tick(10u);
    }
    motion_control_get_status(p_status);
    return ticks;
}

/* True rotation of the model, degrees counter-clockwise. */
static double test_true_heading(void)
{
    return (g_wheel[1].edges - g_wheel[0].edges) * TEST_EDGE_MM /
           TEST_TRACK_MM * 180.0 / TEST_PI;
}

static double test_true_distance(void)
{
    return (g_wheel[0].edges + g_wheel[1].edges) * TEST_EDGE_MM / 2.0;
}

/* ---- Tests ------------------------------------------------------------- */

static void test_turn(bool b_left, uint32_t angle)
{
    motion_control_status_t status = {0};
    char text[96];
    double actual = 0.0;
    uint32_t ticks = 0u;

    test_reset(0u);
    (void)(b_left ? motion_control_turn_left(angle)
                  : motion_control_turn_right(angle));
    ticks = test_run(&status);
    actual = fabs(test_true_heading());
    snprintf(text, sizeof(text),
             "%s %u deg: turned %.1f deg in %.2f s, heading field %u",
             b_left ? "left" : "right", angle, actual, ticks / 100.0,
             status.heading_deg);
    test_check((MOTION_CONTROL_COMPLETE == status.state) &&
                   (fabs(actual - angle) <= 2.0),
               text);
}

static void test_straight(bool b_forward, uint32_t distance_mm)
{
    motion_control_status_t status = {0};
    char text[128];
    double distance = 0.0;
    double heading = 0.0;

    test_reset(0u);
    (void)(b_forward ? motion_control_move_forward(distance_mm)
                     : motion_control_move_backward(distance_mm));
    (void)test_run(&status);
    distance = fabs(test_true_distance());
    heading = test_true_heading();
    snprintf(text, sizeof(text),
             "%s %u mm: moved %.1f mm, drift %.2f deg, odometer %u mm",
             b_forward ? "forward" : "backward", distance_mm, distance, heading,
             status.distance_mm);
    test_check(
        (MOTION_CONTROL_COMPLETE == status.state) &&
            (fabs(distance - distance_mm) <= 3.0) && (fabs(heading) <= 1.0) &&
            (abs((int32_t)status.distance_mm - (int32_t)distance_mm) <= 3),
        text);
}

static void test_drive_and_stop(void)
{
    motion_control_status_t status = {0};
    uint32_t tick = 0u;
    int32_t left = 0;
    int32_t right = 0;
    char text[96];

    test_reset(0u);
    test_check(motion_control_drive(200, 100), "drive accepted");
    for (tick = 0u; tick < 100u; tick++)
    {
        test_physics();
        motion_control_tick(10u);
        /* Repeated, as a follower would. */
        (void)motion_control_drive(200, 100);
    }
    motion_control_get_status(&status);
    left = status.left_speed;
    right = status.right_speed;
    snprintf(text, sizeof(text), "drive 200,100 holds %d,%d mm/s after 1 s",
             left, right);
    test_check((MOTION_CONTROL_RUNNING == status.state) &&
                   (abs(left - 200) <= 15) && (abs(right - 100) <= 15),
               text);

    /* A late wake-up: one step covers 20 ms, the queued one 0 ms. */
    test_physics();
    test_physics();
    motion_control_tick(20u);
    (void)motion_control_drive(200, 100);
    motion_control_tick(0u);
    motion_control_get_status(&status);
    test_check((abs(status.left_speed - 200) <= 15) &&
                   (abs(status.right_speed - 100) <= 15),
               "a late, queued step does not upset the speed");

    motion_control_stop();
    motion_control_get_status(&status);
    test_check((MOTION_CONTROL_IDLE == status.state) &&
                   (0 == g_wheel[0].duty) && (0 == g_wheel[1].duty),
               "stop brakes both wheels at once");

    /* Not renewed: the I3 target expires after 200 ms and both wheels brake. */
    (void)motion_control_drive(150, 150);
    for (tick = 0u; tick < 19u; tick++)
    {
        test_physics();
        motion_control_tick(10u);
    }
    motion_control_get_status(&status);
    test_check(MOTION_CONTROL_RUNNING == status.state,
               "drive still running after 190 ms");
    for (tick = 0u; tick < 2u; tick++)
    {
        test_physics();
        motion_control_tick(10u);
    }
    motion_control_get_status(&status);
    test_check((MOTION_CONTROL_IDLE == status.state) &&
                   (0 == g_wheel[0].duty) && (0 == g_wheel[1].duty),
               "drive not renewed for 200 ms expires and brakes");
}

static void test_faults(void)
{
    motion_control_status_t status = {0};
    uint32_t ticks = 0u;

    test_reset(0u);
    g_wheel[1].b_blocked = true;
    (void)motion_control_move_forward(500u);
    ticks = test_run(&status);
    test_check((MOTION_CONTROL_FAILED == status.state) &&
                   (MOTION_CONTROL_FAULT_STALL_RIGHT == status.faults) &&
                   (ticks >= 100u) && (ticks <= 110u) &&
                   (0 == g_wheel[0].duty) && (0 == g_wheel[1].duty),
               "blocked right wheel: stall fault after 1 s, both braked");

    /* A follower keeps renewing drive(): the stall must still latch. */
    test_reset(0u);
    g_wheel[1].b_blocked = true;
    for (ticks = 0u; ticks < 150u; ticks++)
    {
        (void)motion_control_drive(150, 150);
        test_physics();
        motion_control_tick(10u);
    }
    motion_control_get_status(&status);
    test_check((MOTION_CONTROL_FAILED == status.state) &&
                   (MOTION_CONTROL_FAULT_STALL_RIGHT == status.faults) &&
                   (0 == g_wheel[0].duty) && (0 == g_wheel[1].duty) &&
                   !motion_control_drive(150, 150),
               "renewed drive: stall latches and further drives are refused");
    motion_control_stop();
    test_check(motion_control_drive(150, 150), "stop clears a stalled drive");

    test_reset(0u);
    (void)motion_control_move_forward(200u);
    test_physics();
    motion_control_tick(10u);
    g_skipped++;
    (void)test_run(&status);
    test_check((MOTION_CONTROL_COMPLETE == status.state) &&
                   (MOTION_CONTROL_FAULT_ENCODER == status.faults),
               "missed encoder state is reported and the move still completes");

    test_reset(0u);
    (void)motion_control_move_forward(200u);
    (void)test_run(&status);
    (void)motion_control_move_forward(100u);
    motion_control_get_status(&status);
    test_check(0u == status.faults, "a new command clears the faults");
}

static void test_limits(void)
{
    motion_control_status_t status = {0};

    test_reset(0u);
    test_check(
        !motion_control_move_forward(0u) &&
            !motion_control_move_forward(MOTION_CONTROL_MAX_DISTANCE_MM + 1u) &&
            !motion_control_move_backward(0u) &&
            !motion_control_turn_left(0u) &&
            !motion_control_turn_right(MOTION_CONTROL_MAX_ANGLE_DEG + 1u) &&
            !motion_control_drive(1001, 0) && !motion_control_drive(0, -1001) &&
            !motion_control_set_speed(9u) && !motion_control_set_speed(1001u),
        "out-of-range commands and settings are rejected");
    motion_control_get_status(&status);
    test_check(MOTION_CONTROL_IDLE == status.state, "rejection leaves it idle");

    /* Encoder counters wrap during the move. */
    test_reset(65000u);
    (void)motion_control_move_forward(300u);
    (void)test_run(&status);
    test_check((MOTION_CONTROL_COMPLETE == status.state) &&
                   (fabs(test_true_distance() - 300.0) <= 3.0),
               "16-bit encoder counter wrap is handled");
}

static void test_pack(void)
{
    motion_control_status_t status = {0};
    uint8_t packet[MOTION_CONTROL_PACKET_BYTES];
    static const uint8_t expected[MOTION_CONTROL_PACKET_BYTES] = {
        0x9d, 0xac, 0xc6, 0x44, 0x18, 0x24, 0x69, 0x57, 0x9b, 0xde, 0x02, 0x46};

    /* state 2, cmd 3, faults 5, heading 345, speeds +100 / -500,
     * counts 0x1234 / 0xabcd, distance 0xef0123 (see docs table). */
    status.state = 2u;
    status.command = 3u;
    status.faults = 5u;
    status.heading_deg = 345u;
    status.left_speed = 100;
    status.right_speed = -500;
    status.left_count = 0x1234u;
    status.right_count = 0xabcdu;
    status.distance_mm = 0xef0123u;
    memset(packet, 0xff, sizeof(packet));
    motion_control_pack(&status, packet);
    test_check(0 == memcmp(packet, expected, sizeof(packet)),
               "telemetry: 12 bytes, big-endian, fixed bit layout");
    test_check(12u == sizeof(motion_control_status_t),
               "status snapshot is 12 bytes (full-width equivalent 36 bytes)");
}

static void test_timing(void)
{
    clock_t start = 0;
    uint32_t ticks = 0u;
    double us = 0.0;

    test_reset(0u);
    (void)motion_control_drive(150, 150);
    start = clock();
    for (ticks = 0u; ticks < 100000u; ticks++)
    {
        motion_control_tick(10u);
    }
    us = ((double)(clock() - start) / CLOCKS_PER_SEC) * 1e6 / ticks;
    printf("INFO  host cost of one control step: %.3f us\n", us);
}

int main(void)
{
    test_turn(true, 90u);
    test_turn(false, 90u);
    test_turn(true, 180u);
    test_turn(false, 45u);
    test_turn(true, 360u);
    test_straight(true, 500u);
    test_straight(false, 300u);
    test_straight(true, 2000u);
    test_drive_and_stop();
    test_faults();
    test_limits();
    test_pack();
    test_timing();
    printf("%s: %d failure(s)\n", (0 == g_failures) ? "ALL PASS" : "FAILED",
           g_failures);
    return (0 == g_failures) ? 0 : 1;
}

/*** end of file ***/

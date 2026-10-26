#define _POSIX_C_SOURCE 200809L
#include "../../../app_program/subsystem2/motion_control.h"
#include "../../../app_program/subsystem2/ctrl_math.h"
#include "../../../app_program/subsystem2/console_motion.h"
#include "../../../app_program/subsystem2/drv_motion.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static drv_motion_encoders_t g_counts;
static uint32_t g_now;
static int32_t g_left;
static int32_t g_right;
static uint32_t g_checks;
static bool g_ready = true;

#define CHECK(condition) do { g_checks++; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
    exit(EXIT_FAILURE); } } while (0)

uint32_t drv_motion_lock(void)
{
    (void)pthread_mutex_lock(&g_lock);
    return 0u;
}
void drv_motion_unlock(uint32_t saved)
{
    (void)saved;
    (void)pthread_mutex_unlock(&g_lock);
}
bool drv_motion_init(void)
{
    memset(&g_counts, 0, sizeof(g_counts));
    g_counts.left = MOTION_CONTROL_COUNT_BIAS;
    g_counts.right = MOTION_CONTROL_COUNT_BIAS;
    g_left = 0;
    g_right = 0;
    return g_ready;
}
uint32_t drv_motion_time_us(void) { return g_now; }
void drv_motion_snapshot(drv_motion_encoders_t * p_counts)
{
    *p_counts = g_counts;
}
void drv_motion_write(int32_t left, int32_t right)
{
    g_left = left;
    g_right = right;
}
void console_motion_report(void) {}

static motion_control_status_t status(void)
{
    motion_control_status_t result = {0};
    motion_control_get_status(&result);
    return result;
}
static void step(int32_t left, int32_t right)
{
    g_counts.left = (uint32_t)((int32_t)g_counts.left + left);
    g_counts.right = (uint32_t)((int32_t)g_counts.right + right);
    g_now += MOTION_CONTROL_PERIOD_US;
    motion_control_tick(g_now);
}
static motion_control_config_t configuration(void)
{
    motion_control_config_t result = {0};
    result.circumference_mm = 200u;
    result.track_mm = 100u;
    result.counts_per_rev = 1000u;
    result.kp_q8 = 256u;
    result.ki_q8 = 16u;
    result.speed_mm_s = 100u;
    result.duty_limit = 500u;
    result.timeout_ms = 10000u;
    result.stall_ms = 500u;
    result.confirmed = 1u;
    return result;
}
static void setup(void)
{
    motion_control_config_t config = configuration();
    g_now = 0u;
    g_ready = true;
    motion_control_init();
    CHECK(0 == g_left && 0 == g_right);
    CHECK(!motion_control_arm());
    CHECK(motion_control_configure(&config));
    step(0, 0);
    CHECK(motion_control_arm());
}

static void test_primitives(void)
{
    uint32_t sequence[] = {0u, 1u, 3u, 2u, 0u};
    uint32_t index = 0u;
    int32_t total = 0;
    int32_t previous = 0;
    uint32_t channel_a = 0u;
    uint32_t channel_b = 0u;
    drv_motion_encoders_t counts = {0};
    ctrl_math_pid_t pid = {0};
    for (index = 0u; index < 4u; index++)
    {
        total += drv_motion_decode(sequence[index], sequence[index + 1u]);
    }
    CHECK(4 == total);
    for (index = 4u; index > 0u; index--)
    {
        total += drv_motion_decode(sequence[index], sequence[index - 1u]);
    }
    CHECK(0 == total);
    CHECK(0 == drv_motion_decode(0u, 3u));
    CHECK(0 == drv_motion_decode(2u, 2u));
    CHECK(0u == drv_motion_count(0u, 1u, 0u, &counts));
    CHECK(1u == counts.overflow);
    counts.overflow = 0u;
    CHECK(16777215u == drv_motion_count(16777215u, 0u, 1u, &counts));
    CHECK(1u == counts.overflow);
    CHECK(100u == drv_motion_count(100u, 0u, 3u, &counts));
    CHECK(1u == counts.invalid);
    counts.invalid = 65535u;
    counts.overflow = 0u;
    (void)drv_motion_count(100u, 0u, 3u, &counts);
    CHECK(1u == counts.overflow);
    drv_motion_plan(1200, &previous, &channel_a, &channel_b);
    CHECK(1000u == channel_a && 0u == channel_b);
    drv_motion_plan(-500, &previous, &channel_a, &channel_b);
    CHECK(0u == channel_a && 0u == channel_b);
    drv_motion_plan(-500, &previous, &channel_a, &channel_b);
    CHECK(0u == channel_a && 500u == channel_b);
    drv_motion_plan(0, &previous, &channel_a, &channel_b);
    CHECK(0u == channel_a && 0u == channel_b);
    CHECK(0 == previous);
    ctrl_math_pid_reset(&pid);
    for (index = 0u; index < 1000u; index++)
    {
        CHECK(100 == ctrl_math_pid(&pid, 1000, 0, 4095u,
                                      4095u, 4095u, 100u));
    }
    CHECK(65535u == pid.integral);
    CHECK(0 == ctrl_math_pid(&pid, 0, 100, 1u, 1u, 1u, 1000u));
    CHECK(4096u == pid.previous_error);
    (void)ctrl_math_pid(&pid, 1000, -2000, 4095u, 1u, 0u, 100u);
    CHECK(7096u == pid.previous_error);
    CHECK(-100 == ctrl_math_pid(&pid, -1000, 0, 4095u,
                                   4095u, 4095u, 100u));
    CHECK(INT32_MAX == ctrl_math_clamp(INT64_MAX, INT32_MAX));
    CHECK(-INT32_MAX == ctrl_math_clamp(INT64_MIN, INT32_MAX));
}

static void test_movement(void)
{
    uint32_t index = 0u;
    motion_control_config_t config = configuration();
    setup();
    CHECK(!motion_control_move_forward(0u));
    CHECK(!motion_control_move_forward(10001u));
    CHECK(!motion_control_turn_left(3601u));
    CHECK(motion_control_move_forward(20u));
    CHECK(!motion_control_move_forward(20u));
    CHECK(!motion_control_configure(&config));
    CHECK(!motion_control_set_speed(200u));
    step(0, 0);
    CHECK(g_left > 0 && g_right > 0);
    step(10, 0);
    CHECK(g_left < g_right);
    for (index = 0u; index < 20u; index++) { step(5, 5); }
    CHECK(MOTION_CONTROL_COMPLETE == status().state);
    CHECK(0 == g_left && 0 == g_right);
    CHECK(22 == motion_control_decode_count(status().left_mm));
    CHECK(20 == motion_control_decode_count(status().right_mm));
    CHECK(21 == motion_control_decode_count(status().average_mm));
    CHECK(100 == motion_control_decode_speed(status().left_mm_s));
    CHECK(motion_control_move_backward(20u));
    step(0, 0);
    CHECK(g_left < 0 && g_right < 0);
    for (index = 0u; index < 21u; index++) { step(-5, -5); }
    CHECK(MOTION_CONTROL_COMPLETE == status().state);
    setup();
    CHECK(motion_control_turn_left(900u));
    step(0, 0);
    CHECK(g_left < 0 && g_right > 0);
    for (index = 0u; index < 79u; index++) { step(-5, 5); }
    CHECK(MOTION_CONTROL_COMPLETE == status().state);
    CHECK(motion_control_turn_right(900u));
    step(0, 0);
    CHECK(g_left > 0 && g_right < 0);
    for (index = 0u; index < 79u; index++) { step(5, -5); }
    CHECK(MOTION_CONTROL_COMPLETE == status().state);
    CHECK(0 == motion_control_decode_count(status().average_mm));
    setup();
    CHECK(motion_control_move_backward(20u));
    step(-10, 0);
    CHECK(-g_left < -g_right);
    motion_control_stop();
    step(0, 0);
    CHECK(MOTION_CONTROL_CANCELLED == status().state);
    CHECK(0 == g_left && 0 == g_right);
    motion_control_disarm();
    CHECK(!motion_control_move_forward(10u));
}

static void test_faults(void)
{
    uint32_t index = 0u;
    motion_control_config_t config = configuration();
    setup();
    CHECK(motion_control_move_forward(1000u));
    for (index = 0u; index < 50u; index++) { step(0, 0); }
    CHECK(MOTION_CONTROL_STALL == status().faults);
    CHECK(0 == g_left && 0 == g_right);
    CHECK(!motion_control_arm());
    motion_control_stop();
    CHECK(MOTION_CONTROL_STALL == status().faults);
    CHECK(motion_control_reset_faults());
    CHECK(!motion_control_move_forward(10u));
    CHECK(motion_control_arm());
    CHECK(motion_control_move_forward(10u));
    motion_control_safety_stop(MOTION_CONTROL_ENCODER);
    motion_control_safety_stop(MOTION_CONTROL_STALE);
    step(0, 0);
    CHECK((MOTION_CONTROL_ENCODER | MOTION_CONTROL_STALE |
           MOTION_CONTROL_SAFETY) == status().faults);
    CHECK(0u == status().armed);
    CHECK(!motion_control_move_forward(10u));
    setup();
    motion_control_disarm();
    config.timeout_ms = 100u;
    CHECK(motion_control_configure(&config));
    step(0, 0);
    CHECK(motion_control_arm());
    CHECK(motion_control_move_forward(1000u));
    for (index = 0u; index < 10u; index++) { step(5, 5); }
    CHECK(MOTION_CONTROL_TIMEOUT == status().faults);
    setup();
    CHECK(motion_control_move_forward(10u));
    g_counts.invalid++;
    step(0, 0);
    CHECK(MOTION_CONTROL_ENCODER == status().faults);
    CHECK(motion_control_reset_faults());
    g_counts.overflow = 1u;
    step(0, 0);
    CHECK(!motion_control_reset_faults());
    setup();
    CHECK(motion_control_move_forward(10u));
    step(1000, 0);
    CHECK(MOTION_CONTROL_RANGE == status().faults);
    step(0, 0);
    CHECK(motion_control_reset_faults());
    step(0, 0);
    CHECK(0u == status().faults);
    setup();
    CHECK(motion_control_move_forward(10u));
    g_now += 10000u;
    step(0, 0);
    CHECK(MOTION_CONTROL_TIMING == status().faults);
    CHECK(1u == status().missed_periods);
    setup();
    CHECK(motion_control_move_forward(10u));
    g_now += 40000u;
    step(0, 0);
    CHECK(MOTION_CONTROL_STALE == status().faults);
    setup();
    CHECK(motion_control_move_forward(100u));
    step(-15, 0);
    CHECK(MOTION_CONTROL_ENCODER == status().faults);
    CHECK(0 == g_left && 0 == g_right);
    setup();
    motion_control_record_execution(10000u);
    CHECK(MOTION_CONTROL_TIMING == status().faults);
    CHECK(10000u == status().max_execution_us);
    setup();
    CHECK(!motion_control_test_motor(2u, 100, 100u));
    CHECK(!motion_control_test_motor(0u, 201, 100u));
    CHECK(!motion_control_test_motor(0u, 100, 1001u));
    CHECK(motion_control_test_motor(1u, -100, 100u));
    step(0, 0);
    CHECK(0 == g_left && -100 == g_right);
    for (index = 0u; index < 9u; index++) { step(0, 0); }
    CHECK(MOTION_CONTROL_COMPLETE == status().state);
    CHECK(0 == g_left && 0 == g_right);
    g_now += 40000u;
    CHECK(!motion_control_move_forward(10u));
    motion_control_disarm();
    CHECK(!motion_control_arm());
    g_ready = false;
    motion_control_init();
    step(0, 0);
    CHECK(MOTION_CONTROL_INIT == status().faults);
    CHECK(!motion_control_reset_faults());
}

static void test_packet(void)
{
    uint8_t packet[MOTION_CONTROL_PACKET_BYTES + 1u] = {0};
    motion_control_status_t original = {0};
    motion_control_status_t decoded = {0};
    setup();
    original = status();
    original.left_count = 0u;
    original.right_count = 16777215u;
    original.left_mm_s = 48u;
    original.right_mm_s = 4048u;
    original.timestamp_us = UINT32_MAX;
    original.max_execution_us = UINT32_MAX;
    original.max_period_us = UINT32_MAX;
    original.missed_periods = UINT32_MAX;
    original.faults = 255u;
    packet[MOTION_CONTROL_PACKET_BYTES] = 0xa5u;
    motion_control_pack(&original, packet);
    CHECK(0xa5u == packet[MOTION_CONTROL_PACKET_BYTES]);
    CHECK(0x1fu == packet[0]);
    CHECK(0xf0u == packet[1]);
    CHECK(motion_control_unpack(&decoded, packet, MOTION_CONTROL_PACKET_BYTES));
    CHECK(original.left_count == decoded.left_count);
    CHECK(original.right_count == decoded.right_count);
    CHECK(original.left_mm_s == decoded.left_mm_s);
    CHECK(original.right_mm_s == decoded.right_mm_s);
    CHECK(original.timestamp_us == decoded.timestamp_us);
    CHECK(original.faults == decoded.faults);
    CHECK(original.missed_periods == decoded.missed_periods);
    CHECK(!motion_control_unpack(&decoded, packet, 35u));
    packet[35] |= 1u;
    CHECK(!motion_control_unpack(&decoded, packet, 36u));
    packet[35] &= 254u;
    packet[0] |= 224u;
    CHECK(!motion_control_unpack(&decoded, packet, 36u));
    CHECK(-8388608 == motion_control_decode_count(0u));
    CHECK(8388607 == motion_control_decode_count(16777215u));
}

static void * writer(void * p_unused)
{
    uint32_t index = 0u;
    (void)p_unused;
    for (index = 0u; index < 10000u; index++)
    {
        motion_control_safety_stop((index % 2u) ?
            MOTION_CONTROL_ENCODER : MOTION_CONTROL_STALE);
    }
    return NULL;
}
static void test_concurrent(void)
{
    pthread_t first;
    pthread_t second;
    uint32_t index = 0u;
    setup();
    CHECK(0 == pthread_create(&first, NULL, writer, NULL));
    CHECK(0 == pthread_create(&second, NULL, writer, NULL));
    for (index = 0u; index < 10000u; index++)
    {
        motion_control_status_t sample = status();
        CHECK((0u == sample.faults) ||
              ((0u == sample.armed) &&
               (MOTION_CONTROL_FAILED == sample.state)));
    }
    CHECK(0 == pthread_join(first, NULL));
    CHECK(0 == pthread_join(second, NULL));
    CHECK((MOTION_CONTROL_SAFETY | MOTION_CONTROL_ENCODER |
           MOTION_CONTROL_STALE) == status().faults);
}

static void test_console_and_boundaries(void)
{
    motion_control_config_t config = configuration();
    setup();
    CHECK(console_motion_execute("status"));
    {
        const char * p_old = "reset\narm\nforward 100\n";
        uint32_t index = 0u;
        while ('\0' != *p_old)
        {
            tm_console_receive_byte((uint8_t)*p_old);
            p_old++;
        }
        tm_console_receive_byte('!');
        CHECK('!' == console_motion_getchar());
        CHECK(-1 == console_motion_getchar());
        CHECK(MOTION_CONTROL_SAFETY == status().faults);
        for (index = 0u; index < 128u; index++)
        {
            tm_console_receive_byte('x');
        }
        CHECK('!' == console_motion_getchar());
        CHECK(-1 == console_motion_getchar());
        CHECK(0u == status().armed);
        CHECK(motion_control_reset_faults());
        CHECK(motion_control_arm());
    }
    CHECK(!console_motion_execute("forward -1"));
    CHECK(!console_motion_execute("forward 999999999999"));
    CHECK(!console_motion_execute("motor 0 100 1001"));
    CHECK(!console_motion_execute("config cpr 65536"));
    CHECK(!console_motion_execute("forward 10 extra"));
    CHECK(!console_motion_execute("unknown 10"));
    CHECK(console_motion_execute("forward 10"));
    CHECK(console_motion_execute("stop"));
    CHECK(!console_motion_execute("arm"));
    step(0, 0);
    CHECK(console_motion_execute("reset"));
    CHECK(console_motion_execute("disarm"));
    CHECK(console_motion_execute("config circ 210"));
    CHECK(!motion_control_arm());
    CHECK(console_motion_execute("config confirm 1"));
    step(0, 0);
    CHECK(motion_control_arm());
    motion_control_disarm();
    config.encoder_left_reverse = 1u;
    config.motor_left_reverse = 1u;
    CHECK(motion_control_configure(&config));
    step(-5, 5);
    CHECK(100 == motion_control_decode_speed(status().left_mm_s));
    CHECK(motion_control_arm());
    CHECK(motion_control_move_forward(10u));
    step(0, 0);
    CHECK(g_left < 0 && g_right > 0);
    setup();
    motion_control_disarm();
    config = configuration();
    config.counts_per_rev = 65535u;
    config.circumference_mm = 50u;
    CHECK(motion_control_configure(&config));
    step(0, 0);
    CHECK(motion_control_arm());
    CHECK(!motion_control_move_forward(10000u));
    motion_control_disarm();
    config = configuration();
    config.track_mm = 1000u;
    config.circumference_mm = 50u;
    config.counts_per_rev = 65535u;
    CHECK(motion_control_configure(&config));
    step(0, 0);
    CHECK(motion_control_arm());
    CHECK(motion_control_turn_right(3600u));
    setup();
    step(5, 5);
    motion_control_disarm();
    config = configuration();
    config.encoder_left_reverse = 1u;
    CHECK(motion_control_configure(&config));
    CHECK(!motion_control_arm());
    step(0, 0);
    CHECK(0u == status().faults);
    CHECK(-5 == motion_control_decode_count(status().left_count));
    CHECK(0 == motion_control_decode_speed(status().left_mm_s));
    CHECK(motion_control_arm());
    CHECK(!motion_control_set_speed(1001u));
    setup();
    g_now = UINT32_MAX - 4999u;
    motion_control_tick(g_now);
    CHECK(motion_control_arm());
    CHECK(motion_control_move_forward(10u));
    step(5, 5);
    CHECK(0u == status().faults);
    CHECK(5000u == status().timestamp_us);
}

static void evidence(void)
{
    struct timespec start = {0};
    struct timespec end = {0};
    uint32_t index = 0u;
    uint8_t packet[MOTION_CONTROL_PACKET_BYTES] = {0};
    motion_control_status_t sample = status();
    motion_control_status_t decoded = {0};
    double elapsed = 0;
    (void)clock_gettime(CLOCK_MONOTONIC, &start);
    for (index = 0u; index < 100000u; index++)
    {
        motion_control_pack(&sample, packet);
        (void)motion_control_unpack(&decoded, packet, sizeof(packet));
    }
    (void)clock_gettime(CLOCK_MONOTONIC, &end);
    elapsed = (double)(end.tv_sec - start.tv_sec) * 1e9 +
              (double)(end.tv_nsec - start.tv_nsec);
    printf("representation bytes: config=%zu status=%zu pid=%zu encoder=%zu "
           "wire=%u reference_status=%zu reference_config=%zu\n",
           sizeof(motion_control_config_t), sizeof(motion_control_status_t),
           sizeof(ctrl_math_pid_t), sizeof(drv_motion_encoders_t),
           MOTION_CONTROL_PACKET_BYTES, 15u * sizeof(uint32_t),
           15u * sizeof(uint32_t));
    printf("host packing roundtrip: %.1f ns/op (100000 iterations); "
           "hardware timing pending\n", elapsed / 100000.0);
    setup();
    motion_control_disarm();
    {
        motion_control_config_t config = configuration();
        config.timeout_ms = 60000u;
        CHECK(motion_control_configure(&config));
    }
    step(0, 0);
    CHECK(motion_control_arm());
    (void)clock_gettime(CLOCK_MONOTONIC, &start);
    for (index = 0u; index < 100000u; index++)
    {
        if (MOTION_CONTROL_RUNNING != status().state)
        {
            (void)motion_control_move_forward(1000u);
        }
        step(5, 5);
    }
    (void)clock_gettime(CLOCK_MONOTONIC, &end);
    elapsed = (double)(end.tv_sec - start.tv_sec) * 1e9 +
              (double)(end.tv_nsec - start.tv_nsec);
    CHECK(0u == status().faults);
    printf("host active control fixture: %.1f ns/tick (100000 iterations); "
           "not RP2040 WCET\n", elapsed / 100000.0);
}

int main(void)
{
    test_primitives();
    test_movement();
    test_faults();
    test_packet();
    test_concurrent();
    test_console_and_boundaries();
    evidence();
    printf("PASS: %u assertions\n", g_checks);
    return EXIT_SUCCESS;
}

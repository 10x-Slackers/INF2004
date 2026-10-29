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
uint32_t drv_motion_sample(drv_motion_encoders_t * p_counts)
{
    drv_motion_snapshot(p_counts);
    return g_now;
}
void drv_motion_write(int32_t left, int32_t right)
{
    g_left = left;
    g_right = right;
}
void console_motion_report(void) {}
static uint32_t g_config_reports;
void console_motion_config_report(void) { g_config_reports++; }
static uint32_t g_encoder_reports;
void console_motion_encoder_report(void) { g_encoder_reports++; }

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
    for (uint32_t old = 0u; old < 4u; old++)
    {
        for (uint32_t next = 0u; next < 4u; next++)
        {
            int32_t expected = ((next == ((old + 1u) % 4u)) ? 1 :
                ((old == ((next + 1u) % 4u)) ? -1 : 0));
            CHECK(expected == drv_motion_decode(sequence[old], sequence[next]));
        }
    }
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
    CHECK(-500 == previous);
    CHECK(0u == channel_a && 0u == channel_b);
    drv_motion_plan(500, &previous, &channel_a, &channel_b);
    CHECK(0u == channel_a && 0u == channel_b && 0 == previous);
    drv_motion_plan(500, &previous, &channel_a, &channel_b);
    CHECK(500u == channel_a && 0u == channel_b);
    ctrl_math_pid_reset(&pid);
    for (index = 0u; index < 1000u; index++)
    {
        CHECK(100 == ctrl_math_pid(&pid, 1000, 0, 4095u,
                                      4095u, 4095u, 100u, 10000u));
    }
    CHECK(65535u == pid.integral);
    CHECK(0 == ctrl_math_pid(&pid, 0, 100, 1u, 1u, 1u, 1000u, 10000u));
    CHECK(4096u == pid.previous_error);
    (void)ctrl_math_pid(&pid, 1000, -2000, 4095u, 1u, 0u, 100u, 10000u);
    CHECK(7096u == pid.previous_error);
    CHECK(-100 == ctrl_math_pid(&pid, -1000, 0, 4095u,
                                   4095u, 4095u, 100u, 10000u));
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

static void test_short_measurements(void)
{
    motion_control_config_t config = configuration();
    motion_control_status_t before = {0};
    uint32_t baseline_us = 0u;
    setup();
    motion_control_disarm();
    config.circumference_mm = 200u;
    config.left_counts_per_rev = 1000u;
    config.right_counts_per_rev = 2000u;
    CHECK(motion_control_configure(&config));
    step(0, 0);
    CHECK(motion_control_arm());
    CHECK(motion_control_test_motor(0u, 100, 100u));
    step(0, 0);
    CHECK(100 == g_left);
    before = status();
    baseline_us = g_now;
    g_counts.left += 6u;
    g_now += 500u;
    motion_control_tick(g_now);
    CHECK(before.timestamp_us == status().timestamp_us);
    CHECK(before.left_count == status().left_count);
    CHECK(100 == g_left && 0u == status().faults);
    g_now = baseline_us + MOTION_CONTROL_MIN_SAMPLE_US - 1u;
    motion_control_tick(g_now);
    CHECK(before.timestamp_us == status().timestamp_us);
    g_counts.left += 4u;
    g_now = baseline_us + MOTION_CONTROL_PERIOD_US;
    motion_control_tick(g_now);
    CHECK(10 == motion_control_decode_count(status().left_count));
    CHECK(200 == motion_control_decode_speed(status().left_mm_s));
    CHECK(0u == status().faults && 0u == status().missed_periods);
    CHECK(MOTION_CONTROL_RUNNING == status().state);
    /* A genuine excessive delta is retained across early notifications too. */
    baseline_us = g_now;
    g_counts.left += 300u;
    g_now += 500u;
    motion_control_tick(g_now);
    CHECK(0u == status().faults);
    g_now = baseline_us + MOTION_CONTROL_PERIOD_US;
    motion_control_tick(g_now);
    CHECK(MOTION_CONTROL_RANGE == status().faults && 0u == status().armed);
    CHECK(0 == g_left && 0 == g_right);
    setup();
    CHECK(motion_control_move_forward(100u));
    before = status();
    g_now += 500u;
    motion_control_tick(g_now);
    CHECK(before.timestamp_us == status().timestamp_us);
    g_now = before.timestamp_us + 15001u;
    motion_control_tick(g_now);
    CHECK(MOTION_CONTROL_TIMING == status().faults);
    CHECK(1u == status().missed_periods && 0 == g_left && 0 == g_right);
    setup();
    CHECK(motion_control_move_forward(100u));
    step(0, 0);
    {
        int32_t previous_duty = g_left;
        g_counts.left += 5u;
        g_now += 500u;
        motion_control_tick(g_now);
        CHECK(previous_duty == g_left); /* No second PID integration. */
        motion_control_safety_stop(0u);
        CHECK(0 == g_left && 0 == g_right);
        motion_control_tick(g_now);
        CHECK(0 == g_left && 0 == g_right && 0u == status().armed);
    }
    /* Emulated cold boot near timer wrap. */
    g_now = UINT32_MAX - 11000u;
    motion_control_init();
    config = configuration();
    CHECK(motion_control_configure(&config));
    step(0, 0);
    baseline_us = g_now;
    g_counts.left += 5u;
    g_now += 1500u;
    motion_control_tick(g_now);
    CHECK(baseline_us == status().timestamp_us);
    g_now = baseline_us + 10000u;
    motion_control_tick(g_now);
    CHECK(5 == motion_control_decode_count(status().left_count));
    CHECK(100 == motion_control_decode_speed(status().left_mm_s));
    CHECK(0u == status().faults);
}

static void test_prompt_shutdown(void)
{
    uint32_t action = 0u;
    motion_control_config_t config = configuration();
    for (action = 0u; action < 6u; action++)
    {
        setup();
        CHECK(motion_control_test_motor(0u, 100, 1000u));
        step(0, 0);
        CHECK(100 == g_left);
        if (0u == action) { motion_control_stop(); }
        if (1u == action) { motion_control_disarm(); }
        if (2u == action) { motion_control_safety_stop(0u); }
        if (3u == action) { CHECK(console_motion_execute("stop")); }
        if (4u == action) { tm_console_receive_byte('!'); }
        if (5u == action) { motion_control_record_execution(10000u); }
        /* No next tick: the API itself must have written zero. */
        CHECK(0 == g_left && 0 == g_right);
        g_now += 143000u;
        motion_control_tick(g_now);
        CHECK(0 == g_left && 0 == g_right);
        CHECK(MOTION_CONTROL_RUNNING != status().state);
        if (action >= 2u)
        {
            CHECK(0u == status().armed && 0u != status().faults);
            CHECK(!motion_control_test_motor(0u, 100, 100u));
            CHECK(motion_control_reset_faults());
            CHECK(0 == g_left && 0 == g_right);
            step(0, 0);
            CHECK(0 == g_left && 0 == g_right && 0u == status().armed);
        }
    }
    for (action = 0u; action < 3u; action++)
    {
        setup();
        CHECK(motion_control_test_motor(1u, 100, 1000u));
        step(0, 0);
        CHECK(100 == g_right);
        if (0u == action) { g_counts.invalid++; }
        if (1u == action) { g_counts.right += 1000u; }
        if (2u == action) { g_now += 31000u; }
        step(0, 0);
        CHECK(0 == g_left && 0 == g_right);
        CHECK(MOTION_CONTROL_FAILED == status().state);
        CHECK(0u == status().armed && 0u != status().faults);
    }
    setup();
    motion_control_disarm();
    config.timeout_ms = 100u;
    CHECK(motion_control_configure(&config));
    step(0, 0);
    CHECK(motion_control_arm());
    CHECK(motion_control_test_motor(0u, 100, 1000u));
    for (action = 0u; action < 9u; action++) { step(0, 0); }
    CHECK(100 == g_left);
    step(0, 0);
    CHECK(MOTION_CONTROL_TIMEOUT == status().faults);
    CHECK(0 == g_left && 0 == g_right);
    setup();
    CHECK(motion_control_test_motor(1u, -100, 20u));
    step(0, 0);
    CHECK(-100 == g_right);
    step(0, 0);
    CHECK(MOTION_CONTROL_COMPLETE == status().state);
    CHECK(0 == g_left && 0 == g_right);
}

static void test_independent_cpr(void)
{
    motion_control_config_t config = configuration();
    uint32_t command = 0u;
    uint32_t index = 0u;
    for (command = 0u; command < 4u; command++)
    {
        int32_t left_sign = (1u == command || 2u == command) ? -1 : 1;
        int32_t right_sign = (1u == command || 3u == command) ? -1 : 1;
        setup();
        motion_control_disarm();
        config.left_counts_per_rev = 1000u;
        config.right_counts_per_rev = 2000u;
        config.ki_q8 = 0u;
        CHECK(motion_control_configure(&config));
        step(0, 0);
        CHECK(motion_control_arm());
        if (0u == command) { CHECK(motion_control_move_forward(20u)); }
        if (1u == command) { CHECK(motion_control_move_backward(20u)); }
        if (2u == command) { CHECK(motion_control_turn_left(300u)); }
        if (3u == command) { CHECK(motion_control_turn_right(300u)); }
        step(5 * left_sign, 10 * right_sign);
        CHECK(left_sign * 100 ==
              motion_control_decode_speed(status().left_mm_s));
        CHECK(right_sign * 100 ==
              motion_control_decode_speed(status().right_mm_s));
        CHECK(0 == g_left && 0 == g_right); /* Equal physical progress. */
        for (index = 1u; index < ((command < 2u) ? 20u : 26u); index++)
        {
            step(5 * left_sign, 10 * right_sign);
        }
        if (command >= 2u)
        {
            step(left_sign, right_sign); /* Left131; right261 of262. */
            CHECK(MOTION_CONTROL_RUNNING == status().state);
            CHECK(0 == g_left);
            step(0, right_sign);
        }
        CHECK(MOTION_CONTROL_COMPLETE == status().state);
        CHECK(0u == status().faults);
        CHECK(0 == g_left && 0 == g_right);
    }
    setup();
    motion_control_disarm();
    config.left_counts_per_rev = 1u;
    CHECK(!motion_control_configure(&config));
    config.left_counts_per_rev = 0u; /* Shared fallback for just this wheel. */
    CHECK(motion_control_configure(&config));
    CHECK(console_motion_execute("config cpr 1234"));
    motion_control_get_config(&config);
    CHECK(1234u == config.counts_per_rev);
    CHECK(0u == config.left_counts_per_rev && 0u == config.right_counts_per_rev);
    CHECK(console_motion_execute("config cprl 1200"));
    CHECK(console_motion_execute("config cprr 1600"));
    motion_control_get_config(&config);
    CHECK(1200u == config.left_counts_per_rev);
    CHECK(1600u == config.right_counts_per_rev && 0u == config.confirmed);
    CHECK(!motion_control_arm());
    config = configuration();
    config.circumference_mm = 50u;
    config.left_counts_per_rev = 4u;
    config.right_counts_per_rev = 65535u;
    CHECK(motion_control_configure(&config));
    step(0, 0);
    CHECK(motion_control_arm());
    CHECK(!motion_control_move_forward(10000u));
    CHECK(MOTION_CONTROL_RUNNING != status().state);
}

static void test_encoder_diagnostics(void)
{
    drv_motion_encoders_t counts = {0};
    drv_motion_encoders_t before = {0};
    uint32_t old_pair = 0u;
    uint32_t new_pair = 0u;
    for (old_pair = 0u; old_pair < 16u; old_pair++)
    {
        for (new_pair = 0u; new_pair < 16u; new_pair++)
        {
            memset(&counts, 0, sizeof(counts));
            counts.left = MOTION_CONTROL_COUNT_BIAS;
            counts.right = MOTION_CONTROL_COUNT_BIAS;
            counts.left_phase = old_pair >> 2u;
            counts.right_phase = old_pair & 3u;
            before = counts;
            before.left = drv_motion_count(before.left,
                before.left_phase, new_pair >> 2u, &before);
            before.right = drv_motion_count(before.right,
                before.right_phase, new_pair & 3u, &before);
            drv_motion_capture(&counts, new_pair >> 2u, new_pair & 3u);
            CHECK(before.left == counts.left);
            CHECK(before.right == counts.right);
            CHECK(before.invalid == counts.invalid);
            CHECK(before.overflow == counts.overflow);
            CHECK((3u == ((old_pair >> 2u) ^ (new_pair >> 2u))) ==
                  (1u == counts.left_invalid));
            CHECK((3u == ((old_pair & 3u) ^ (new_pair & 3u))) ==
                  (1u == counts.right_invalid));
            CHECK((new_pair >> 2u) == counts.left_phase &&
                  (new_pair & 3u) == counts.right_phase);
        }
    }
    memset(&counts, 0, sizeof(counts));
    counts.left = MOTION_CONTROL_COUNT_BIAS;
    counts.right = MOTION_CONTROL_COUNT_BIAS;
    drv_motion_capture(&counts, 3u, 0u);
    CHECK(1u == counts.left_invalid && 0u == counts.right_invalid);
    CHECK(1u == counts.invalid && 0u == counts.overflow);
    CHECK(3u == counts.left_phase && 0u == counts.right_phase);
    CHECK(MOTION_CONTROL_COUNT_BIAS == counts.left);
    drv_motion_capture(&counts, 2u, 3u);
    CHECK(1u == counts.left_invalid && 1u == counts.right_invalid);
    CHECK(2u == counts.invalid);
    CHECK(MOTION_CONTROL_COUNT_BIAS + 1u == counts.left);
    before = counts;
    drv_motion_capture(&counts, 2u, 3u);
    CHECK(0 == memcmp(&before, &counts, sizeof(counts)));
    counts.left_invalid = 65535u;
    counts.invalid = 65535u;
    drv_motion_capture(&counts, 1u, 3u);
    CHECK(65535u == counts.left_invalid && 65535u == counts.invalid);
    CHECK(1u == counts.overflow && 1u == counts.right_invalid);
    setup();
    before = g_counts;
    CHECK(!console_motion_execute("encoder"));
    CHECK(0u == g_encoder_reports);
    CHECK(!console_motion_execute("encoder 1"));
    CHECK(0 == memcmp(&before, &g_counts, sizeof(before)));
    CHECK(1u == status().armed && 0u == status().faults);
    CHECK(0 == g_left && 0 == g_right);
}

static void test_deadline_accounting(void)
{
    uint32_t before = 0u;
    setup();
    motion_control_disarm();
    before = status().missed_periods;
    g_now += 5000u;
    step(0, 0); /* Exactly 15 ms is not an overrun. */
    CHECK(before == status().missed_periods);
    g_now += 5001u;
    step(0, 0);
    CHECK(before + 1u == status().missed_periods);
    CHECK(15001u == status().max_period_us);
    CHECK(0u == status().faults);
    motion_control_record_execution(210u);
    CHECK(210u == status().max_execution_us);
    motion_control_record_execution(100u);
    CHECK(210u == status().max_execution_us);
    /* One long sample increments once, not once per lost 10 ms slot. */
    g_now += 131977u;
    step(0, 0);
    CHECK(before + 2u == status().missed_periods);
    CHECK(141977u == status().max_period_us);
    CHECK(0u == status().faults);
    setup();
    CHECK(motion_control_move_forward(100u));
    g_now += 5001u;
    step(0, 0);
    CHECK(MOTION_CONTROL_TIMING == status().faults);
    CHECK(0 == g_left && 0 == g_right);
}

static void test_encoder_policy(void)
{
    uint32_t mode = 0u;
    uint32_t phase = 0u;

    setup();
    motion_control_disarm();
    /* Both phases changed: discard the ambiguous edge and resynchronise. */
    g_counts.left = drv_motion_count(g_counts.left, 0u, 3u, &g_counts);
    CHECK(1u == g_counts.invalid);
    CHECK(MOTION_CONTROL_COUNT_BIAS == g_counts.left);
    step(0, 0);
    CHECK(0u == status().faults);
    CHECK(0u == status().armed);
    CHECK(0 == g_left && 0 == g_right);
    g_counts.left = drv_motion_count(g_counts.left, 3u, 2u, &g_counts);
    step(0, 0);
    CHECK(1 == motion_control_decode_count(status().left_count));
    CHECK(motion_control_arm());
    step(0, 0);
    CHECK(0u == status().faults);

    /* Armed idle, closed-loop motion and open-loop bench pulses are guarded. */
    for (mode = 0u; mode < 3u; mode++)
    {
        setup();
        if (1u == mode)
        {
            CHECK(motion_control_move_forward(100u));
        }
        else if (2u == mode)
        {
            CHECK(motion_control_test_motor(0u, 50, 100u));
        }
        step(0, 0);
        g_counts.right = drv_motion_count(g_counts.right, 0u, 3u,
                                          &g_counts);
        step(0, 0);
        CHECK(MOTION_CONTROL_ENCODER == status().faults);
        CHECK(MOTION_CONTROL_FAILED == status().state);
        CHECK(0u == status().armed);
        CHECK(0 == g_left && 0 == g_right);
        CHECK(!motion_control_arm());
        CHECK(!motion_control_move_forward(10u));
        CHECK(motion_control_reset_faults());
        step(0, 0);
        CHECK(0u == status().faults);
        CHECK(0u == status().armed);
        CHECK(0 == g_left && 0 == g_right);
        CHECK(motion_control_arm());
        CHECK(motion_control_move_forward(10u));
        step(0, 0);
        CHECK(0u == status().faults);
    }

    setup();
    motion_control_disarm();
    CHECK(!console_motion_execute("config el typo"));
    step(0, 0);
    CHECK(0u == status().faults);
    /* A pending edge at arm is conservatively checked on the next tick. */
    g_counts.left = drv_motion_count(g_counts.left, 0u, 3u, &g_counts);
    CHECK(motion_control_arm());
    step(0, 0);
    CHECK(MOTION_CONTROL_ENCODER == status().faults);
    CHECK(motion_control_reset_faults());
    for (phase = 0u; phase < 2u; phase++)
    {
        setup();
        motion_control_disarm();
        if (0u == phase)
        {
            g_counts.left = 16777215u;
            g_counts.left = drv_motion_count(g_counts.left, 0u, 1u,
                                             &g_counts);
        }
        else
        {
            g_counts.invalid = 65535u;
            g_counts.right = drv_motion_count(g_counts.right, 0u, 3u,
                                              &g_counts);
        }
        CHECK(1u == g_counts.overflow);
        step(0, 0);
        CHECK(0u != (MOTION_CONTROL_ENCODER & status().faults));
        CHECK(!motion_control_reset_faults());
        CHECK(!motion_control_arm());
    }
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
    CHECK(!console_motion_execute("status"));
    motion_control_disarm();
    CHECK(console_motion_execute("status"));
    CHECK(console_motion_execute("encoder"));
    motion_control_disarm();
    CHECK(console_motion_execute("config"));
    CHECK(1u == g_config_reports);
    CHECK(motion_control_arm());
    CHECK(!console_motion_execute("config"));
    CHECK(1u == g_config_reports);
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

/* Synthetic calibration only: none of these constants describe the robot. */
static void test_sample_time_and_pid(void)
{
    ctrl_math_pid_t pid = {0};
    motion_control_config_t config = configuration();
    uint32_t mask = 0u;
    ctrl_math_pid_reset(&pid);
    CHECK(50 == ctrl_math_pid(&pid, 100, 0, 0u, 256u, 0u,
                              1000u, 5000u));
    CHECK(150 == ctrl_math_pid(&pid, 100, 0, 0u, 256u, 0u,
                               1000u, 10000u));
    CHECK(300 == ctrl_math_pid(&pid, 100, 0, 0u, 256u, 0u,
                               1000u, 15000u));
    ctrl_math_pid_reset(&pid);
    CHECK(200 == ctrl_math_pid(&pid, 100, 0, 0u, 0u, 256u,
                               1000u, 5000u));
    CHECK(-100 == ctrl_math_pid(&pid, 0 - 100, 0, 0u, 0u, 256u,
                                1000u, 20000u));
    setup();
    step(5, 5);
    CHECK(100 == motion_control_decode_speed(status().left_mm_s));
    motion_control_disarm();
    /* Reconfiguration takes a new count/time baseline together. */
    g_now += 7000u;
    g_counts.left += 7u;
    CHECK(motion_control_configure(&config));
    g_now += 10000u;
    g_counts.left += 5u;
    motion_control_tick(0u); /* Task entry time is deliberately wrong. */
    CHECK(g_now == status().timestamp_us);
    CHECK(100 == motion_control_decode_speed(status().left_mm_s));
    CHECK(0u == status().faults);
    /* Every electrical polarity combination normalizes forward correctly. */
    for (mask = 0u; mask < 16u; mask++)
    {
        setup();
        motion_control_disarm();
        config.motor_left_reverse = mask & 1u;
        config.motor_right_reverse = (mask >> 1u) & 1u;
        config.encoder_left_reverse = (mask >> 2u) & 1u;
        config.encoder_right_reverse = (mask >> 3u) & 1u;
        CHECK(motion_control_configure(&config));
        step(0, 0);
        CHECK(motion_control_arm());
        CHECK(motion_control_move_forward(20u));
        step(config.encoder_left_reverse ? -2 : 2,
             config.encoder_right_reverse ? -2 : 2);
        CHECK(40 == motion_control_decode_speed(status().left_mm_s));
        CHECK(40 == motion_control_decode_speed(status().right_mm_s));
        CHECK(config.motor_left_reverse ? g_left < 0 : g_left > 0);
        CHECK(config.motor_right_reverse ? g_right < 0 : g_right > 0);
        CHECK(0u == status().faults);
    }
}

static void test_count_capacity_and_early_fault(void)
{
    setup();
    motion_control_disarm();
    g_counts.left = 16777200u; /* 15 positive edges remain. */
    g_counts.right = 16777200u;
    motion_control_tick(g_now + 1u);
    /* Advance the hardware clock, not just the task-entry hint. */
    g_now += 1000000000u;
    motion_control_tick(g_now);
    CHECK(0u == status().faults);
    CHECK(motion_control_arm());
    CHECK(!motion_control_move_forward(4u)); /* Needs 20 edges. */
    CHECK(motion_control_move_backward(4u));
    motion_control_stop();
    setup();
    motion_control_disarm();
    {
        motion_control_config_t config = configuration();
        config.counts_per_rev = 200u; /* Exactly one edge per mm. */
        config.encoder_left_reverse = 1u;
        config.encoder_right_reverse = 1u;
        g_counts.left = 16777215u;
        g_counts.right = 16777215u;
        CHECK(motion_control_configure(&config));
        step(0, 0);
        CHECK(0u == status().faults);
        CHECK(motion_control_arm());
        CHECK(!motion_control_move_backward(1u));
        CHECK(motion_control_move_forward(1u));
    }
    setup();
    CHECK(motion_control_move_forward(100u));
    step(0, 0);
    CHECK(g_left > 0);
    g_now += 1u;
    g_counts.invalid++;
    motion_control_tick(g_now);
    CHECK(MOTION_CONTROL_ENCODER == status().faults);
    CHECK(0 == g_left && 0 == g_right && 0u == status().armed);
}

/* Printed field labels are not console command keys. */
static void test_encoder_config_syntax(void)
{
    motion_control_config_t config = {0};
    setup();
    CHECK(!console_motion_execute("config el 1")); /* Armed edits rejected. */
    motion_control_disarm();
    CHECK(!console_motion_execute("config elrev 1"));
    CHECK(!console_motion_execute("config errev 0"));
    motion_control_get_config(&config);
    CHECK(0u == config.encoder_left_reverse);
    CHECK(0u == config.encoder_right_reverse && 1u == config.confirmed);
    CHECK(console_motion_execute("config el 1"));
    CHECK(console_motion_execute("config er 0"));
    motion_control_get_config(&config);
    CHECK(1u == config.encoder_left_reverse);
    CHECK(0u == config.encoder_right_reverse && 0u == config.confirmed);
    CHECK(!console_motion_execute("config el 2"));
    CHECK(!console_motion_execute("config er -1"));
    step(-5, 5); /* Synthetic raw forward; both normalize positive. */
    CHECK(5 == motion_control_decode_count(status().left_count));
    CHECK(5 == motion_control_decode_count(status().right_count));
    step(5, -5); /* Raw backward reverses the normalized movement. */
    CHECK(0 == motion_control_decode_count(status().left_count));
    CHECK(0 == motion_control_decode_count(status().right_count));
    CHECK(0u == status().armed && 0u == status().faults);
    CHECK(0 == g_left && 0 == g_right);
    /* Unconfirmed staging permits polarity before geometry/CPR/gain setup. */
    motion_control_init();
    CHECK(console_motion_execute("config el 1"));
    CHECK(console_motion_execute("config er 0"));
    motion_control_get_config(&config);
    CHECK(1u == config.encoder_left_reverse);
    CHECK(0u == config.encoder_right_reverse && 0u == config.confirmed);
    CHECK(!motion_control_arm());
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
           17u * sizeof(uint32_t));
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
    test_short_measurements();
    test_prompt_shutdown();
    test_independent_cpr();
    test_encoder_diagnostics();
    test_deadline_accounting();
    test_encoder_policy();
    test_packet();
    test_concurrent();
    test_console_and_boundaries();
    test_sample_time_and_pid();
    test_count_capacity_and_early_fault();
    test_encoder_config_syntax();
    evidence();
    printf("PASS: %u assertions\n", g_checks);
    return EXIT_SUCCESS;
}

#include "motion_control.h"
#include "ctrl_math.h"
#include "drv_motion.h"
#include <string.h>

#define MOTION_CONTROL_FORWARD 1u
#define MOTION_CONTROL_BACKWARD 2u
#define MOTION_CONTROL_LEFT 3u
#define MOTION_CONTROL_RIGHT 4u
#define MOTION_CONTROL_TEST 5u
#define MOTION_CONTROL_MAX_DISTANCE 10000u
#define MOTION_CONTROL_MAX_ANGLE 3600u
#define MOTION_CONTROL_COUNT_MAX 8388607

static motion_control_config_t g_config;
static motion_control_status_t g_status;
static ctrl_math_pid_t g_pid[2];
static struct
{
    unsigned int start_left : 24;
    unsigned int start_right : 24;
    unsigned int goal : 23;
    unsigned int last_left : 24;
    unsigned int last_right : 24;
    unsigned int invalid : 16;
    unsigned int test_wheel : 1;
    unsigned int test_duty : 9; /* Signed value + 200. */
    unsigned int test_duration_ms : 10;
    uint32_t start_us;
    uint32_t last_us;
    uint32_t left_edge_us;
    uint32_t right_edge_us;
} g_run;

int32_t motion_control_decode_count(uint32_t encoded)
{
    return (int32_t)encoded - 8388608; /* 24-bit encoding fits int32_t. */
}

int32_t motion_control_decode_speed(uint32_t encoded)
{
    return (int32_t)encoded - 2048; /* 12-bit encoding fits int32_t. */
}

static void motion_control_finish(uint32_t state, uint32_t faults)
{
    g_status.state = state;
    g_status.faults |= faults;
    if (0u != faults)
    {
        g_status.armed = 0u;
    }
    ctrl_math_pid_reset(&g_pid[0]);
    ctrl_math_pid_reset(&g_pid[1]);
}

void motion_control_init(void)
{
    bool ready = drv_motion_init();
    uint32_t saved = drv_motion_lock();

    memset(&g_status, 0, sizeof(g_status));
    memset(&g_run, 0, sizeof(g_run));
    memset(&g_config, 0, sizeof(g_config));
    g_config.speed_mm_s = 100u;
    g_config.duty_limit = 200u;
    g_config.timeout_ms = 10000u;
    g_config.stall_ms = 500u;
    g_config.kp_q8 = 256u;
    g_config.ki_q8 = 16u;
    g_status.left_count = MOTION_CONTROL_COUNT_BIAS;
    g_status.right_count = MOTION_CONTROL_COUNT_BIAS;
    g_status.left_mm = MOTION_CONTROL_COUNT_BIAS;
    g_status.right_mm = MOTION_CONTROL_COUNT_BIAS;
    g_status.average_mm = MOTION_CONTROL_COUNT_BIAS;
    g_status.left_mm_s = MOTION_CONTROL_SPEED_BIAS;
    g_status.right_mm_s = MOTION_CONTROL_SPEED_BIAS;
    g_run.last_left = MOTION_CONTROL_COUNT_BIAS;
    g_run.last_right = MOTION_CONTROL_COUNT_BIAS;
    g_run.last_us = drv_motion_time_us();
    motion_control_finish(MOTION_CONTROL_IDLE,
                           ready ? 0u : MOTION_CONTROL_INIT);
    drv_motion_write(0, 0);
    drv_motion_unlock(saved);
}

static bool motion_control_valid_config(
    const motion_control_config_t * p_config)
{
    return ((p_config->circumference_mm >= 50u) &&
            (p_config->circumference_mm <= 1000u) &&
            (p_config->track_mm >= 50u) && (p_config->track_mm <= 1000u) &&
            (p_config->counts_per_rev >= 4u) &&
            (p_config->speed_mm_s >= 10u) &&
            (p_config->speed_mm_s <= 1000u) &&
            (p_config->duty_limit >= 1u) &&
            (p_config->duty_limit <= 1000u) &&
            (p_config->timeout_ms >= 100u) &&
            (p_config->timeout_ms <= 60000u) &&
            (p_config->stall_ms >= 100u) &&
            (p_config->stall_ms <= 10000u) && (p_config->kp_q8 > 0u));
}

bool motion_control_configure(const motion_control_config_t * p_config)
{
    uint32_t saved = drv_motion_lock();
    bool accepted = false;
    drv_motion_encoders_t counts = {0};
    uint32_t left = 0u;
    uint32_t right = 0u;

    drv_motion_snapshot(&counts);
    left = (0u != p_config->encoder_left_reverse) ?
           16777216u - counts.left : counts.left;
    right = (0u != p_config->encoder_right_reverse) ?
            16777216u - counts.right : counts.right;
    if ((left <= 16777215u) && (right <= 16777215u) &&
        (0u == g_status.armed) &&
        ((0u == p_config->confirmed) ||
         motion_control_valid_config(p_config)))
    {
        g_config = *p_config;
        g_run.last_left = left;
        g_run.last_right = right;
        /* A fresh tick must apply new units/polarity before arming. */
        g_status.timestamp_us = drv_motion_time_us() - 30001u;
        accepted = true;
    }
    drv_motion_unlock(saved);
    return accepted;
}

void motion_control_get_config(motion_control_config_t * p_config)
{
    uint32_t saved = drv_motion_lock();
    *p_config = g_config;
    drv_motion_unlock(saved);
}

bool motion_control_set_speed(uint32_t speed_mm_s)
{
    uint32_t saved = drv_motion_lock();
    bool accepted = ((speed_mm_s >= 10u) && (speed_mm_s <= 1000u) &&
                     (MOTION_CONTROL_RUNNING != g_status.state));
    if (accepted)
    {
        g_config.speed_mm_s = speed_mm_s;
    }
    drv_motion_unlock(saved);
    return accepted;
}

bool motion_control_arm(void)
{
    uint32_t saved = drv_motion_lock();
    bool accepted = ((0u == g_status.faults) &&
                     (0u != g_config.confirmed) &&
                     motion_control_valid_config(&g_config) &&
                     ((drv_motion_time_us() - g_status.timestamp_us) <=
                      30000u));
    if (accepted)
    {
        g_status.armed = 1u;
    }
    drv_motion_unlock(saved);
    return accepted;
}

void motion_control_stop(void)
{
    uint32_t saved = drv_motion_lock();
    motion_control_finish(MOTION_CONTROL_CANCELLED, 0u);
    drv_motion_unlock(saved);
}

void motion_control_disarm(void)
{
    uint32_t saved = drv_motion_lock();
    motion_control_finish(MOTION_CONTROL_CANCELLED, 0u);
    g_status.armed = 0u;
    drv_motion_unlock(saved);
}

void motion_control_safety_stop(uint32_t faults)
{
    uint32_t saved = drv_motion_lock();
    motion_control_finish(MOTION_CONTROL_FAILED,
                           (faults & 255u) | MOTION_CONTROL_SAFETY);
    drv_motion_unlock(saved);
}

bool motion_control_reset_faults(void)
{
    drv_motion_encoders_t counts = {0};
    uint32_t saved = drv_motion_lock();
    bool accepted = false;

    drv_motion_snapshot(&counts);
    if ((0u == g_status.armed) && (0u == counts.overflow) &&
        (0u == (g_status.faults & MOTION_CONTROL_INIT)) &&
        ((drv_motion_time_us() - g_status.timestamp_us) <= 30000u))
    {
        g_status.faults = 0u;
        g_run.invalid = counts.invalid;
        motion_control_finish(MOTION_CONTROL_IDLE, 0u);
        accepted = true;
    }
    drv_motion_unlock(saved);
    return accepted;
}

static bool motion_control_command(uint32_t command, uint32_t amount)
{
    uint32_t saved = drv_motion_lock();
    bool accepted = false;
    int64_t numerator = 0;
    int64_t denominator = 1;
    int64_t ticks = 0;

    if ((0u != g_status.armed) && (0u == g_status.faults) &&
        (MOTION_CONTROL_RUNNING != g_status.state) && (amount > 0u) &&
        ((drv_motion_time_us() - g_status.timestamp_us) <= 30000u))
    {
        if (((MOTION_CONTROL_FORWARD == command) ||
             (MOTION_CONTROL_BACKWARD == command)) &&
            (amount <= MOTION_CONTROL_MAX_DISTANCE))
        {
            numerator = (int64_t)amount * g_config.counts_per_rev;
            denominator = g_config.circumference_mm;
        }
        else if (((MOTION_CONTROL_LEFT == command) ||
                  (MOTION_CONTROL_RIGHT == command)) &&
                 (amount <= MOTION_CONTROL_MAX_ANGLE))
        {
            /* pi ~= 355/113; wheel arc = pi * track * angle / 3600. */
            numerator = (int64_t)g_config.track_mm * amount * 355 *
                        g_config.counts_per_rev;
            denominator = (int64_t)3600 * 113 * g_config.circumference_mm;
        }
        else
        {
            numerator = 0;
        }
        ticks = (numerator + denominator - 1) / denominator;
        if ((ticks > 0) && (ticks <= MOTION_CONTROL_COUNT_MAX))
        {
            g_run.goal = (uint32_t)ticks;
            g_run.start_left = g_status.left_count;
            g_run.start_right = g_status.right_count;
            g_run.start_us = drv_motion_time_us();
            g_run.left_edge_us = g_run.start_us;
            g_run.right_edge_us = g_run.start_us;
            g_status.command = command;
            g_status.state = MOTION_CONTROL_RUNNING;
            accepted = true;
        }
    }
    drv_motion_unlock(saved);
    return accepted;
}

bool motion_control_move_forward(uint32_t distance_mm)
{
    return motion_control_command(MOTION_CONTROL_FORWARD, distance_mm);
}

bool motion_control_move_backward(uint32_t distance_mm)
{
    return motion_control_command(MOTION_CONTROL_BACKWARD, distance_mm);
}

bool motion_control_turn_left(uint32_t angle_ddeg)
{
    return motion_control_command(MOTION_CONTROL_LEFT, angle_ddeg);
}

bool motion_control_turn_right(uint32_t angle_ddeg)
{
    return motion_control_command(MOTION_CONTROL_RIGHT, angle_ddeg);
}

bool motion_control_test_motor(uint32_t wheel, int32_t duty,
                               uint32_t duration_ms)
{
    uint32_t saved = drv_motion_lock();
    bool accepted = ((0u != g_status.armed) && (0u == g_status.faults) &&
                     (MOTION_CONTROL_RUNNING != g_status.state) &&
                     (wheel < 2u) && (duty >= -200) && (duty <= 200) &&
                     (0 != duty) && (duration_ms > 0u) &&
                     (duration_ms <= 1000u) &&
                     ((drv_motion_time_us() - g_status.timestamp_us) <=
                      30000u));
    if (accepted)
    {
        g_run.test_wheel = wheel;
        g_run.test_duty = (uint32_t)(duty + 200);
        g_run.test_duration_ms = duration_ms;
        g_run.start_us = drv_motion_time_us();
        g_run.left_edge_us = g_run.start_us;
        g_run.right_edge_us = g_run.start_us;
        g_status.command = MOTION_CONTROL_TEST;
        g_status.state = MOTION_CONTROL_RUNNING;
    }
    drv_motion_unlock(saved);
    return accepted;
}

static int32_t motion_control_signed_count(uint32_t encoded,
                                            uint32_t reverse)
{
    int32_t count = motion_control_decode_count(encoded);
    return (0u != reverse) ? -count : count;
}

static bool motion_control_measure(const drv_motion_encoders_t * p_counts,
                                    uint32_t elapsed_us)
{
    int32_t left = motion_control_signed_count(
        p_counts->left, g_config.encoder_left_reverse);
    int32_t right = motion_control_signed_count(
        p_counts->right, g_config.encoder_right_reverse);
    int32_t old_left = motion_control_decode_count(g_run.last_left);
    int32_t old_right = motion_control_decode_count(g_run.last_right);
    int64_t left_speed = 0;
    int64_t right_speed = 0;
    int64_t left_mm = 0;
    int64_t right_mm = 0;

    if ((left > MOTION_CONTROL_COUNT_MAX) ||
        (right > MOTION_CONTROL_COUNT_MAX))
    {
        return false;
    }
    g_status.left_count = (uint32_t)(left + 8388608);
    g_status.right_count = (uint32_t)(right + 8388608);
    /* An abnormal delta must not poison every later speed sample. */
    g_run.last_left = g_status.left_count;
    g_run.last_right = g_status.right_count;
    if (left != old_left)
    {
        g_run.left_edge_us = g_status.timestamp_us;
    }
    if (right != old_right)
    {
        g_run.right_edge_us = g_status.timestamp_us;
    }
    if (0u != g_config.counts_per_rev)
    {
        left_mm = (int64_t)left * g_config.circumference_mm /
                  g_config.counts_per_rev;
        right_mm = (int64_t)right * g_config.circumference_mm /
                   g_config.counts_per_rev;
        left_speed = ((int64_t)left - old_left) *
                     g_config.circumference_mm * 1000000 /
                     ((int64_t)g_config.counts_per_rev * elapsed_us);
        right_speed = ((int64_t)right - old_right) *
                      g_config.circumference_mm * 1000000 /
                      ((int64_t)g_config.counts_per_rev * elapsed_us);
    }
    if ((left_mm > 8388607) || (left_mm < -8388608) ||
        (right_mm > 8388607) || (right_mm < -8388608) ||
        (left_speed > 2000) || (left_speed < -2000) ||
        (right_speed > 2000) || (right_speed < -2000))
    {
        return false;
    }
    g_status.left_mm = (uint32_t)(left_mm + 8388608);
    g_status.right_mm = (uint32_t)(right_mm + 8388608);
    g_status.average_mm = (uint32_t)((left_mm + right_mm) / 2 + 8388608);
    g_status.left_mm_s = (uint32_t)(left_speed + 2048);
    g_status.right_mm_s = (uint32_t)(right_speed + 2048);
    return true;
}

static void motion_control_targets(int32_t * p_left, int32_t * p_right)
{
    int32_t left_sign = 1;
    int32_t right_sign = 1;
    int32_t speed = (int32_t)g_config.speed_mm_s;
    int32_t correction = 0;
    int64_t left_progress = 0;
    int64_t right_progress = 0;

    if ((MOTION_CONTROL_BACKWARD == g_status.command) ||
        (MOTION_CONTROL_LEFT == g_status.command))
    {
        left_sign = -1;
    }
    if ((MOTION_CONTROL_BACKWARD == g_status.command) ||
        (MOTION_CONTROL_RIGHT == g_status.command))
    {
        right_sign = -1;
    }
    left_progress = ((int64_t)g_status.left_count - g_run.start_left) *
                    left_sign;
    right_progress = ((int64_t)g_status.right_count - g_run.start_right) *
                     right_sign;
    if ((left_progress < -((int64_t)g_config.counts_per_rev * 2 /
                            g_config.circumference_mm + 2)) ||
        (right_progress < -((int64_t)g_config.counts_per_rev * 2 /
                             g_config.circumference_mm + 2)))
    {
        motion_control_finish(MOTION_CONTROL_FAILED, MOTION_CONTROL_ENCODER);
        return;
    }
    /* Correct wheel mismatch in either travel direction, limited to 25%. */
    correction = ctrl_math_clamp(
        (left_progress - right_progress) * g_config.circumference_mm * 2 /
            g_config.counts_per_rev,
        speed / 4);
    *p_left = (left_progress >= g_run.goal) ? 0 :
              left_sign * (speed - correction);
    *p_right = (right_progress >= g_run.goal) ? 0 :
               right_sign * (speed + correction);
    if ((0 == *p_left) && (0 == *p_right))
    {
        motion_control_finish(MOTION_CONTROL_COMPLETE, 0u);
    }
}

void motion_control_tick(uint32_t now_us)
{
    drv_motion_encoders_t counts = {0};
    uint32_t saved = drv_motion_lock();
    uint32_t elapsed_us = now_us - g_run.last_us;
    int32_t left = 0;
    int32_t right = 0;

    if (elapsed_us > g_status.max_period_us)
    {
        g_status.max_period_us = elapsed_us;
    }
    if (elapsed_us > 15000u)
    {
        if (UINT32_MAX != g_status.missed_periods)
        {
            g_status.missed_periods++;
        }
        if (MOTION_CONTROL_RUNNING == g_status.state)
        {
            motion_control_finish(MOTION_CONTROL_FAILED,
                (elapsed_us > 30000u) ? MOTION_CONTROL_STALE :
                                       MOTION_CONTROL_TIMING);
        }
    }
    g_status.timestamp_us = now_us;
    drv_motion_snapshot(&counts);
    if ((0u != counts.overflow) || (counts.invalid != g_run.invalid))
    {
        motion_control_finish(MOTION_CONTROL_FAILED, MOTION_CONTROL_ENCODER);
    }
    g_run.invalid = counts.invalid;
    if ((0u != elapsed_us) && !motion_control_measure(&counts, elapsed_us))
    {
        motion_control_finish(MOTION_CONTROL_FAILED, MOTION_CONTROL_RANGE);
    }
    g_run.last_us = now_us;
    if (MOTION_CONTROL_RUNNING == g_status.state)
    {
        if ((now_us - g_run.start_us) >= g_config.timeout_ms * 1000u)
        {
            motion_control_finish(MOTION_CONTROL_FAILED,
                                   MOTION_CONTROL_TIMEOUT);
        }
        else if (MOTION_CONTROL_TEST == g_status.command)
        {
            if ((now_us - g_run.start_us) >=
                g_run.test_duration_ms * 1000u)
            {
                motion_control_finish(MOTION_CONTROL_COMPLETE, 0u);
            }
            else if (0u == g_run.test_wheel)
            {
                left = (int32_t)g_run.test_duty - 200;
            }
            else
            {
                right = (int32_t)g_run.test_duty - 200;
            }
        }
        else
        {
            motion_control_targets(&left, &right);
            left = ctrl_math_pid(&g_pid[0], left,
                motion_control_decode_speed(g_status.left_mm_s),
                g_config.kp_q8, g_config.ki_q8, g_config.kd_q8,
                g_config.duty_limit);
            right = ctrl_math_pid(&g_pid[1], right,
                motion_control_decode_speed(g_status.right_mm_s),
                g_config.kp_q8, g_config.ki_q8, g_config.kd_q8,
                g_config.duty_limit);
        }
        if (((0 != left) && ((now_us - g_run.left_edge_us) >=
                             g_config.stall_ms * 1000u)) ||
            ((0 != right) && ((now_us - g_run.right_edge_us) >=
                              g_config.stall_ms * 1000u)))
        {
            motion_control_finish(MOTION_CONTROL_FAILED, MOTION_CONTROL_STALL);
        }
    }
    if ((MOTION_CONTROL_RUNNING != g_status.state) ||
        (0u == g_status.armed) || (0u != g_status.faults))
    {
        left = 0;
        right = 0;
    }
    left = ctrl_math_clamp(left, (int32_t)g_config.duty_limit);
    right = ctrl_math_clamp(right, (int32_t)g_config.duty_limit);
    drv_motion_write((0u != g_config.motor_left_reverse) ? -left : left,
                     (0u != g_config.motor_right_reverse) ? -right : right);
    drv_motion_unlock(saved);
}

void motion_control_record_execution(uint32_t elapsed_us)
{
    uint32_t saved = drv_motion_lock();
    if (elapsed_us > g_status.max_execution_us)
    {
        g_status.max_execution_us = elapsed_us;
    }
    if (elapsed_us >= MOTION_CONTROL_PERIOD_US)
    {
        motion_control_finish(MOTION_CONTROL_FAILED, MOTION_CONTROL_TIMING);
        drv_motion_write(0, 0);
    }
    drv_motion_unlock(saved);
}

void motion_control_get_status(motion_control_status_t * p_status)
{
    uint32_t saved = drv_motion_lock();
    *p_status = g_status;
    drv_motion_unlock(saved);
}

static void motion_control_bits(uint8_t * p_packet, uint32_t * p_offset,
                                 uint32_t value, uint32_t width)
{
    uint32_t bit = 0u;
    for (bit = 0u; bit < width; bit++)
    {
        p_packet[*p_offset / 8u] |=
            (uint8_t)(((value >> (width - bit - 1u)) & 1u) <<
                      (7u - *p_offset % 8u));
        (*p_offset)++;
    }
}

static uint32_t motion_control_read_bits(const uint8_t * p_packet,
                                          uint32_t * p_offset,
                                          uint32_t width)
{
    uint32_t value = 0u;
    uint32_t bit = 0u;
    for (bit = 0u; bit < width; bit++)
    {
        value = (value << 1u) |
                (((uint32_t)p_packet[*p_offset / 8u] >>
                  (7u - *p_offset % 8u)) & 1u);
        (*p_offset)++;
    }
    return value;
}

void motion_control_pack(const motion_control_status_t * p_status,
                         uint8_t * p_packet)
{
    uint32_t offset = 0u;
    memset(p_packet, 0, MOTION_CONTROL_PACKET_BYTES);
    motion_control_bits(p_packet, &offset, p_status->state, 3u);
    motion_control_bits(p_packet, &offset, p_status->armed, 1u);
    motion_control_bits(p_packet, &offset, p_status->faults, 8u);
    motion_control_bits(p_packet, &offset, p_status->command, 3u);
    motion_control_bits(p_packet, &offset, p_status->left_count, 24u);
    motion_control_bits(p_packet, &offset, p_status->right_count, 24u);
    motion_control_bits(p_packet, &offset, p_status->left_mm, 24u);
    motion_control_bits(p_packet, &offset, p_status->right_mm, 24u);
    motion_control_bits(p_packet, &offset, p_status->average_mm, 24u);
    motion_control_bits(p_packet, &offset, p_status->left_mm_s, 12u);
    motion_control_bits(p_packet, &offset, p_status->right_mm_s, 12u);
    motion_control_bits(p_packet, &offset, p_status->timestamp_us, 32u);
    motion_control_bits(p_packet, &offset, p_status->max_execution_us, 32u);
    motion_control_bits(p_packet, &offset, p_status->max_period_us, 32u);
    motion_control_bits(p_packet, &offset, p_status->missed_periods, 32u);
}

bool motion_control_unpack(motion_control_status_t * p_status,
                           const uint8_t * p_packet, uint32_t length)
{
    uint32_t offset = 0u;
    bool valid = false;
    motion_control_status_t decoded = {0};
    if ((MOTION_CONTROL_PACKET_BYTES == length) &&
        (0u == (p_packet[length - 1u] & 1u)))
    {
        decoded.state = motion_control_read_bits(p_packet, &offset, 3u);
        decoded.armed = motion_control_read_bits(p_packet, &offset, 1u);
        decoded.faults = motion_control_read_bits(p_packet, &offset, 8u);
        decoded.command = motion_control_read_bits(p_packet, &offset, 3u);
        decoded.left_count = motion_control_read_bits(p_packet, &offset, 24u);
        decoded.right_count = motion_control_read_bits(p_packet, &offset, 24u);
        decoded.left_mm = motion_control_read_bits(p_packet, &offset, 24u);
        decoded.right_mm = motion_control_read_bits(p_packet, &offset, 24u);
        decoded.average_mm = motion_control_read_bits(p_packet, &offset, 24u);
        decoded.left_mm_s = motion_control_read_bits(p_packet, &offset, 12u);
        decoded.right_mm_s = motion_control_read_bits(p_packet, &offset, 12u);
        decoded.timestamp_us =
            motion_control_read_bits(p_packet, &offset, 32u);
        decoded.max_execution_us =
            motion_control_read_bits(p_packet, &offset, 32u);
        decoded.max_period_us =
            motion_control_read_bits(p_packet, &offset, 32u);
        decoded.missed_periods =
            motion_control_read_bits(p_packet, &offset, 32u);
        valid = ((decoded.state <= MOTION_CONTROL_FAILED) &&
                 (decoded.command <= MOTION_CONTROL_TEST) &&
                 (decoded.left_mm_s >= 48u) &&
                 (decoded.left_mm_s <= 4048u) &&
                 (decoded.right_mm_s >= 48u) &&
                 (decoded.right_mm_s <= 4048u));
        if (valid)
        {
            *p_status = decoded;
        }
    }
    return valid;
}
/* End of file. */

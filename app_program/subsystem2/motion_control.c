/** @file motion_control.c
 *
 * @brief Wheel speed PID, odometry, straight-line correction and
 * encoder-based turns.
 *
 * motion_control_tick() runs every 10 ms in the motion task. The other
 * functions may be called from any task. All state is guarded by the
 * driver lock, and hardware is reached only through drv_motion.h.
 */

#include "motion_control.h"
#include "drv_motion.h"
#include "../app_fault.h"
#include <string.h>

/* Geometry. Turn edges per wheel: track * angle * counts / (360 * D).
 * 355 / 113 is pi to 7 digits. */
#define MOTION_COUNTS_PER_REV    2537  /* Measured over 10 turns by hand. */
#define MOTION_WHEEL_DIAMETER_UM 65000 /* 65 mm. */
#define MOTION_CIRCUMFERENCE_UM  ((MOTION_WHEEL_DIAMETER_UM * 355) / 113)

/* Tuning, measured on the robot. Speed can be changed at run time. */
#define MOTION_TRACK_UM           113500 /* Wheel centre to centre. */
#define MOTION_DEFAULT_SPEED_MM_S 100u
#define MOTION_KP                 256 /* Q8: 1 permille per mm/s. */
#define MOTION_KI                 16  /* Q8, per 10 ms sample. */
#define MOTION_KD                 0
#define MOTION_MIN_SPEED_MM_S     10u

/* Controller tuning. */
#define MOTION_DUTY_LIMIT     800 /* Permille. */
#define MOTION_DUTY_OFFSET    100 /* Feedforward for static friction. */
#define MOTION_APPROACH_MM_S  30  /* Speed at the end of a move. */
#define MOTION_ACCEL_MM_S     5   /* Speed increase per tick (500 mm/s^2). */
#define MOTION_DECEL_PER_S    3   /* Approach mm/s per mm still to go. */
#define MOTION_SYNC_DIVISOR   4   /* Edges of mismatch per 1 mm/s fix. */
#define MOTION_STALL_TICKS    50u /* 0.5 s driven without an edge. */
#define MOTION_DRIVE_TICKS    20u /* Drive target expires after 200 ms. */
#define MOTION_PROGRESS_MAX   131071
#define MOTION_SPEED_LIMIT    1023
#define MOTION_INTEGRAL_LIMIT 32767
#define MOTION_ERROR_LIMIT    2047
#define MOTION_PACKET_OFFSET  1024 /* Telemetry speeds are mm/s + 1024. */
#define MOTION_Q8             256

static motion_control_status_t g_status;
static bool gb_latched = true; /* Safety stop: every command refused. */
static bool gb_ready;          /* Init ran: the latch can be released. */

static struct
{
    unsigned int goal : 16;          /* Edges each wheel must travel. */
    unsigned int left_progress : 17; /* Edges in the commanded direction. */
    unsigned int right_progress : 17;
    unsigned int speed_mm_s : 10;    /* Cruise speed of moves and turns. */
    unsigned int profile_speed : 10; /* Current ramp speed, mm/s. */
    signed int left_target : 11;     /* Drive mode, mm/s. */
    signed int right_target : 11;
    unsigned int left_raw : 16; /* Driver counts at the last tick. */
    unsigned int right_raw : 16;
    unsigned int skipped : 8;
    unsigned int left_stall : 6; /* Ticks driven without an edge. */
    unsigned int right_stall : 6;
    unsigned int drive_age : 5; /* Ticks since the last drive call. */
    unsigned int heading : 30;  /* (right - left) * D, mod one turn. */
    unsigned int odometer : 30; /* |left| + |right| edges, wraps. */
} g_run;

static struct
{
    signed int integral : 16;
    signed int last_error : 12; /* mm/s. */
} g_pid[2];

static int32_t motion_control_clamp(int64_t value, int32_t min, int32_t max);
static int32_t motion_control_wrap(uint32_t difference);
static bool motion_control_start(uint32_t command, uint32_t amount);
static void motion_control_begin(uint32_t command);
static void motion_control_step(int32_t left_delta, int32_t right_delta,
                                uint32_t skipped);
static void motion_control_odometry(int32_t left_delta, int32_t right_delta,
                                    uint32_t elapsed_ms);
static int32_t motion_control_speed(int32_t delta, uint32_t elapsed_ms);
static void motion_control_profile(int32_t left_delta, int32_t right_delta,
                                   int32_t * p_left, int32_t * p_right);
static int32_t motion_control_pid(uint32_t wheel, int32_t target,
                                  int32_t measured);
static void motion_control_put(uint8_t * p_packet, uint32_t * p_bit,
                               uint32_t value, uint32_t width);

/*!
 * @brief Load the defaults, brake and release the start-up latch. Call
 * once, after drv_motion_init() succeeds: until then the latch stays set,
 * so a failed start-up refuses every command for good.
 */
void motion_control_init(void)
{
    drv_motion_counts_t counts = {0};

    drv_motion_read(&counts);
    memset(&g_status, 0, sizeof(g_status));
    memset(&g_run, 0, sizeof(g_run));
    memset(g_pid, 0, sizeof(g_pid));
    g_run.speed_mm_s = MOTION_DEFAULT_SPEED_MM_S;
    g_run.left_raw = counts.left;
    g_run.right_raw = counts.right;
    g_run.skipped = counts.skipped;
    gb_ready = true;
    gb_latched = false;
    drv_motion_write(0, 0);
}

/*!
 * @brief Drive forward in a straight line.
 *
 * @param[in] distance_mm 1..5000 mm.
 * @return False if out of range; nothing changes.
 */
bool motion_control_move_forward(uint32_t distance_mm)
{
    return motion_control_start(MOTION_CONTROL_CMD_FORWARD, distance_mm);
}

/*!
 * @brief Reverse in a straight line.
 *
 * @param[in] distance_mm 1..5000 mm.
 * @return False if out of range; nothing changes.
 */
bool motion_control_move_backward(uint32_t distance_mm)
{
    return motion_control_start(MOTION_CONTROL_CMD_BACKWARD, distance_mm);
}

/*!
 * @brief Turn counter-clockwise on the spot.
 *
 * @param[in] angle_deg 1..360 degrees. 180 is a U-turn.
 * @return False if out of range; nothing changes.
 */
bool motion_control_turn_left(uint32_t angle_deg)
{
    return motion_control_start(MOTION_CONTROL_CMD_LEFT, angle_deg);
}

/*!
 * @brief Turn clockwise on the spot.
 *
 * @param[in] angle_deg 1..360 degrees. 180 is a U-turn.
 * @return False if out of range; nothing changes.
 */
bool motion_control_turn_right(uint32_t angle_deg)
{
    return motion_control_start(MOTION_CONTROL_CMD_RIGHT, angle_deg);
}

/*!
 * @brief Hold each wheel at a speed (line following, bypass arcs).
 *
 * The target lasts 200 ms. Call again before then, or both wheels brake
 * and the state becomes IDLE. After a stall the drive stays FAILED until
 * motion_control_stop() or another command.
 *
 * @param[in] left_mm_s  -1000..1000 mm/s, positive is forward.
 * @param[in] right_mm_s -1000..1000 mm/s.
 * @return False if out of range, stalled or latched; nothing changes.
 */
bool motion_control_drive(int32_t left_mm_s, int32_t right_mm_s)
{
    bool b_valid = ((left_mm_s >= -MOTION_CONTROL_MAX_SPEED_MM_S) &&
                    (left_mm_s <= MOTION_CONTROL_MAX_SPEED_MM_S) &&
                    (right_mm_s >= -MOTION_CONTROL_MAX_SPEED_MM_S) &&
                    (right_mm_s <= MOTION_CONTROL_MAX_SPEED_MM_S));

    drv_motion_lock();
    b_valid = b_valid && (!gb_latched) &&
              ((MOTION_CONTROL_CMD_DRIVE != g_status.command) ||
               (MOTION_CONTROL_FAILED != g_status.state));
    if (b_valid)
    {
        if ((MOTION_CONTROL_CMD_DRIVE != g_status.command) ||
            (MOTION_CONTROL_RUNNING != g_status.state))
        {
            motion_control_begin(MOTION_CONTROL_CMD_DRIVE);
        }
        g_run.left_target = left_mm_s;
        g_run.right_target = right_mm_s;
        g_run.drive_age = 0u;
    }
    drv_motion_unlock();
    return b_valid;
}

/*!
 * @brief Brake both wheels now and cancel the running command.
 */
void motion_control_stop(void)
{
    drv_motion_lock();
    g_status.state = MOTION_CONTROL_IDLE;
    g_status.command = MOTION_CONTROL_CMD_NONE;
    drv_motion_write(0, 0);
    drv_motion_unlock();
}

/*!
 * @brief Latch or release a safety stop. Latching brakes like
 * motion_control_stop(); while latched every move, turn and drive is
 * refused. A release is ignored until motion_control_init() has run.
 *
 * @return True if it was already latched.
 */
bool motion_control_latch(bool b_latched)
{
    bool b_was_latched = false;

    drv_motion_lock();
    b_was_latched = gb_latched;
    gb_latched = b_latched || (!gb_ready);
    if (b_latched && (!b_was_latched))
    {
        g_status.state = MOTION_CONTROL_IDLE;
        g_status.command = MOTION_CONTROL_CMD_NONE;
        drv_motion_write(0, 0);
    }
    drv_motion_unlock();
    return b_was_latched;
}

/*!
 * @brief True while a safety stop is latched.
 */
bool motion_control_latched(void)
{
    bool b_latched = false;

    drv_motion_lock();
    b_latched = gb_latched;
    drv_motion_unlock();
    return b_latched;
}

/*!
 * @brief Set the cruise speed of later moves and turns.
 *
 * @param[in] speed_mm_s 10..1000 mm/s.
 * @return False if out of range.
 */
bool motion_control_set_speed(uint32_t speed_mm_s)
{
    /* The maximum is a positive constant, so the cast keeps its value. */
    bool b_valid = ((speed_mm_s >= MOTION_MIN_SPEED_MM_S) &&
                    (speed_mm_s <= (uint32_t)MOTION_CONTROL_MAX_SPEED_MM_S));

    if (b_valid)
    {
        drv_motion_lock();
        g_run.speed_mm_s = speed_mm_s;
        drv_motion_unlock();
    }
    return b_valid;
}

/*!
 * @brief Copy the motion status.
 */
void motion_control_get_status(motion_control_status_t * p_status)
{
    drv_motion_lock();
    *p_status = g_status;
    drv_motion_unlock();
}

/*!
 * @brief Pack a status into the 12-byte telemetry payload.
 *
 * Fields are written most significant bit first, in the order given in
 * docs/MOTION_CONTROL.md. The last 4 bits are always zero.
 */
void motion_control_pack(const motion_control_status_t * p_status,
                         uint8_t * p_packet)
{
    uint32_t bit = 0u;

    memset(p_packet, 0, MOTION_CONTROL_PACKET_BYTES);
    motion_control_put(p_packet, &bit, p_status->state, 2u);
    motion_control_put(p_packet, &bit, p_status->command, 3u);
    motion_control_put(p_packet, &bit, p_status->heading_deg, 9u);
    /* Speeds are -1023..1023, so the offset values are positive. */
    motion_control_put(p_packet, &bit,
                       (uint32_t)(p_status->left_speed + MOTION_PACKET_OFFSET),
                       11u);
    motion_control_put(p_packet, &bit,
                       (uint32_t)(p_status->right_speed + MOTION_PACKET_OFFSET),
                       11u);
    motion_control_put(p_packet, &bit, p_status->left_count, 16u);
    motion_control_put(p_packet, &bit, p_status->right_count, 16u);
    motion_control_put(p_packet, &bit, p_status->distance_mm, 24u);
}

/*!
 * @brief One 10 ms control step. Called only by the motion task.
 *
 * @param[in] elapsed_ms Kernel time since the last step. 0 for a queued
 * wake-up: edges are counted, everything else waits for the next step.
 */
void motion_control_tick(uint32_t elapsed_ms)
{
    drv_motion_counts_t counts = {0};
    int32_t left_delta = 0;
    int32_t right_delta = 0;

    drv_motion_read(&counts);
    drv_motion_lock();
    /* uint16_t to uint32_t keeps the value; the subtraction wraps. */
    left_delta = motion_control_wrap((uint32_t)counts.left - g_run.left_raw);
    right_delta = motion_control_wrap((uint32_t)counts.right - g_run.right_raw);
    g_run.left_raw = counts.left;
    g_run.right_raw = counts.right;
    motion_control_odometry(left_delta, right_delta, elapsed_ms);
    if (0u != elapsed_ms)
    {
        motion_control_step(left_delta, right_delta, counts.skipped);
    }
    drv_motion_unlock();
}

/*!
 * @brief Distance rolled up to now, in um, wraps. Unlike status.distance_mm
 * it includes the edges since the last step.
 */
uint32_t motion_control_distance_um(void)
{
    drv_motion_counts_t counts = {0};
    int32_t left = 0;
    int32_t right = 0;
    uint32_t edges = 0u;

    drv_motion_lock();
    drv_motion_read(&counts);
    /* As in motion_control_tick(): uint16_t to uint32_t keeps the value,
     * and a delta's magnitude is at most 32768, so it fits uint32_t. */
    left = motion_control_wrap((uint32_t)counts.left - g_run.left_raw);
    right = motion_control_wrap((uint32_t)counts.right - g_run.right_raw);
    edges = g_run.odometer + (uint32_t)((left < 0) ? -left : left) +
            (uint32_t)((right < 0) ? -right : right);
    drv_motion_unlock();
    /* The odometer adds both wheels' edges. int64_t holds the product; the
     * result keeps its low 32 bits, so it wraps. */
    return (uint32_t)(((int64_t)edges * MOTION_CIRCUMFERENCE_UM) /
                      (2 * MOTION_COUNTS_PER_REV));
}

/*!
 * @brief Targets, stall checks and PID for one step. Caller holds the lock.
 */
static void motion_control_step(int32_t left_delta, int32_t right_delta,
                                uint32_t skipped)
{
    int32_t left_target = 0;
    int32_t right_target = 0;
    int32_t left_duty = 0;
    int32_t right_duty = 0;

    if (MOTION_CONTROL_RUNNING == g_status.state)
    {
        if (skipped != g_run.skipped)
        {
            app_fault_set(APP_FAULT_ENCODER);
        }
        if ((MOTION_CONTROL_CMD_DRIVE == g_status.command) &&
            (g_run.drive_age >= MOTION_DRIVE_TICKS))
        {
            g_status.state = MOTION_CONTROL_IDLE; /* Not renewed: brake. */
        }
        else if (MOTION_CONTROL_CMD_DRIVE == g_status.command)
        {
            g_run.drive_age++;
            left_target = g_run.left_target;
            right_target = g_run.right_target;
        }
        else
        {
            motion_control_profile(left_delta, right_delta, &left_target,
                                   &right_target);
        }

        g_run.left_stall = ((0 != left_target) && (0 == left_delta))
                               ? (g_run.left_stall + 1u)
                               : 0u;
        g_run.right_stall = ((0 != right_target) && (0 == right_delta))
                                ? (g_run.right_stall + 1u)
                                : 0u;
        if (g_run.left_stall >= MOTION_STALL_TICKS)
        {
            app_fault_set(APP_FAULT_STALL_LEFT);
            g_status.state = MOTION_CONTROL_FAILED;
        }
        if (g_run.right_stall >= MOTION_STALL_TICKS)
        {
            app_fault_set(APP_FAULT_STALL_RIGHT);
            g_status.state = MOTION_CONTROL_FAILED;
        }
    }
    g_run.skipped = skipped;

    if (MOTION_CONTROL_RUNNING == g_status.state)
    {
        left_duty = motion_control_pid(0u, left_target, g_status.left_speed);
        right_duty = motion_control_pid(1u, right_target, g_status.right_speed);
    }
    drv_motion_write(left_duty, right_duty);
}

/*!
 * @brief Limit a value to min..max.
 */
static int32_t motion_control_clamp(int64_t value, int32_t min, int32_t max)
{
    if (value > max)
    {
        value = max;
    }
    else if (value < min)
    {
        value = min;
    }
    else
    {
        /* Already in range. */
    }
    return (int32_t)value; /* Now within min..max. */
}

/*!
 * @brief Turn a 16-bit counter difference into -32768..32767 edges.
 */
static int32_t motion_control_wrap(uint32_t difference)
{
    /* The masked value is at most 0xffff, so it fits int32_t. */
    return (int32_t)((difference + 32768u) & 0xffffu) - 32768;
}

/*!
 * @brief Start a move or turn. amount is 1..5000 mm, or 1..360 degrees for
 * a turn.
 *
 * @return False if out of range or latched; nothing changes.
 */
static bool motion_control_start(uint32_t command, uint32_t amount)
{
    bool const b_turn = ((MOTION_CONTROL_CMD_LEFT == command) ||
                         (MOTION_CONTROL_CMD_RIGHT == command));
    bool b_valid = ((amount > 0u) &&
                    (amount <= (b_turn ? MOTION_CONTROL_MAX_ANGLE_DEG
                                       : MOTION_CONTROL_MAX_DISTANCE_MM)));
    int64_t goal = 0;

    /* int64_t: amount * 1000 * 2537 does not fit 32 bits. */
    if (b_turn)
    {
        /* Each wheel rolls pi * track * angle / 360; pi cancels out. */
        goal = (((int64_t)MOTION_TRACK_UM * (int64_t)amount *
                 MOTION_COUNTS_PER_REV) +
                (180 * MOTION_WHEEL_DIAMETER_UM)) /
               (360 * MOTION_WHEEL_DIAMETER_UM);
    }
    else
    {
        goal = (((int64_t)amount * 1000 * MOTION_COUNTS_PER_REV) +
                (MOTION_CIRCUMFERENCE_UM / 2)) /
               MOTION_CIRCUMFERENCE_UM;
    }
    if (b_valid)
    {
        drv_motion_lock();
        b_valid = !gb_latched;
        if (b_valid)
        {
            g_run.goal = (uint32_t)goal; /* Limits keep it under 62120. */
            g_run.left_progress = 0u;
            g_run.right_progress = 0u;
            g_run.profile_speed = MOTION_APPROACH_MM_S;
            motion_control_begin(command);
        }
        drv_motion_unlock();
    }
    return b_valid;
}

/*!
 * @brief Enter RUNNING with a fresh PID and no faults. Caller holds the
 * lock.
 */
static void motion_control_begin(uint32_t command)
{
    memset(g_pid, 0, sizeof(g_pid));
    g_run.left_stall = 0u;
    g_run.right_stall = 0u;
    g_status.command = command;
    app_fault_clear(APP_FAULT_MOTION);
    g_status.state = MOTION_CONTROL_RUNNING;
}

/*!
 * @brief Update counts, speeds, heading and distance from new edges.
 */
static void motion_control_odometry(int32_t left_delta, int32_t right_delta,
                                    uint32_t elapsed_ms)
{
    /* int64_t keeps every product below in range. */
    int64_t const full_turn =
        2 * (int64_t)MOTION_TRACK_UM * MOTION_COUNTS_PER_REV;
    int64_t heading =
        ((int64_t)g_run.heading +
         ((int64_t)(right_delta - left_delta) * MOTION_WHEEL_DIAMETER_UM)) %
        full_turn;

    if (heading < 0)
    {
        heading += full_turn;
    }
    /* heading is now 0..full_turn, which is below 2^30. */
    g_run.heading = (uint32_t)heading;
    g_status.heading_deg = (uint32_t)((heading * 360) / full_turn);

    /* These counters wrap at their field width on purpose. A per-tick
     * delta is at most 32768, so its magnitude fits uint32_t. */
    g_status.left_count = g_status.left_count + (uint32_t)left_delta;
    g_status.right_count = g_status.right_count + (uint32_t)right_delta;
    g_run.odometer = g_run.odometer +
                     (uint32_t)((left_delta < 0) ? -left_delta : left_delta) +
                     (uint32_t)((right_delta < 0) ? -right_delta : right_delta);
    g_status.distance_mm =
        (uint32_t)(((int64_t)g_run.odometer * MOTION_CIRCUMFERENCE_UM) /
                   (2 * MOTION_COUNTS_PER_REV * 1000));
    if (0u != elapsed_ms)
    {
        g_status.left_speed = motion_control_speed(left_delta, elapsed_ms);
        g_status.right_speed = motion_control_speed(right_delta, elapsed_ms);
    }
}

/*!
 * @brief Wheel speed in mm/s, limited to -1023..1023.
 */
static int32_t motion_control_speed(int32_t delta, uint32_t elapsed_ms)
{
    /* um per edge * edges / ms = mm/s. int64_t holds the product, and
     * elapsed_ms is a few ms, so it fits. */
    return motion_control_clamp(
        ((int64_t)delta * MOTION_CIRCUMFERENCE_UM) /
            (MOTION_COUNTS_PER_REV * (int64_t)elapsed_ms),
        -MOTION_SPEED_LIMIT, MOTION_SPEED_LIMIT);
}

/*!
 * @brief Wheel targets for a move or turn: ramp up, slow down near the
 * goal, keep both wheels level, and stop each wheel at the goal.
 */
static void motion_control_profile(int32_t left_delta, int32_t right_delta,
                                   int32_t * p_left, int32_t * p_right)
{
    int32_t left_sign = 1;
    int32_t right_sign = 1;
    int32_t goal = 0;
    int32_t left = 0;
    int32_t right = 0;
    int32_t remaining = 0;
    int32_t speed = 0;
    int32_t correction = 0;

    if ((MOTION_CONTROL_CMD_BACKWARD == g_status.command) ||
        (MOTION_CONTROL_CMD_LEFT == g_status.command))
    {
        left_sign = -1;
    }
    if ((MOTION_CONTROL_CMD_BACKWARD == g_status.command) ||
        (MOTION_CONTROL_CMD_RIGHT == g_status.command))
    {
        right_sign = -1;
    }
    /* progress is 17 bits, so it fits int32_t. */
    g_run.left_progress = (uint32_t)motion_control_clamp(
        (int32_t)g_run.left_progress + (left_delta * left_sign), 0,
        MOTION_PROGRESS_MAX);
    g_run.right_progress = (uint32_t)motion_control_clamp(
        (int32_t)g_run.right_progress + (right_delta * right_sign), 0,
        MOTION_PROGRESS_MAX);

    /* These bit-fields are at most 17 bits, so they fit int32_t. */
    goal = (int32_t)g_run.goal;
    left = (int32_t)g_run.left_progress;
    right = (int32_t)g_run.right_progress;

    if ((left >= goal) && (right >= goal))
    {
        g_status.state = MOTION_CONTROL_COMPLETE;
    }
    else
    {
        remaining = goal - ((left + right) / 2);
        remaining = (remaining < 0) ? 0 : remaining;
        /* int64_t holds the product of remaining (17 bits) and the gains. */
        speed =
            MOTION_APPROACH_MM_S +
            motion_control_clamp(((int64_t)remaining * MOTION_CIRCUMFERENCE_UM *
                                  MOTION_DECEL_PER_S) /
                                     (MOTION_COUNTS_PER_REV * 1000),
                                 0, MOTION_SPEED_LIMIT);
        /* profile_speed and speed_mm_s are 10 bits, so they fit int32_t. */
        if (speed > ((int32_t)g_run.profile_speed + MOTION_ACCEL_MM_S))
        {
            speed = (int32_t)g_run.profile_speed + MOTION_ACCEL_MM_S;
        }
        if (speed > (int32_t)g_run.speed_mm_s)
        {
            speed = (int32_t)g_run.speed_mm_s;
        }
        g_run.profile_speed = (uint32_t)speed; /* 30..1000 here. */

        /* The wheel that is ahead slows down, the other speeds up. */
        correction = motion_control_clamp((left - right) / MOTION_SYNC_DIVISOR,
                                          -speed / 2, speed / 2);
        *p_left = (left < goal) ? (left_sign * (speed - correction)) : 0;
        *p_right = (right < goal) ? (right_sign * (speed + correction)) : 0;
    }
}

/*!
 * @brief Speed PID for one wheel, plus friction feedforward.
 *
 * @return Duty in permille. Zero (brake) when the target is zero.
 */
static int32_t motion_control_pid(uint32_t wheel, int32_t target,
                                  int32_t measured)
{
    int32_t const error = motion_control_clamp(
        target - measured, -MOTION_ERROR_LIMIT, MOTION_ERROR_LIMIT);
    int32_t const old_integral = g_pid[wheel].integral;
    int32_t integral = motion_control_clamp(
        old_integral + error, -MOTION_INTEGRAL_LIMIT, MOTION_INTEGRAL_LIMIT);
    int32_t duty = 0;

    if (0 == target)
    {
        g_pid[wheel].integral = 0;
        g_pid[wheel].last_error = 0;
    }
    else
    {
        duty = (((MOTION_KP * error) + (MOTION_KI * integral) +
                 (MOTION_KD * (error - g_pid[wheel].last_error))) /
                MOTION_Q8) +
               ((target > 0) ? MOTION_DUTY_OFFSET : -MOTION_DUTY_OFFSET);
        if ((duty > MOTION_DUTY_LIMIT) || (duty < -MOTION_DUTY_LIMIT))
        {
            /* Saturated: hold the integral so it does not wind up. */
            integral = old_integral;
            duty = motion_control_clamp(duty, -MOTION_DUTY_LIMIT,
                                        MOTION_DUTY_LIMIT);
        }
        g_pid[wheel].integral = integral;
        g_pid[wheel].last_error = error;
    }
    return duty;
}

/*!
 * @brief Append the low width bits of value to a packet, MSB first.
 */
static void motion_control_put(uint8_t * p_packet, uint32_t * p_bit,
                               uint32_t value, uint32_t width)
{
    uint32_t bit = 0u;

    for (bit = 0u; bit < width; bit++)
    {
        if (0u != ((value >> (width - 1u - bit)) & 1u))
        {
            /* A single bit of 0x80 >> 0..7 fits uint8_t. */
            p_packet[*p_bit / 8u] |= (uint8_t)(0x80u >> (*p_bit % 8u));
        }
        (*p_bit)++;
    }
}

/*** end of file ***/

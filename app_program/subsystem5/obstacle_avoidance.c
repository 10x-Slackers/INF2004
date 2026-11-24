/** @file obstacle_avoidance.c
 *
 * @brief Obstacle avoidance API and task. The task drives the bypass
 * through the motion control API (interface I3) and checks the motion
 * status itself, so it knows when the car has stopped.
 */

#include "obstacle_avoidance.h"
#include "ultrasonic_scan.h"
#include "subsystem5.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem3/line_following.h"

#define OBSTACLE_AVOIDANCE_POLL_MS  20u
#define OBSTACLE_AVOIDANCE_STILL_MS 1000u  /* Braking to standstill. */
#define OBSTACLE_AVOIDANCE_SCAN_MS  12000u /* Longest profile scan. */
#define OBSTACLE_AVOIDANCE_MOVE_MS  10000u
#define OBSTACLE_AVOIDANCE_STEPS    7u
#define OBSTACLE_AVOIDANCE_SPEED    300u /* mm/s, bypass moves and turns. */
#define OBSTACLE_AVOIDANCE_TURN_DEG 60   /* Diagonals' angle off the line. */
#define OBSTACLE_AVOIDANCE_SIN_TURN 866u /* sin 60 degrees, per 1000. */
#define OBSTACLE_AVOIDANCE_COS_TURN 500u /* cos 60 degrees, per 1000. */
#define OBSTACLE_AVOIDANCE_HALF_MM  85u  /* Half the car's width. */
#define OBSTACLE_AVOIDANCE_FRONT_MM 100u /* Axle to the sensor. */
#define OBSTACLE_AVOIDANCE_REAR_MM  80u  /* Axle to the back of the car. */
#define OBSTACLE_AVOIDANCE_GAP_MM   40u  /* Clearance kept everywhere. */
#define OBSTACLE_AVOIDANCE_BLOCKED  2u   /* In a row: skips a stray echo. */
#define OBSTACLE_AVOIDANCE_SHORT_MM 25u  /* Return diagonal cut, see below. */
#define OBSTACLE_AVOIDANCE_OUT      0u   /* Legs of the bypass. */
#define OBSTACLE_AVOIDANCE_PASS     1u
#define OBSTACLE_AVOIDANCE_BACK     2u
/* One bypass step: a turn towards the bypass side (+1) or back (-1), or,
 * with no turn (0), a move forward along leg. The first turn is
 * obstacle_avoidance_face()'s, which checks the diagonal. */
typedef struct
{
    int8_t turn;
    uint8_t leg;
} obstacle_avoidance_step_t;

static const obstacle_avoidance_step_t g_steps[OBSTACLE_AVOIDANCE_STEPS] = {
    {1, 0u},  {0, OBSTACLE_AVOIDANCE_OUT},
    {-1, 0u}, {0, OBSTACLE_AVOIDANCE_PASS},
    {-1, 0u}, {0, OBSTACLE_AVOIDANCE_BACK},
    {1, 0u}};

static ID gh_mutex;
static ID gh_task;
static obstacle_avoidance_status_t g_status;

static void obstacle_avoidance_task(INT stacd, void * p_exinf);
static bool obstacle_avoidance_running(uint32_t state);
static void obstacle_avoidance_set(uint32_t state, uint32_t step);
static void obstacle_avoidance_stop(uint32_t fault);
static bool obstacle_avoidance_active(void);
static uint32_t obstacle_avoidance_ms(void);
static bool obstacle_avoidance_scan(uint32_t mode, ultrasonic_scan_t * p_scan);
static bool obstacle_avoidance_still(void);
static bool obstacle_avoidance_move(int32_t turn_deg, uint32_t distance_mm);
static uint32_t obstacle_avoidance_diagonal(const scan_profile_t * p_profile,
                                            uint32_t side);
static bool obstacle_avoidance_turn(uint32_t side, int32_t turns);
static bool obstacle_avoidance_step(uint32_t side, uint32_t step,
                                    const uint32_t * p_legs);
static bool obstacle_avoidance_path(const scan_profile_t * p_profile,
                                    uint32_t side);
static uint32_t obstacle_avoidance_face(const scan_profile_t * p_profile,
                                        uint32_t side);
static void obstacle_avoidance_avoid(void);

/*!
 * @brief Create the status lock.
 *
 * @return True if it was created.
 */
bool obstacle_avoidance_init(void)
{
    T_CMTX mutex = {0};

    mutex.mtxatr = TA_INHERIT;
    gh_mutex = tk_cre_mtx(&mutex);
    return (gh_mutex > 0);
}

/*!
 * @brief Start the avoidance task, and keep its ID for
 * obstacle_avoidance_start() to wake it.
 */
ID obstacle_avoidance_start_task(void)
{
    /* FP is the kernel's generic task pointer type. */
    gh_task = app_start_task((FP)obstacle_avoidance_task, S5_PRI_AVOID, 0,
                             "s5_avoid");
    return gh_task;
}

/*!
 * @brief Clear the faults and start watching the range ahead. Also leaves
 * a safe stop. Does nothing if already started.
 */
void obstacle_avoidance_start(void)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    if (!obstacle_avoidance_running(g_status.state))
    {
        g_status.state = OBSTACLE_AVOIDANCE_MONITOR;
        app_fault_clear(APP_FAULT_AVOID);
        (void)tk_wup_tsk(gh_task);
    }
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Stop watching. If avoidance is driving the car, brake it.
 */
void obstacle_avoidance_cancel(void)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    if ((OBSTACLE_AVOIDANCE_PROFILE == g_status.state) ||
        (OBSTACLE_AVOIDANCE_BYPASS == g_status.state))
    {
        motion_control_stop();
    }
    g_status.state = OBSTACLE_AVOIDANCE_IDLE;
    g_status.step = 0u;
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Copy the status.
 */
void obstacle_avoidance_get_status(obstacle_avoidance_status_t * p_status)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    *p_status = g_status;
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Avoidance task: a forward reading after another while started;
 * avoid when OBSTACLE_AVOIDANCE_BLOCKED in a row are blocked.
 */
static void obstacle_avoidance_task(INT stacd, void * p_exinf)
{
    ultrasonic_scan_t scan = {0};
    uint32_t blocked = 0u;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        if (!obstacle_avoidance_active())
        {
            blocked = 0u;
            (void)tk_slp_tsk(TMO_FEVR);
        }
        else if (!obstacle_avoidance_scan(ULTRASONIC_SCAN_FORWARD, &scan))
        {
            /* Cancelled, or stopped on a sensor fault. */
        }
        else
        {
            /* Clear ahead restarts the count; the next reading follows
             * straight away. */
            blocked = (scan.forward_mm <= SCAN_PROFILE_BLOCK_MM)
                          ? (blocked + 1u)
                          : 0u;
            if (OBSTACLE_AVOIDANCE_BLOCKED == blocked)
            {
                blocked = 0u;
                obstacle_avoidance_avoid();
            }
        }
    }
}

/*!
 * @brief True for a state that is neither cancelled nor stopped.
 */
static bool obstacle_avoidance_running(uint32_t state)
{
    return (OBSTACLE_AVOIDANCE_IDLE != state) &&
           (OBSTACLE_AVOIDANCE_STOPPED != state);
}

/*!
 * @brief Set the state and the bypass step, unless cancelled or stopped
 * meanwhile.
 */
static void obstacle_avoidance_set(uint32_t state, uint32_t step)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    if (obstacle_avoidance_running(g_status.state))
    {
        g_status.state = state;
        g_status.step = step;
    }
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Safe stop: brake, set the fault (APP_FAULT_AVOID_...), wait for
 * the next start.
 * Nothing happens if cancelled meanwhile, so the car is the caller's again.
 */
static void obstacle_avoidance_stop(uint32_t fault)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    if (obstacle_avoidance_running(g_status.state))
    {
        motion_control_stop();
        g_status.state = OBSTACLE_AVOIDANCE_STOPPED;
        app_fault_set(fault);
        g_status.step = 0u;
    }
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief True while started and not stopped or cancelled.
 */
static bool obstacle_avoidance_active(void)
{
    obstacle_avoidance_status_t status = {0};

    obstacle_avoidance_get_status(&status);
    return obstacle_avoidance_running(status.state);
}

/*!
 * @brief Milliseconds on the kernel clock, for timeouts: counting polls
 * would add a tick to each, as tk_dly_tsk() does.
 */
static uint32_t obstacle_avoidance_ms(void)
{
    SYSTIM now = {0};

    (void)tk_get_otm(&now);
    return now.lo; /* Wraps; compare differences. */
}

/*!
 * @brief Request a scan and wait for its result. A missing echo or result
 * is a safe stop.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_scan(uint32_t mode, ultrasonic_scan_t * p_scan)
{
    uint32_t const request = ultrasonic_scan_request(mode);
    uint32_t const start_ms = obstacle_avoidance_ms();

    ultrasonic_scan_get(p_scan);
    /* An older scan may finish first; only the one for this request counts. */
    while ((obstacle_avoidance_active()) && (request != p_scan->request) &&
           ((obstacle_avoidance_ms() - start_ms) < OBSTACLE_AVOIDANCE_SCAN_MS))
    {
        tk_dly_tsk(OBSTACLE_AVOIDANCE_POLL_MS);
        ultrasonic_scan_get(p_scan);
    }
    if ((obstacle_avoidance_active()) &&
        ((request != p_scan->request) ||
         (0u != (app_faults() & APP_FAULT_NO_ECHO))))
    {
        obstacle_avoidance_stop(APP_FAULT_AVOID_SENSOR);
    }
    return obstacle_avoidance_active();
}

/*!
 * @brief Wait until both wheels have stopped turning.
 *
 * @return False if they still turn after 1 s.
 */
static bool obstacle_avoidance_still(void)
{
    motion_control_status_t status = {0};
    uint32_t const start_ms = obstacle_avoidance_ms();

    do
    {
        tk_dly_tsk(OBSTACLE_AVOIDANCE_POLL_MS);
        motion_control_get_status(&status);
    } while (
        ((0 != status.left_speed) || (0 != status.right_speed)) &&
        ((obstacle_avoidance_ms() - start_ms) < OBSTACLE_AVOIDANCE_STILL_MS));
    return (0 == status.left_speed) && (0 == status.right_speed);
}

/*!
 * @brief Turn the car (positive is left) or, with no turn, drive forward
 * by distance_mm, and wait for the move to finish. A forward move keeps reading
 * the range and stops within SCAN_PROFILE_BLOCK_MM of anything ahead.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_move(int32_t turn_deg, uint32_t distance_mm)
{
    motion_control_status_t status = {0};
    ultrasonic_scan_t scan = {0};
    uint32_t start_ms = 0u;

    /* Under the lock, so a cancel cannot slip in before the command. */
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    /* The cruise speed is shared: the controller's junction moves keep it. */
    (void)motion_control_set_speed(OBSTACLE_AVOIDANCE_SPEED);
    if (!obstacle_avoidance_running(g_status.state))
    {
        /* Cancelled or stopped. */
    }
    else if (0 == turn_deg)
    {
        (void)motion_control_move_forward(distance_mm);
    }
    else if (turn_deg > 0)
    {
        (void)motion_control_turn_left((uint32_t)turn_deg); /* Positive. */
    }
    else
    {
        /* Negative here, so -turn_deg is positive and fits. */
        (void)motion_control_turn_right((uint32_t)-turn_deg);
    }
    (void)tk_unl_mtx(gh_mutex);

    start_ms = obstacle_avoidance_ms();
    do
    {
        if (0 != turn_deg)
        {
            tk_dly_tsk(OBSTACLE_AVOIDANCE_POLL_MS);
        }
        else if ((obstacle_avoidance_scan(ULTRASONIC_SCAN_FORWARD, &scan)) &&
                 (scan.forward_mm <= SCAN_PROFILE_BLOCK_MM))
        {
            obstacle_avoidance_stop(APP_FAULT_AVOID_BLOCKED);
        }
        else
        {
            /* Clear ahead, or already stopped by the scan. */
        }
        motion_control_get_status(&status);
    } while (
        (obstacle_avoidance_active()) &&
        (MOTION_CONTROL_RUNNING == status.state) &&
        ((obstacle_avoidance_ms() - start_ms) < OBSTACLE_AVOIDANCE_MOVE_MS));
    if ((obstacle_avoidance_active()) &&
        (MOTION_CONTROL_COMPLETE != status.state))
    {
        obstacle_avoidance_stop(APP_FAULT_AVOID_MOTION);
    }
    return obstacle_avoidance_active();
}

/*!
 * @brief Diagonal towards side: far enough out to clear the obstacle's edge
 * on that side.
 */
static uint32_t obstacle_avoidance_diagonal(const scan_profile_t * p_profile,
                                            uint32_t side)
{
    /* The run of rays always contains straight ahead: no underflow. */
    uint32_t const edge_deg =
        (SCAN_PROFILE_LEFT == side)
            ? (p_profile->left_deg - SCAN_PROFILE_AHEAD_DEG)
            : (SCAN_PROFILE_AHEAD_DEG - p_profile->right_deg);
    /* Edge offset as an arc at SCAN_PROFILE_BLOCK_MM, the furthest a
     * blocked edge ray reads: never under the true one. */
    uint32_t const side_mm =
        scan_profile_arc_mm(SCAN_PROFILE_BLOCK_MM, edge_deg) +
        OBSTACLE_AVOIDANCE_HALF_MM + OBSTACLE_AVOIDANCE_GAP_MM;

    return (side_mm * 1000u) / OBSTACLE_AVOIDANCE_SIN_TURN;
}

/*!
 * @brief Turn by turns times OBSTACLE_AVOIDANCE_TURN_DEG, towards side if
 * positive, back if negative.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_turn(uint32_t side, int32_t turns)
{
    /* Left is positive. */
    return obstacle_avoidance_move(
        ((SCAN_PROFILE_LEFT == side) ? turns : -turns) *
            OBSTACLE_AVOIDANCE_TURN_DEG,
        0u);
}

/*!
 * @brief Run one bypass step and wait for it to finish.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_step(uint32_t side, uint32_t step,
                                    const uint32_t * p_legs)
{
    const obstacle_avoidance_step_t * p_step = &g_steps[step];

    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_BYPASS, step + 1u);
    return (0 != p_step->turn)
               ? obstacle_avoidance_turn(side, p_step->turn)
               : obstacle_avoidance_move(0, p_legs[p_step->leg]);
}

/*!
 * @brief With the car facing side, read the range along the diagonal.
 *
 * @return True if the diagonal and the stopping gap fit.
 */
static bool obstacle_avoidance_path(const scan_profile_t * p_profile,
                                    uint32_t side)
{
    ultrasonic_scan_t scan = {0};

    return (obstacle_avoidance_scan(ULTRASONIC_SCAN_FORWARD, &scan)) &&
           (scan.forward_mm >= (obstacle_avoidance_diagonal(p_profile, side) +
                                SCAN_PROFILE_BLOCK_MM));
}

/*!
 * @brief Bypass step 1: turn towards side and check the diagonal, which
 * the servo cannot see. If it is short, turn round to the other side,
 * if the scan found that one open, and check it. Neither is a safe stop,
 * turned back to the original heading.
 *
 * @return The side faced, or SCAN_PROFILE_NONE if cancelled or stopped.
 */
static uint32_t obstacle_avoidance_face(const scan_profile_t * p_profile,
                                        uint32_t side)
{
    uint32_t const other_mm =
        (SCAN_PROFILE_LEFT == side) ? p_profile->right_mm : p_profile->left_mm;
    bool b_clear = false;

    if (SCAN_PROFILE_NONE != side)
    {
        obstacle_avoidance_set(OBSTACLE_AVOIDANCE_BYPASS, 1u);
        b_clear = (obstacle_avoidance_turn(side, 1)) &&
                  (obstacle_avoidance_path(p_profile, side));
        if ((!b_clear) && (obstacle_avoidance_active()) &&
            (other_mm >= SCAN_PROFILE_CLEAR_MM))
        {
            side = (SCAN_PROFILE_LEFT == side) ? SCAN_PROFILE_RIGHT
                                               : SCAN_PROFILE_LEFT;
            b_clear = (obstacle_avoidance_turn(side, 2)) &&
                      (obstacle_avoidance_path(p_profile, side));
        }
    }
    if (!b_clear)
    {
        if (SCAN_PROFILE_NONE != side)
        {
            /* Does nothing if cancelled or stopped meanwhile. */
            (void)obstacle_avoidance_turn(side, -1);
        }
        obstacle_avoidance_stop(APP_FAULT_AVOID_BLOCKED);
        side = SCAN_PROFILE_NONE;
    }
    return side;
}

/*!
 * @brief Stop, profile the obstacle and the clearance each side, and drive
 * round it on the more open side: out on a diagonal, straight past, and
 * back on a diagonal to the original line and heading. Back to watching if the
 * obstacle has gone.
 */
static void obstacle_avoidance_avoid(void)
{
    ultrasonic_scan_t scan = {0};
    uint32_t side = SCAN_PROFILE_NONE;
    uint32_t legs[3] = {0u}; /* OBSTACLE_AVOIDANCE_OUT, _PASS, _BACK. */
    uint32_t step = 0u;

    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_PROFILE, 0u);
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    if (OBSTACLE_AVOIDANCE_PROFILE == g_status.state)
    {
        /* Brakes, and the line follower cannot drive again before the
         * controller restarts it after the bypass. */
        line_following_stop();
    }
    (void)tk_unl_mtx(gh_mutex);
    if (!obstacle_avoidance_active())
    {
        /* Cancelled. */
    }
    else if (!obstacle_avoidance_still())
    {
        obstacle_avoidance_stop(APP_FAULT_AVOID_MOTION);
    }
    else if ((obstacle_avoidance_scan(ULTRASONIC_SCAN_PROFILE, &scan)) &&
             (0u != scan.profile.b_obstacle))
    {
        side = scan_profile_side(scan.profile.left_mm, scan.profile.right_mm);
        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_status.side = side;
        g_status.left_mm = scan.profile.left_mm;
        g_status.right_mm = scan.profile.right_mm;
        (void)tk_unl_mtx(gh_mutex);
        side = obstacle_avoidance_face(&scan.profile, side);
        legs[OBSTACLE_AVOIDANCE_OUT] =
            obstacle_avoidance_diagonal(&scan.profile, side);
        /* Axle travel along the line until the back clears the obstacle,
         * taken as twice as long as its blocked edges are apart, less what
         * the diagonal covers. At least 240 against at most 163: no
         * underflow. */
        legs[OBSTACLE_AVOIDANCE_PASS] =
            OBSTACLE_AVOIDANCE_FRONT_MM + scan.profile.closest_mm +
            (2u * scan_profile_width_mm(scan.profile.closest_mm,
                                        scan.profile.left_deg -
                                            scan.profile.right_deg)) +
            OBSTACLE_AVOIDANCE_REAR_MM + OBSTACLE_AVOIDANCE_GAP_MM -
            ((legs[OBSTACLE_AVOIDANCE_OUT] * OBSTACLE_AVOIDANCE_COS_TURN) /
             1000u);
        /* Back short of the line, so the line sensors, ahead of the axle,
         * do not cross it before the last turn. The diagonal is at least
         * 144 mm: no underflow. */
        legs[OBSTACLE_AVOIDANCE_BACK] =
            legs[OBSTACLE_AVOIDANCE_OUT] - OBSTACLE_AVOIDANCE_SHORT_MM;
        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_status.side = side; /* Faced, after any turn round. */
        (void)tk_unl_mtx(gh_mutex);
        /* Step 1, the turn, is done. */
        for (step = 1u;
             (SCAN_PROFILE_NONE != side) && (step < OBSTACLE_AVOIDANCE_STEPS) &&
             (obstacle_avoidance_step(side, step, legs));
             step++)
        {
            /* Each step runs in the loop test. */
        }
        if (OBSTACLE_AVOIDANCE_STEPS == step)
        {
            (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
            g_status.bypasses++;
            (void)tk_unl_mtx(gh_mutex);
        }
    }
    else
    {
        /* Cancelled, stopped, or the obstacle has gone. */
    }
    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_MONITOR, 0u);
}

/*** end of file ***/

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
#define OBSTACLE_AVOIDANCE_SPEED    250u /* mm/s, bypass moves and turns. */
#define OBSTACLE_AVOIDANCE_TURN_DEG 60   /* Diagonals' angle off the line. */
#define OBSTACLE_AVOIDANCE_SIN_TURN 866u /* sin 60 degrees, per 1000. */
#define OBSTACLE_AVOIDANCE_COS_TURN 500u /* cos 60 degrees, per 1000. */
#define OBSTACLE_AVOIDANCE_HALF_MM  85u  /* Half the car's width. */
#define OBSTACLE_AVOIDANCE_FRONT_MM 100u /* Axle to the sensor. */
#define OBSTACLE_AVOIDANCE_REAR_MM  80u  /* Axle to the back of the car. */
#define OBSTACLE_AVOIDANCE_GAP_MM   40u  /* Clearance kept everywhere. */
#define OBSTACLE_AVOIDANCE_BLOCKED  2u   /* In a row: skips a stray echo. */
#define OBSTACLE_AVOIDANCE_LINE_MM  115u /* Line sensors ahead of axle. */
#define OBSTACLE_AVOIDANCE_EARLY_MM 100u /* Line watched this far early. */
#define OBSTACLE_AVOIDANCE_RETURN   6u   /* Step of the return diagonal. */
#define OBSTACLE_AVOIDANCE_OVER_MM  150u /* Return runs on to find the line. */
/* Both sides blocked: back off until the obstacle is this far ahead, still
 * within SCAN_PROFILE_BLOCK_MM for the profile, and scan again once. */
#define OBSTACLE_AVOIDANCE_RETRY_MM 180u
#define OBSTACLE_AVOIDANCE_OUT      0u /* Legs of the bypass. */
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
    {-1, 0u}, {0, OBSTACLE_AVOIDANCE_BACK}, /* Back: out plus OVER_MM. */
    {1, 0u}};

static ID gh_mutex = 0;
static ID gh_task = 0;
static obstacle_avoidance_status_t g_status = {0};
/* Return diagonal, under gh_mutex: where the line is watched for. */
static struct
{
    unsigned int start_mm : 24; /* Motion distance_mm at its start. */
    unsigned int from_mm : 9;   /* Watched from this far along it. */
    bool b_found : 1;           /* Line seen; rolling the axle onto it. */
} g_watch = {0};

static void obstacle_avoidance_task(INT stacd, void * p_exinf);
static bool obstacle_avoidance_running(uint32_t state);
static void obstacle_avoidance_set(uint32_t state, uint32_t step);
static void obstacle_avoidance_stop(uint32_t fault);
static bool obstacle_avoidance_active(void);
static uint32_t obstacle_avoidance_ms(void);
static bool obstacle_avoidance_scan(uint32_t mode, ultrasonic_scan_t * p_scan);
static bool obstacle_avoidance_still(void);
static bool obstacle_avoidance_move(int32_t turn_deg, int32_t distance_mm);
static uint32_t obstacle_avoidance_diagonal(const scan_profile_t * p_profile,
                                            uint32_t side);
static bool obstacle_avoidance_turn(uint32_t side, int32_t turns);
static bool obstacle_avoidance_step(uint32_t side, uint32_t step,
                                    const uint32_t * p_legs);
static bool obstacle_avoidance_path(const scan_profile_t * p_profile,
                                    uint32_t side);
static uint32_t obstacle_avoidance_face(const scan_profile_t * p_profile,
                                        uint32_t side);
static bool obstacle_avoidance_bypass(const scan_profile_t * p_profile);
static bool obstacle_avoidance_try(void);
static bool obstacle_avoidance_back(void);
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
    gh_task =
        app_start_task(obstacle_avoidance_task, S5_PRI_AVOID, 0u, "s5_avoid");
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
 * @brief A line sensor sees black. Near where the line should be on the
 * return diagonal, cut the diagonal short so the car stops with its axle
 * on the line, then turns along it. Elsewhere, nothing.
 */
void obstacle_avoidance_line(void)
{
    motion_control_status_t motion = {0};

    /* Under the lock, so the diagonal's own command has been given. */
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    motion_control_get_status(&motion);
    if ((OBSTACLE_AVOIDANCE_BYPASS == g_status.state) &&
        (OBSTACLE_AVOIDANCE_RETURN == g_status.step) && (!g_watch.b_found) &&
        (MOTION_CONTROL_RUNNING == motion.state) &&
        (MOTION_CONTROL_CMD_FORWARD == motion.command) &&
        (motion_control_mm_since(g_watch.start_mm, motion.distance_mm) >=
         g_watch.from_mm))
    {
        g_watch.b_found = true;
        (void)motion_control_move_forward(OBSTACLE_AVOIDANCE_LINE_MM);
    }
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
 * by distance_mm (back if negative), and wait for the move to finish. A
 * forward move keeps reading the range and stops within
 * SCAN_PROFILE_BLOCK_MM of anything ahead.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_move(int32_t turn_deg, int32_t distance_mm)
{
    motion_control_status_t status = {0};
    ultrasonic_scan_t scan = {0};
    uint32_t start_ms = 0u;

    /* Under the lock, so a cancel cannot slip in before the command. */
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    /* The cruise speed is shared: the controller sets its own for junctions. */
    (void)motion_control_set_speed(OBSTACLE_AVOIDANCE_SPEED);
    if (!obstacle_avoidance_running(g_status.state))
    {
        /* Cancelled or stopped. */
    }
    else if (distance_mm > 0)
    {
        (void)motion_control_move_forward((uint32_t)distance_mm);
    }
    else if (distance_mm < 0)
    {
        /* Negative here, so -distance_mm is positive and fits. */
        (void)motion_control_move_backward((uint32_t)(-distance_mm));
    }
    else if (turn_deg > 0)
    {
        (void)motion_control_turn_left((uint32_t)turn_deg); /* Positive. */
    }
    else
    {
        /* Negative here, so -turn_deg is positive and fits. */
        (void)motion_control_turn_right((uint32_t)(-turn_deg));
    }
    (void)tk_unl_mtx(gh_mutex);

    start_ms = obstacle_avoidance_ms();
    do
    {
        if (distance_mm <= 0)
        {
            /* A turn, or back: nothing to read ahead. */
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
        /* Under the lock, so obstacle_avoidance_line() cannot replace a
         * move this loop has just seen complete. */
        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        motion_control_get_status(&status);
        (void)tk_unl_mtx(gh_mutex);
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
    const obstacle_avoidance_step_t * const p_step = &g_steps[step];
    motion_control_status_t motion = {0};
    uint32_t const ahead_mm =
        OBSTACLE_AVOIDANCE_LINE_MM + OBSTACLE_AVOIDANCE_EARLY_MM;

    if (OBSTACLE_AVOIDANCE_RETURN == (step + 1u))
    {
        /* The sensors should reach the line LINE_MM before
         * the end. The diagonal is under 330 mm, so from_mm is under 115;
         * its 9 bits leave room for a larger GAP_MM or HALF_MM. */
        motion_control_get_status(&motion);
        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_watch.start_mm = motion.distance_mm;
        g_watch.from_mm = (p_legs[OBSTACLE_AVOIDANCE_OUT] > ahead_mm)
                              ? (p_legs[OBSTACLE_AVOIDANCE_OUT] - ahead_mm)
                              : 0u;
        g_watch.b_found = false;
        (void)tk_unl_mtx(gh_mutex);
    }
    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_BYPASS, step + 1u);
    return (0 != p_step->turn)
               ? obstacle_avoidance_turn(side, p_step->turn)
               : obstacle_avoidance_move(0, (int32_t)p_legs[p_step->leg]);
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
 * the servo cannot see. If it is short, turn round to the other side and
 * check it. Neither turns back to the original heading.
 *
 * @return The side faced, or SCAN_PROFILE_NONE if neither is clear,
 *         cancelled or stopped.
 */
static uint32_t obstacle_avoidance_face(const scan_profile_t * p_profile,
                                        uint32_t side)
{
    bool b_clear = false;

    if (SCAN_PROFILE_NONE != side)
    {
        obstacle_avoidance_set(OBSTACLE_AVOIDANCE_BYPASS, 1u);
        b_clear = (obstacle_avoidance_turn(side, 1)) &&
                  (obstacle_avoidance_path(p_profile, side));
        if ((!b_clear) && (obstacle_avoidance_active()))
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
        side = SCAN_PROFILE_NONE;
    }
    return side;
}

/*!
 * @brief Drive round the obstacle on the more open side: out on a diagonal,
 * straight past, and back on a diagonal to the original line (cut short
 * where the line sensors find it, obstacle_avoidance_line()) and heading.
 *
 * @param[in] p_profile The obstacle and the clearance each side.
 * @return True if both sides are blocked, back at the original heading.
 */
static bool obstacle_avoidance_bypass(const scan_profile_t * p_profile)
{
    uint32_t side = scan_profile_side(p_profile->left_mm, p_profile->right_mm);
    uint32_t legs[3] = {0u}; /* OBSTACLE_AVOIDANCE_OUT, _PASS, _BACK. */
    uint32_t step = 1u;      /* Step 1, the turn, is face(). */

    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    g_status.side = side;
    g_status.left_mm = p_profile->left_mm;
    g_status.right_mm = p_profile->right_mm;
    (void)tk_unl_mtx(gh_mutex);
    side = obstacle_avoidance_face(p_profile, side);
    legs[OBSTACLE_AVOIDANCE_OUT] = obstacle_avoidance_diagonal(p_profile, side);
    /* Longer than out, so heading error from the turns cannot leave the car
     * short of the line; the line sensors end it at the line. */
    legs[OBSTACLE_AVOIDANCE_BACK] =
        legs[OBSTACLE_AVOIDANCE_OUT] + OBSTACLE_AVOIDANCE_OVER_MM;
    /* Axle travel along the line until the back clears the obstacle, its
     * length taken as the distance between its blocked edges, less what
     * the diagonal covers. At least 240 against at most 163: no
     * underflow. */
    legs[OBSTACLE_AVOIDANCE_PASS] =
        OBSTACLE_AVOIDANCE_FRONT_MM + p_profile->closest_mm +
        scan_profile_width_mm(p_profile->closest_mm,
                              p_profile->left_deg - p_profile->right_deg) +
        OBSTACLE_AVOIDANCE_REAR_MM + OBSTACLE_AVOIDANCE_GAP_MM -
        ((legs[OBSTACLE_AVOIDANCE_OUT] * OBSTACLE_AVOIDANCE_COS_TURN) / 1000u);
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    g_status.side = side; /* Faced, after any turn round. */
    (void)tk_unl_mtx(gh_mutex);
    while ((SCAN_PROFILE_NONE != side) && (step < OBSTACLE_AVOIDANCE_STEPS) &&
           (obstacle_avoidance_step(side, step, legs)))
    {
        step++;
    }
    if (OBSTACLE_AVOIDANCE_STEPS == step)
    {
        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_status.bypasses++;
        (void)tk_unl_mtx(gh_mutex);
    }
    return (SCAN_PROFILE_NONE == side) && (obstacle_avoidance_active());
}

/*!
 * @brief Profile the obstacle and drive round it.
 *
 * @return True if both sides are blocked, back at the original heading.
 */
static bool obstacle_avoidance_try(void)
{
    ultrasonic_scan_t scan = {0};
    bool b_blocked = false;

    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_PROFILE, 0u);
    if ((obstacle_avoidance_scan(ULTRASONIC_SCAN_PROFILE, &scan)) &&
        (scan.profile.b_obstacle))
    {
        APP_PRINTF("[avoid] obstacle: width %u mm, closest %u mm at %u deg, "
                   "clear left %u mm, right %u mm\n",
                   scan.profile.width_mm, scan.profile.closest_mm,
                   scan.profile.closest_deg, scan.profile.left_mm,
                   scan.profile.right_mm);
        b_blocked = obstacle_avoidance_bypass(&scan.profile);
    }
    return b_blocked;
}

/*!
 * @brief Back off until the obstacle is OBSTACLE_AVOIDANCE_RETRY_MM ahead:
 * stopped too close, the side rays may hit the obstacle itself. Blind, as
 * nothing reads behind the car, but only a few cm.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_back(void)
{
    ultrasonic_scan_t scan = {0};

    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_PROFILE, 0u);
    return (obstacle_avoidance_scan(ULTRASONIC_SCAN_FORWARD, &scan)) &&
           ((scan.forward_mm >= OBSTACLE_AVOIDANCE_RETRY_MM) ||
            (obstacle_avoidance_move(
                0, (int32_t)scan.forward_mm -
                       (int32_t)OBSTACLE_AVOIDANCE_RETRY_MM)));
}

/*!
 * @brief Stop and profile the obstacle and the clearance each side, then
 * drive round it. With both sides blocked, back off and try once more;
 * still blocked is a safe stop. Back to watching afterwards, or at once if
 * the obstacle has gone.
 */
static void obstacle_avoidance_avoid(void)
{
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
    else if ((obstacle_avoidance_try()) && (obstacle_avoidance_back()) &&
             (obstacle_avoidance_try()))
    {
        obstacle_avoidance_stop(APP_FAULT_AVOID_BLOCKED);
    }
    else
    {
        /* Passed, cancelled, stopped, or the obstacle has gone. */
    }
    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_MONITOR, 0u);
}

/*** end of file ***/

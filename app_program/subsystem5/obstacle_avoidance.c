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

#define OBSTACLE_AVOIDANCE_POLL_MS  20    /* Two kernel ticks. */
#define OBSTACLE_AVOIDANCE_STILL_MS 1000u /* Braking to standstill. */
#define OBSTACLE_AVOIDANCE_SCAN_MS  8000u /* Longest profile scan. */
#define OBSTACLE_AVOIDANCE_MOVE_MS  10000u
#define OBSTACLE_AVOIDANCE_STEPS    7u
#define OBSTACLE_AVOIDANCE_TURN_DEG 90
#define OBSTACLE_AVOIDANCE_LOOK_DEG 45   /* Car turn for the clearance. */
#define OBSTACLE_AVOIDANCE_SIDE_MM  300u /* Out to the side and back. */
#define OBSTACLE_AVOIDANCE_PASS_MM  400u /* Alongside the obstacle. */

/* One bypass step: a turn towards the bypass side (+1) or back (-1), or a
 * move forward (0) by mm. */
typedef struct
{
    int8_t turn;
    uint16_t mm;
} obstacle_avoidance_step_t;

static const obstacle_avoidance_step_t g_steps[OBSTACLE_AVOIDANCE_STEPS] = {
    {1, 0u},  {0, OBSTACLE_AVOIDANCE_SIDE_MM},
    {-1, 0u}, {0, OBSTACLE_AVOIDANCE_PASS_MM},
    {-1, 0u}, {0, OBSTACLE_AVOIDANCE_SIDE_MM},
    {1, 0u}};

static ID g_mutex;
static ID g_task;
static obstacle_avoidance_status_t g_status;

static void obstacle_avoidance_set(uint32_t state, uint32_t step);
static void obstacle_avoidance_stop(uint32_t fault);
static bool obstacle_avoidance_active(void);
static bool obstacle_avoidance_scan(uint32_t mode, ultrasonic_scan_t * p_scan);
static bool obstacle_avoidance_still(void);
static bool obstacle_avoidance_move(int32_t turn_deg, uint32_t mm);
static bool obstacle_avoidance_look(uint32_t * p_left_mm,
                                    uint32_t * p_right_mm);
static bool obstacle_avoidance_step(uint32_t side, uint32_t step);
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
    g_mutex = tk_cre_mtx(&mutex);
    return (g_mutex > 0);
}

/*!
 * @brief Clear the faults and start watching the range ahead. Also leaves
 * a safe stop. Does nothing if already started.
 */
void obstacle_avoidance_start(void)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    if ((OBSTACLE_AVOIDANCE_IDLE == g_status.state) ||
        (OBSTACLE_AVOIDANCE_STOPPED == g_status.state))
    {
        g_status.state = OBSTACLE_AVOIDANCE_MONITOR;
        g_status.faults = 0u;
    }
    (void)tk_unl_mtx(g_mutex);
    (void)tk_wup_tsk(g_task);
}

/*!
 * @brief Stop watching. If avoidance is driving the car, brake it.
 */
void obstacle_avoidance_cancel(void)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    if ((OBSTACLE_AVOIDANCE_PROFILE == g_status.state) ||
        (OBSTACLE_AVOIDANCE_BYPASS == g_status.state))
    {
        motion_control_stop();
    }
    g_status.state = OBSTACLE_AVOIDANCE_IDLE;
    g_status.step = 0u;
    (void)tk_unl_mtx(g_mutex);
}

/*!
 * @brief Copy the status.
 */
void obstacle_avoidance_get_status(obstacle_avoidance_status_t * p_status)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    *p_status = g_status;
    (void)tk_unl_mtx(g_mutex);
}

/*!
 * @brief Avoidance task: a forward reading after another while started;
 * avoid when one is blocked.
 */
void obstacle_avoidance_task(INT stacd, void * p_exinf)
{
    ultrasonic_scan_t scan = {0};

    (void)stacd;
    (void)p_exinf;
    g_task = tk_get_tid();
    for (;;)
    {
        if (!obstacle_avoidance_active())
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
        else if (!obstacle_avoidance_scan(ULTRASONIC_SCAN_FORWARD, &scan))
        {
            /* Cancelled, or stopped on a sensor fault. */
        }
        else if (scan.forward_mm <= SCAN_PROFILE_BLOCK_MM)
        {
            obstacle_avoidance_avoid();
        }
        else
        {
            /* Clear ahead: the next reading follows straight away. */
        }
    }
}

/*!
 * @brief Set the state and the bypass step, unless cancelled or stopped
 * meanwhile.
 */
static void obstacle_avoidance_set(uint32_t state, uint32_t step)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    if ((OBSTACLE_AVOIDANCE_IDLE != g_status.state) &&
        (OBSTACLE_AVOIDANCE_STOPPED != g_status.state))
    {
        g_status.state = state;
        g_status.step = step;
    }
    (void)tk_unl_mtx(g_mutex);
}

/*!
 * @brief Safe stop: brake, record the fault, wait for the next start.
 */
static void obstacle_avoidance_stop(uint32_t fault)
{
    motion_control_stop();
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    g_status.state = OBSTACLE_AVOIDANCE_STOPPED;
    g_status.faults |= fault;
    g_status.step = 0u;
    (void)tk_unl_mtx(g_mutex);
}

/*!
 * @brief True while started and not stopped or cancelled.
 */
static bool obstacle_avoidance_active(void)
{
    obstacle_avoidance_status_t status = {0};

    obstacle_avoidance_get_status(&status);
    return (OBSTACLE_AVOIDANCE_IDLE != status.state) &&
           (OBSTACLE_AVOIDANCE_STOPPED != status.state);
}

/*!
 * @brief Request a scan and wait for its result. A missing echo or result
 * is a safe stop.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_scan(uint32_t mode, ultrasonic_scan_t * p_scan)
{
    uint32_t count = 0u;
    uint32_t waited = 0u;

    ultrasonic_scan_get(p_scan);
    count = p_scan->count;
    ultrasonic_scan_request(mode);
    while (obstacle_avoidance_active() && (count == p_scan->count) &&
           (waited < OBSTACLE_AVOIDANCE_SCAN_MS))
    {
        tk_dly_tsk(OBSTACLE_AVOIDANCE_POLL_MS);
        waited += OBSTACLE_AVOIDANCE_POLL_MS;
        ultrasonic_scan_get(p_scan);
    }
    if (obstacle_avoidance_active() &&
        ((count == p_scan->count) || (0u != p_scan->b_fault)))
    {
        obstacle_avoidance_stop(OBSTACLE_AVOIDANCE_FAULT_SENSOR);
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
    uint32_t waited = 0u;

    do
    {
        tk_dly_tsk(OBSTACLE_AVOIDANCE_POLL_MS);
        waited += OBSTACLE_AVOIDANCE_POLL_MS;
        motion_control_get_status(&status);
    } while (((0 != status.left_speed) || (0 != status.right_speed)) &&
             (waited < OBSTACLE_AVOIDANCE_STILL_MS));
    return (0 == status.left_speed) && (0 == status.right_speed);
}

/*!
 * @brief Turn the car (positive is left) or, with no turn, drive forward
 * by mm, and wait for the move to finish.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_move(int32_t turn_deg, uint32_t mm)
{
    motion_control_status_t status = {0};
    uint32_t waited = 0u;

    /* Under the lock, so a cancel cannot slip in before the command. */
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    if ((OBSTACLE_AVOIDANCE_IDLE == g_status.state) ||
        (OBSTACLE_AVOIDANCE_STOPPED == g_status.state))
    {
        /* Cancelled or stopped. */
    }
    else if (0 == turn_deg)
    {
        (void)motion_control_move_forward(mm);
    }
    else if (turn_deg > 0)
    {
        (void)motion_control_turn_left((uint32_t)turn_deg);
    }
    else
    {
        (void)motion_control_turn_right((uint32_t)-turn_deg);
    }
    (void)tk_unl_mtx(g_mutex);

    do
    {
        tk_dly_tsk(OBSTACLE_AVOIDANCE_POLL_MS);
        waited += OBSTACLE_AVOIDANCE_POLL_MS;
        motion_control_get_status(&status);
    } while (obstacle_avoidance_active() &&
             (MOTION_CONTROL_RUNNING == status.state) &&
             (waited < OBSTACLE_AVOIDANCE_MOVE_MS));
    if (obstacle_avoidance_active() &&
        (MOTION_CONTROL_COMPLETE != status.state))
    {
        obstacle_avoidance_stop(OBSTACLE_AVOIDANCE_FAULT_MOTION);
    }
    return obstacle_avoidance_active();
}

/*!
 * @brief Coarse scan: turn the car 45 degrees left, read the range, turn
 * 90 degrees right, read, and turn back to the original heading.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_look(uint32_t * p_left_mm, uint32_t * p_right_mm)
{
    ultrasonic_scan_t scan = {0};

    if (obstacle_avoidance_move(OBSTACLE_AVOIDANCE_LOOK_DEG, 0u) &&
        obstacle_avoidance_scan(ULTRASONIC_SCAN_FORWARD, &scan))
    {
        *p_left_mm = scan.forward_mm;
    }
    if (obstacle_avoidance_move(-2 * OBSTACLE_AVOIDANCE_LOOK_DEG, 0u) &&
        obstacle_avoidance_scan(ULTRASONIC_SCAN_FORWARD, &scan))
    {
        *p_right_mm = scan.forward_mm;
    }
    return obstacle_avoidance_move(OBSTACLE_AVOIDANCE_LOOK_DEG, 0u);
}

/*!
 * @brief Run one bypass step and wait for it to finish. Before a forward
 * move, check that the way ahead is clear.
 *
 * @return False if cancelled or stopped.
 */
static bool obstacle_avoidance_step(uint32_t side, uint32_t step)
{
    const obstacle_avoidance_step_t * p_step = &g_steps[step];
    ultrasonic_scan_t scan = {0};
    /* Towards the bypass side or back; left is positive. */
    int32_t turn_deg = p_step->turn * OBSTACLE_AVOIDANCE_TURN_DEG;

    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_BYPASS, step + 1u);
    if ((0 == p_step->turn) &&
        obstacle_avoidance_scan(ULTRASONIC_SCAN_FORWARD, &scan) &&
        (scan.forward_mm <= SCAN_PROFILE_BLOCK_MM))
    {
        obstacle_avoidance_stop(OBSTACLE_AVOIDANCE_FAULT_BLOCKED);
    }
    return obstacle_avoidance_move(
        (SCAN_PROFILE_LEFT == side) ? turn_deg : -turn_deg, p_step->mm);
}

/*!
 * @brief Stop, measure the clearance each side, profile the obstacle and
 * drive round it on the more open side. Back to watching if the obstacle
 * has gone.
 */
static void obstacle_avoidance_avoid(void)
{
    ultrasonic_scan_t scan = {0};
    uint32_t left_mm = 0u;
    uint32_t right_mm = 0u;
    uint32_t side = SCAN_PROFILE_NONE;
    uint32_t step = 0u;

    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_PROFILE, 0u);
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    if (OBSTACLE_AVOIDANCE_PROFILE == g_status.state)
    {
        motion_control_stop();
    }
    (void)tk_unl_mtx(g_mutex);
    if (!obstacle_avoidance_active())
    {
        /* Cancelled. */
    }
    else if (!obstacle_avoidance_still())
    {
        obstacle_avoidance_stop(OBSTACLE_AVOIDANCE_FAULT_MOTION);
    }
    else if (obstacle_avoidance_look(&left_mm, &right_mm) &&
             obstacle_avoidance_scan(ULTRASONIC_SCAN_PROFILE, &scan) &&
             (0u != scan.profile.b_obstacle))
    {
        side = scan_profile_side(left_mm, right_mm);
        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        g_status.side = side;
        g_status.left_mm = left_mm;
        g_status.right_mm = right_mm;
        (void)tk_unl_mtx(g_mutex);
        if (SCAN_PROFILE_NONE == side)
        {
            obstacle_avoidance_stop(OBSTACLE_AVOIDANCE_FAULT_BLOCKED);
        }
        for (step = 0u;
             (SCAN_PROFILE_NONE != side) && (step < OBSTACLE_AVOIDANCE_STEPS) &&
             obstacle_avoidance_step(side, step);
             step++)
        {
        }
        if (OBSTACLE_AVOIDANCE_STEPS == step)
        {
            (void)tk_loc_mtx(g_mutex, TMO_FEVR);
            g_status.bypasses++;
            (void)tk_unl_mtx(g_mutex);
        }
    }
    else
    {
        /* Cancelled, stopped, or the obstacle has gone. */
    }
    obstacle_avoidance_set(OBSTACLE_AVOIDANCE_MONITOR, 0u);
}

/*** end of file ***/

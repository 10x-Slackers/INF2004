/** @file safety_monitor.c
 *
 * @brief Safety Monitor task. Every 10 ms it checks the IMU faults and the
 * motion state, and latches a stop on either. Collision risk belongs to
 * obstacle avoidance, which stops on two readings within 150 mm. The latch is
 * motion control's own (motion_control_latch()), so it is kept in one place.
 */

#include "safety_monitor.h"
#include "../subsystem2/motion_control.h"

#define SAFETY_MONITOR_PERIOD_MS 10u

static void safety_monitor_task(INT stacd, void * p_exinf);

/*!
 * @brief Start the Safety Monitor task.
 */
ER safety_monitor_start(void)
{
    /* FP is the kernel's generic task pointer type. */
    return app_start_cyclic(app_start_task((FP)safety_monitor_task,
                                           SAFETY_MONITOR_PRI, 0u, "safety"),
                            SAFETY_MONITOR_PERIOD_MS);
}

/*!
 * @brief Latch a stop: brake and refuse every motion command until
 * safety_monitor_clear(). Prints the cause of the first stop only.
 *
 * @param[in] cause SAFETY_MONITOR_... bits, for the console.
 */
void safety_monitor_stop(uint32_t cause)
{
    /* One locked step, so two tasks cannot both see it unlatched. */
    if (!motion_control_latch(true))
    {
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[safety] stop, cause 0x%x\n", cause);
    }
}

/*!
 * @brief Release the latch, unless the IMU still reports a fault (tilt and
 * upside down stay latched until reset; lifted clears 1 s after a lift) or
 * motion control keeps it because its start-up failed.
 *
 * @return True if released.
 */
bool safety_monitor_clear(void)
{
    bool b_clear = (0u == (app_faults() & APP_FAULT_IMU));

    if (b_clear)
    {
        (void)motion_control_latch(false);
        b_clear = !motion_control_latched();
    }
    if (b_clear)
    {
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[safety] cleared\n");
    }
    return b_clear;
}

/*!
 * @brief Safety Monitor task: latch a stop on an IMU fault or a stall.
 */
static void safety_monitor_task(INT stacd, void * p_exinf)
{
    motion_control_status_t motion = {0};
    uint32_t cause = 0u;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* Woken every SAFETY_MONITOR_PERIOD_MS. */
        motion_control_get_status(&motion);
        cause =
            (0u != (app_faults() & APP_FAULT_IMU)) ? SAFETY_MONITOR_IMU : 0u;

        if (MOTION_CONTROL_FAILED == motion.state)
        {
            cause |= SAFETY_MONITOR_STALL;
        }
        if (0u != cause)
        {
            safety_monitor_stop(cause);
        }
    }
}

/*** end of file ***/

/** @file safety_monitor.c
 *
 * @brief Safety Monitor task. Every 10 ms it checks the IMU faults and the
 * motion state, and latches a stop on either. Collision risk belongs to
 * obstacle avoidance, which stops at 350 mm. The latch is motion control's own
 * (motion_control_latch()), so it is kept in one place only.
 */

#include "safety_monitor.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem4/subsystem4.h"

#define SAFETY_MONITOR_PERIOD_MS 10u

static void safety_monitor_task(INT stacd, void * p_exinf);
static bool safety_monitor_imu_fault(void);

/*!
 * @brief Start the Safety Monitor task.
 */
ER safety_monitor_start(void)
{
    /* FP is the kernel's generic task pointer type. */
    ID task = app_start_task((FP)safety_monitor_task, SAFETY_MONITOR_PRI, 0,
                             "safety");

    return (task < E_OK) ? task : E_OK;
}

/*!
 * @brief Latch a stop: brake and refuse every motion command until
 * safety_monitor_clear(). Prints the cause of the first stop only.
 *
 * @param[in] cause SAFETY_MONITOR_... bits, for the console.
 */
void safety_monitor_stop(uint32_t cause)
{
    if (!motion_control_latched())
    {
        motion_control_latch(true);
        tm_printf((UB *)"[safety] stop, cause 0x%x\n", cause);
    }
}

/*!
 * @brief Release the latch, unless the IMU still reports a fault (those
 * stay latched until reset).
 *
 * @return True if released.
 */
bool safety_monitor_clear(void)
{
    bool b_clear = !safety_monitor_imu_fault();

    if (b_clear)
    {
        motion_control_latch(false);
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
        (void)tk_dly_tsk(SAFETY_MONITOR_PERIOD_MS);
        motion_control_get_status(&motion);
        cause = safety_monitor_imu_fault() ? SAFETY_MONITOR_IMU : 0u;

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

/*!
 * @brief True while the IMU reports an impact or excessive tilt.
 */
static bool safety_monitor_imu_fault(void)
{
    s4_fault_t fault = {0};

    s4_get_fault(&fault);
    return (0u != fault.impact) || (0u != fault.tilt);
}

/*** end of file ***/

/** @file subsystem5.c
 *
 * @brief Subsystem 5 (ultrasonic scanning and obstacle avoidance) start-up.
 */

#include "subsystem5.h"

/*!
 * @brief Set up the sensor, the servo and the locks. Runs after
 * subsystem2_init(), whose GPIO interrupt carries the echo edges.
 */
EXPORT ER subsystem5_init(void)
{
    return (ultrasonic_scan_init() && obstacle_avoidance_init()) ? E_OK : E_SYS;
}

/*!
 * @brief Start the avoidance and scan tasks.
 */
EXPORT ER subsystem5_start(void)
{
    /* FP is the kernel's generic task pointer type. */
    ID avoid = app_start_task((FP)obstacle_avoidance_task, S5_PRI_AVOID, 0,
                              "s5_avoid");
    ID scan =
        app_start_task((FP)ultrasonic_scan_task, S5_PRI_SCAN, 0, "s5_scan");

    return (avoid < E_OK) ? avoid : ((scan < E_OK) ? scan : E_OK);
}

/*** end of file ***/

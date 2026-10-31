/** @file subsystem5.c
 *
 * @brief Subsystem 5 (ultrasonic scanning and obstacle avoidance) start-up.
 */

#include "subsystem5.h"

/*!
 * @brief Set up the sensor, the servo and the locks.
 */
ER subsystem5_init(void)
{
    bool const b_scan = ultrasonic_scan_init();
    bool const b_avoid = obstacle_avoidance_init();

    return (b_scan && b_avoid) ? E_OK : E_SYS;
}

/*!
 * @brief Start the avoidance and scan tasks.
 */
ER subsystem5_start(void)
{
    ID const h_avoid = obstacle_avoidance_start_task();
    /* FP is the kernel's generic task pointer type. */
    ID const h_scan =
        app_start_task((FP)ultrasonic_scan_task, S5_PRI_SCAN, 0u, "s5_scan");

    return (h_avoid < E_OK) ? h_avoid : ((h_scan < E_OK) ? h_scan : E_OK);
}

/*** end of file ***/

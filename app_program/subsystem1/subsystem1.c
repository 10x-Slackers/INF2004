/** @file subsystem1.c
 *
 * @brief Subsystem 1 (Wi-Fi, command and telemetry) start-up.
 *
 * subsystem1_init() creates this subsystem's kernel objects (semaphores,
 * mailboxes, ...) and starts no tasks. subsystem1_start() creates and
 * starts its tasks, e.g.
 *
 *     ID h_task = app_start_task((FP)subsystem1_task, S1_PRI_MAIN, 0u,
 *                                "s1_main");
 *     return (h_task < E_OK) ? h_task : E_OK;
 *
 * Drivers for this subsystem's hardware go in this folder as
 * drv_<name>.c / drv_<name>.h, called only from the task that owns the
 * peripheral (TEAM.md section 3). Record every pin, task, object and
 * interface in TEAM.md.
 */

#include "subsystem1.h"

/*!
 * @brief Create this subsystem's kernel objects.
 */
ER subsystem1_init(void)
{
    return E_OK;
}

/*!
 * @brief Create and start this subsystem's tasks.
 */
ER subsystem1_start(void)
{
    return E_OK;
}

/*** end of file ***/

/** @file subsystem3.c
 *
 * @brief Subsystem 3 (Line following and barcodes) start-up.
 *
 * subsystem3_init() creates this subsystem's kernel objects (semaphores,
 * mailboxes, ...) and starts no tasks. subsystem3_start() creates and
 * starts its tasks, e.g.
 *
 *     ID task_id = app_start_task((FP)subsystem3_task, S3_PRI_MAIN, 0,
 *                                 "s3_main");
 *     return (task_id < E_OK) ? task_id : E_OK;
 *
 * Drivers for this subsystem's hardware go in this folder as
 * drv_<name>.c / drv_<name>.h, called only from the task that owns the
 * peripheral (TEAM.md section 3). Record every pin, task, object and
 * interface in TEAM.md.
 */

#include "subsystem3.h"

/*!
 * @brief Create this subsystem's kernel objects.
 */
ER subsystem3_init(void)
{
    return E_OK;
}

/*!
 * @brief Create and start this subsystem's tasks.
 */
ER subsystem3_start(void)
{
    return E_OK;
}

/*** end of file ***/

/** @file subsystem4.c
 *
 * @brief Subsystem 4 (IMU and terrain) start-up: the mutex and the IMU
 * task.
 */

#include "subsystem4.h"

ID g_s4_mutex;

/*!
 * @brief Create the mutex that guards the IMU records.
 */
EXPORT ER subsystem4_init(void)
{
    T_CMTX mutex = {0};

    /* The records' bit-fields share memory words, so a reader on the
     * other core must never see a half-written update. */
    mutex.mtxatr = TA_INHERIT;
    g_s4_mutex = tk_cre_mtx(&mutex);
    return (g_s4_mutex < E_OK) ? g_s4_mutex : E_OK;
}

/*!
 * @brief Start the IMU task, on any core.
 */
EXPORT ER subsystem4_start(void)
{
    /* FP is the kernel's generic task pointer type. */
    ID task = app_start_task((FP)s4_imu_task, S4_PRI_IMU, 0, "s4_imu");

    return (task < E_OK) ? task : E_OK;
}

/*** end of file ***/

/** @file subsystem3.c
 *
 * @brief Subsystem 3 (Line following and barcodes) start-up.
 */

#include "subsystem3.h"

static ID g_line_task;

static void subsystem3_wake(void * p_exinf);

/*!
 * @brief Set up the line sensors and their lock.
 */
ER subsystem3_init(void)
{
    return line_following_init() ? E_OK : E_SYS;
}

/*!
 * @brief Start the Line Following task and its 10 ms cyclic handler.
 * tk_dly_tsk() would add a tick to every period.
 */
ER subsystem3_start(void)
{
    T_CCYC cyclic = {0};

    /* FP is the kernel's generic task pointer type. */
    g_line_task =
        app_start_task((FP)line_following_task, S3_PRI_LINE, 0, "s3_line");
    cyclic.cycatr = TA_HLNG | TA_STA;
    cyclic.cychdr = (FP)subsystem3_wake; /* Kernel handler type, as above. */
    cyclic.cyctim = LINE_FOLLOWING_PERIOD_MS;
    cyclic.cycphs = LINE_FOLLOWING_PERIOD_MS;
    return (g_line_task < E_OK) ? g_line_task : tk_cre_cyc(&cyclic);
}

/*!
 * @brief Cyclic handler: wake the Line Following task every 10 ms.
 */
static void subsystem3_wake(void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_line_task);
}

/*** end of file ***/

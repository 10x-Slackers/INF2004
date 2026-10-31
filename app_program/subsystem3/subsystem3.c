/** @file subsystem3.c
 *
 * @brief Subsystem 3 (Line following and barcodes) start-up.
 */

#include "subsystem3.h"

static ID g_barcode_task;
static ID g_line_task;

static void subsystem3_wake(void * p_exinf);

/*!
 * @brief Set up the sensors and their locks.
 */
ER subsystem3_init(void)
{
    bool b_line = line_following_init();
    bool b_barcode = barcode_decoder_init();

    return (b_line && b_barcode) ? E_OK : E_SYS;
}

/*!
 * @brief Start the Barcode Decoder and Line Following tasks and the
 * cyclic handler that wakes both every 10 ms.
 * tk_dly_tsk() would add a tick to every period.
 */
ER subsystem3_start(void)
{
    T_CCYC cyclic = {0};
    ER err = E_OK;

    /* FP is the kernel's generic task pointer type. */
    g_barcode_task = app_start_task((FP)barcode_decoder_task, S3_PRI_BARCODE, 0,
                                    "s3_barcode");
    g_line_task =
        app_start_task((FP)line_following_task, S3_PRI_LINE, 0, "s3_line");
    cyclic.cycatr = TA_HLNG | TA_STA;
    cyclic.cychdr = (FP)subsystem3_wake; /* Kernel handler type, as above. */
    cyclic.cyctim = LINE_FOLLOWING_PERIOD_MS;
    cyclic.cycphs = LINE_FOLLOWING_PERIOD_MS;
    err = (g_barcode_task < E_OK) ? g_barcode_task : g_line_task;
    return (err < E_OK) ? err : tk_cre_cyc(&cyclic);
}

/*!
 * @brief Cyclic handler: wake both tasks every 10 ms.
 */
static void subsystem3_wake(void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_barcode_task);
    (void)tk_wup_tsk(g_line_task);
}

/*** end of file ***/

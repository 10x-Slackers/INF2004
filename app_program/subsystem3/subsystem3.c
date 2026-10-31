/** @file subsystem3.c
 *
 * @brief Subsystem 3 (Line following and barcodes) start-up.
 */

#include "subsystem3.h"

/*!
 * @brief Set up the line sensors and their lock.
 */
ER subsystem3_init(void)
{
    return line_following_init() ? E_OK : E_SYS;
}

/*!
 * @brief Start the Line Following task.
 */
ER subsystem3_start(void)
{
    /* FP is the kernel's generic task pointer type. */
    ID line =
        app_start_task((FP)line_following_task, S3_PRI_LINE, 0, "s3_line");

    return (line < E_OK) ? line : E_OK;
}

/*** end of file ***/

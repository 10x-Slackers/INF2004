/** @file subsystem3.c
 *
 * @brief Subsystem 3 (Line following and barcodes) start-up.
 */

#include "subsystem3.h"

/*!
 * @brief Set up the sensors and their locks.
 */
ER subsystem3_init(void)
{
    bool const b_line = line_following_init();
    bool const b_barcode = barcode_decoder_init();

    return (b_line && b_barcode) ? E_OK : E_SYS;
}

/*!
 * @brief Start the Barcode Decoder and Line Following tasks, each woken
 * every 10 ms.
 */
ER subsystem3_start(void)
{
    ER const h_barcode_cyclic = app_start_cyclic(
        app_start_task(barcode_decoder_task, S3_PRI_BARCODE, 0, "s3_barcode"),
        LINE_FOLLOWING_PERIOD_MS);
    ER const h_line_cyclic = app_start_cyclic(
        app_start_task(line_following_task, S3_PRI_LINE, 0, "s3_line"),
        LINE_FOLLOWING_PERIOD_MS);

    return (h_barcode_cyclic < E_OK) ? h_barcode_cyclic : h_line_cyclic;
}

/*** end of file ***/

/** @file barcode_decoder.c
 *
 * @brief Barcode Decoder API and task, the only task that drives the ADC.
 */

#include "barcode_decoder.h"
#include "drv_barcode.h"
#include "subsystem3.h"

static ID g_mutex;
static barcode_decoder_raw_t g_raw;

/*!
 * @brief Create the lock and start sampling the sensor.
 *
 * @return True if the lock was created.
 */
bool barcode_decoder_init(void)
{
    T_CMTX mutex = {0};

    mutex.mtxatr = TA_INHERIT;
    g_mutex = tk_cre_mtx(&mutex);
    drv_barcode_init();
    return (g_mutex > 0);
}

/*!
 * @brief Copy the latest raw level.
 */
void barcode_decoder_get_raw(barcode_decoder_raw_t * p_raw)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    *p_raw = g_raw;
    (void)tk_unl_mtx(g_mutex);
}

/*!
 * @brief Barcode Decoder task: every 10 ms, take the new samples and
 * publish their range.
 */
void barcode_decoder_task(INT stacd, void * p_exinf)
{
    static uint16_t samples[DRV_BARCODE_RING]; /* Off the task stack. */
    uint32_t count = 0u;
    uint32_t min = 0u;
    uint32_t max = 0u;
    uint32_t i = 0u;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* Woken every 10 ms (subsystem3.c). */
        count = drv_barcode_read(samples);
        min = 4095u;
        max = 0u;
        for (i = 0u; i < count; i++)
        {
            min = (samples[i] < min) ? samples[i] : min;
            max = (samples[i] > max) ? samples[i] : max;
        }

        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        g_raw.min = min;
        g_raw.max = max;
        g_raw.samples += count;
        (void)tk_unl_mtx(g_mutex);
    }
}

/*** end of file ***/

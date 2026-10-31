/** @file barcode_decoder.c
 *
 * @brief Barcode Decoder API and task, the only task that drives the ADC.
 *
 * Each sample is placed along the track by interpolating the odometer
 * across its 10 ms block, in 1/16 mm. A barcode starts at the first black
 * after white and ends at a white run longer than BARCODE_END; its bars
 * and spaces then go to code39_decode().
 */

#include "barcode_decoder.h"
#include "code39.h"
#include "drv_barcode.h"
#include "subsystem3.h"
#include "../subsystem2/motion_control.h"

#define BARCODE_BLACK    2500u /* ADC counts: white reads ~170, black */
#define BARCODE_WHITE    1500u /* ~4070 (S3-U10). Hysteresis between. */
#define BARCODE_SUB      16u   /* Positions in 1/16 mm. */
#define BARCODE_END      (30u * BARCODE_SUB) /* 30 mm: over 3 wide bars. */
#define BARCODE_ODOMETER 0xFFFFFFu /* motion distance_mm wraps at 24 bits. */
#define CODE39_CHAR      9u        /* Elements in one character. */

static ID g_mutex;
static barcode_decoder_t g_barcode;
static barcode_decoder_raw_t g_raw;

/* Barcode Decoder task only. */
static uint16_t g_widths[CODE39_ELEMENTS];
static uint32_t g_elements;  /* Elements so far; over 29 is too many. */
static uint32_t g_run_start; /* 1/16 mm. */
static bool g_b_black;
static bool g_b_in_code;

static void barcode_decoder_sample(uint32_t level, uint32_t pos,
                                   uint32_t odometer_mm);
static void barcode_decoder_end(uint32_t odometer_mm);

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
 * @brief Copy the last validated barcode.
 */
void barcode_decoder_get(barcode_decoder_t * p_barcode)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    *p_barcode = g_barcode;
    (void)tk_unl_mtx(g_mutex);
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
 * @brief Barcode Decoder task: every 10 ms, place the new samples along
 * the track and run them through the barcode reader.
 */
void barcode_decoder_task(INT stacd, void * p_exinf)
{
    static uint16_t samples[DRV_BARCODE_RING]; /* Off the task stack. */
    motion_control_status_t motion = {0};
    uint32_t last_mm = 0u;
    uint32_t moved = 0u; /* 1/16 mm rolled in this block. */
    uint32_t pos = 0u;   /* 1/16 mm, wraps. */
    uint32_t count = 0u;
    uint32_t min = 0u;
    uint32_t max = 0u;
    uint32_t i = 0u;

    (void)stacd;
    (void)p_exinf;
    motion_control_get_status(&motion);
    last_mm = motion.distance_mm;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* Woken every 10 ms (subsystem3.c). */
        count = drv_barcode_read(samples);
        motion_control_get_status(&motion);
        moved =
            ((motion.distance_mm - last_mm) & BARCODE_ODOMETER) * BARCODE_SUB;
        last_mm = motion.distance_mm;
        min = 4095u;
        max = 0u;
        for (i = 0u; i < count; i++)
        {
            min = (samples[i] < min) ? samples[i] : min;
            max = (samples[i] > max) ? samples[i] : max;
            barcode_decoder_sample(samples[i],
                                   pos + ((moved * (i + 1u)) / count),
                                   motion.distance_mm);
        }
        pos += moved;

        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        g_raw.min = min;
        g_raw.max = max;
        g_raw.samples += count;
        (void)tk_unl_mtx(g_mutex);
    }
}

/*!
 * @brief Feed one sample: close a bar or space when the level changes,
 * and end the barcode after a long white run.
 *
 * @param[in] level       ADC counts.
 * @param[in] pos         Where the sample was taken, 1/16 mm.
 * @param[in] odometer_mm Odometer at the end of this block.
 */
static void barcode_decoder_sample(uint32_t level, uint32_t pos,
                                   uint32_t odometer_mm)
{
    bool b_black =
        g_b_black ? (level > BARCODE_WHITE) : (level > BARCODE_BLACK);
    uint32_t width = pos - g_run_start;

    if (b_black != g_b_black)
    {
        if (g_b_black && !g_b_in_code)
        {
            g_b_in_code = true; /* The first bar has ended. */
            g_elements = 0u;
        }
        if (g_b_in_code && (width > BARCODE_END))
        {
            g_b_in_code = false; /* A bar this long is not a barcode. */
        }
        if (g_b_in_code)
        {
            if (g_elements < CODE39_ELEMENTS)
            {
                g_widths[g_elements] = (uint16_t)width;
            }
            g_elements++;
        }
        g_run_start = pos;
        g_b_black = b_black;
    }
    else if (g_b_in_code && !g_b_black && (width > BARCODE_END))
    {
        barcode_decoder_end(odometer_mm);
        g_b_in_code = false;
    }
    else
    {
        /* Inside a bar or space, or white between barcodes. */
    }
}

/*!
 * @brief Decode the bars just read and publish a valid command.
 */
static void barcode_decoder_end(uint32_t odometer_mm)
{
    uint32_t letter =
        (CODE39_ELEMENTS == g_elements) ? code39_decode(g_widths) : 0u;
#if APP_TEST
    uint32_t i = 0u;

    /* Bench evidence for S3-U14: what the sensor measured, in 0.1 mm. */
    if (g_elements >= CODE39_CHAR)
    {
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[barcode] elements=%u letter=%c widths_0.1mm=",
                  g_elements, (0u != letter) ? (int)letter : '-');
        for (i = 0u; (i < g_elements) && (i < CODE39_ELEMENTS); i++)
        {
            tm_printf((UB *)" %u", (g_widths[i] * 10u) / BARCODE_SUB);
        }
        tm_printf((UB *)"\n");
    }
#endif

    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    if (0u != letter)
    {
        g_barcode.command = BARCODE_DECODER_LEFT + (letter - 'A');
        g_barcode.count++;
        g_barcode.distance_mm = odometer_mm;
    }
    else if (g_elements >= CODE39_CHAR)
    {
        g_barcode.rejects++; /* Shorter runs are specks, or the line. */
    }
    else
    {
        /* Ignored. */
    }
    (void)tk_unl_mtx(g_mutex);
}

/*** end of file ***/

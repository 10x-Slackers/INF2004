/** @file barcode_decoder.c
 *
 * @brief Barcode Decoder API and task, the only task that drives the ADC.
 *
 * Each sample is placed along the track by interpolating the distance
 * rolled across its 10 ms block, in um. A barcode starts at the first black
 * after white and ends at a white run longer than BARCODE_END; its bars
 * and spaces then go to code39_decode().
 */

#include "barcode_decoder.h"
#include "code39.h"
#include "drv_barcode.h"
#include "subsystem3.h"
#include "../subsystem2/motion_control.h"

/* ADC counts: white reads ~170, black ~4080 (S3-U10). Low, so narrow bars
 * count (S3-U14), with hysteresis between. */
#define BARCODE_BLACK 1000u
#define BARCODE_WHITE 600u
#define BARCODE_END   30000u /* um: 30 mm, over 3 wide bars. */

/* The course's barcodes: change these lines when the letters are announced
 * (any number of letters per command). Other valid letters are rejected. Z is
 * the barcode between the loops on the example course. */
static const struct
{
    char letter;
    uint8_t command; /* BARCODE_DECODER_... */
} g_actions[] = {
    {'A', BARCODE_DECODER_LEFT},     {'B', BARCODE_DECODER_RIGHT},
    {'C', BARCODE_DECODER_STRAIGHT}, {'D', BARCODE_DECODER_UTURN},
    {'Z', BARCODE_DECODER_LEFT},
};

static ID gh_mutex;
static barcode_decoder_t g_barcode;
static barcode_decoder_raw_t g_raw;

/* Barcode Decoder task only. */
static uint16_t g_widths[CODE39_ELEMENTS];
static uint32_t g_elements;  /* Elements so far; widths kept for 29. */
static uint32_t g_run_start; /* um. */
static bool gb_black;
static bool gb_in_code;

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
    gh_mutex = tk_cre_mtx(&mutex);
    drv_barcode_init();
    return (gh_mutex > 0);
}

/*!
 * @brief Copy the last validated barcode.
 */
void barcode_decoder_get(barcode_decoder_t * p_barcode)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    *p_barcode = g_barcode;
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Copy the latest raw level.
 */
void barcode_decoder_get_raw(barcode_decoder_raw_t * p_raw)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    *p_raw = g_raw;
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Barcode Decoder task: every 10 ms, place the new samples along
 * the track and run them through the barcode reader.
 */
void barcode_decoder_task(INT stacd, void * p_exinf)
{
    static uint16_t samples[DRV_BARCODE_RING]; /* Off the task stack. */
    motion_control_status_t motion = {0};
    uint32_t last_um = motion_control_distance_um();
    uint32_t moved = 0u; /* um rolled in this block. */
    uint32_t count = 0u;
    uint32_t min = 0u;
    uint32_t max = 0u;
    uint32_t sample = 0u;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* Woken every 10 ms (subsystem3.c). */
        count = drv_barcode_read(samples);
        /* Read right after the samples, so both end at the same moment. */
        moved = motion_control_distance_um() - last_um;
        motion_control_get_status(&motion);
        min = 4095u;
        max = 0u;
        for (sample = 0u; sample < count; sample++)
        {
            min = (samples[sample] < min) ? samples[sample] : min;
            max = (samples[sample] > max) ? samples[sample] : max;
            barcode_decoder_sample(samples[sample],
                                   last_um + ((moved * (sample + 1u)) / count),
                                   motion.distance_mm);
        }
        last_um += moved;

        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_raw.min = min;
        g_raw.max = max;
        g_raw.samples += count;
        (void)tk_unl_mtx(gh_mutex);
    }
}

/*!
 * @brief Feed one sample: close a bar or space when the level changes,
 * and end the barcode after a long white run.
 *
 * @param[in] level       ADC counts.
 * @param[in] pos         Where the sample was taken, um, wraps.
 * @param[in] odometer_mm Odometer at the end of this block.
 */
static void barcode_decoder_sample(uint32_t level, uint32_t pos,
                                   uint32_t odometer_mm)
{
    bool const b_black =
        gb_black ? (level > BARCODE_WHITE) : (level > BARCODE_BLACK);
    uint32_t const width = pos - g_run_start;

    if (b_black != gb_black)
    {
        if (gb_black && (!gb_in_code))
        {
            gb_in_code = true; /* The first bar has ended. */
            g_elements = 0u;
        }
        if (gb_in_code && (width > BARCODE_END))
        {
            gb_in_code = false; /* A bar this long is not a barcode. */
        }
        if (gb_in_code && (g_elements < CODE39_ELEMENTS))
        {
            /* In a barcode, width is at most BARCODE_END: it fits. */
            g_widths[g_elements] = (uint16_t)width;
        }
        g_elements++; /* Reset when a barcode starts. */
        g_run_start = pos;
        gb_black = b_black;
    }
    else if (gb_in_code && (!gb_black) && (width > BARCODE_END))
    {
        barcode_decoder_end(odometer_mm);
        gb_in_code = false;
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
    /* Extra elements after a barcode (a crossing line, the edge of the
     * print) are ignored: only the first 29 are kept. */
    uint32_t const letter =
        (g_elements >= CODE39_ELEMENTS) ? code39_decode(g_widths) : 0u;
    uint32_t command = BARCODE_DECODER_NONE;
    uint32_t action = 0u;
#if APP_TEST
    uint32_t element = 0u;

    /* Bench evidence for S3-U14: what the sensor measured, in 0.1 mm. */
    if (g_elements >= CODE39_PER_CHR)
    {
        /* (UB *): tm_printf() takes unsigned chars. (int): letter is 0
         * or a Code 39 character. */
        tm_printf((UB *)"[barcode] elements=%u letter=%c widths_0.1mm=",
                  g_elements, (0u != letter) ? (int)letter : '-');
        for (element = 0u;
             (element < g_elements) && (element < CODE39_ELEMENTS); element++)
        {
            tm_printf((UB *)" %u", g_widths[element] / 100u);
        }
        tm_printf((UB *)"\n");
    }
#endif

    for (action = 0u; action < (sizeof(g_actions) / sizeof(g_actions[0]));
         action++)
    {
        command = ((uint32_t)g_actions[action].letter == letter)
                      ? g_actions[action].command
                      : command;
    }

    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    if (BARCODE_DECODER_NONE != command)
    {
        g_barcode.command = command;
        g_barcode.count++;
        g_barcode.distance_mm = odometer_mm;
    }
    else if (g_elements >= CODE39_PER_CHR)
    {
        g_barcode.rejects++; /* Shorter runs are specks, or the line. */
    }
    else
    {
        /* Ignored. */
    }
    (void)tk_unl_mtx(gh_mutex);
}

/*** end of file ***/

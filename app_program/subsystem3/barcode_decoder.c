/** @file barcode_decoder.c
 *
 * @brief Barcode Decoder API and task, the only task that drives the ADC.
 *
 * Each sample is placed along the track by interpolating the distance
 * rolled across its 10 ms block, in um. A barcode starts at the first black
 * after white and ends at a white run longer than BARCODE_END; its bars
 * and spaces then go to code39_decode(), which guesses at a misread.
 *
 * If the ADC or DMA stops, no barcode is ever seen, so the task counts the
 * samples of every BARCODE_CHECK_WAKES wake-ups: APP_FAULT_BARCODE_SENSOR is
 * set while they are under BARCODE_CHECK_MIN.
 */

#include "barcode_decoder.h"
#include "code39.h"
#include "drv_barcode.h"
#include "subsystem3.h"
#include "../subsystem2/motion_control.h"

/* ADC counts: white reads ~170, black ~4080 (S3-U10). Low, so narrow bars
 * count (S3-U14), with hysteresis between. */
#define BARCODE_BLACK       600u
#define BARCODE_WHITE       400u
#define BARCODE_END         30000u /* um: 30 mm, over 3 wide bars. */
#define BARCODE_CHECK_WAKES 10u    /* 100 ms: 200 samples are due. */
#define BARCODE_CHECK_MIN   100u   /* Fewer samples in that time: a fault. */
/* Elements after the first bar before b_reading: the third bar has begun,
 * each gap under BARCODE_END, which a lone line or speck does not give. */
#define BARCODE_READING 3u
/* um moved since the last read, an unsigned difference: from this up it
 * wrapped, so the distance went back. */
#define BARCODE_BACK 0x80000000u

/* The command of each letter, A to Z, all NONE (ignored) until the
 * supervisor sends a map (barcode_decoder_set_actions()). */
static uint8_t g_actions[BARCODE_DECODER_LETTERS] = {0};

static ID gh_mutex = 0;
static barcode_decoder_t g_barcode = {0};
static barcode_decoder_raw_t g_raw = {0};

/* Barcode Decoder task only: the barcode being read, and the ADC check. */
static uint16_t g_widths[CODE39_KEPT] = {0};
static uint32_t g_run_start = 0u; /* um, as motion_control_distance_um(). */
static struct
{
    unsigned int elements : 5; /* Widths kept, 0..CODE39_KEPT (31). */
    bool b_black : 1;
    bool b_in_code : 1;
} g_code = {0};
static struct
{
    unsigned int wakes : 4;    /* Wake-ups so far, 0..BARCODE_CHECK_WAKES. */
    unsigned int samples : 12; /* Samples in them, up to 10 rings (2550). */
} g_check = {0};
#if BARCODE_CHECK_WAKES > 15u /* 15 reads of under a ring also fit samples. */
#error "g_check.wakes is too narrow"
#endif

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
 * @brief Replace the command of every letter, for barcodes read from now.
 * A value that is not a command (over UTURN) ignores the letter.
 *
 * @param[in] p_actions BARCODE_DECODER_LETTERS commands, A first.
 */
void barcode_decoder_set_actions(const uint8_t * p_actions)
{
    uint32_t letter = 0u;

    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    for (letter = 0u; letter < BARCODE_DECODER_LETTERS; letter++)
    {
        g_actions[letter] = (p_actions[letter] <= BARCODE_DECODER_UTURN)
                                ? p_actions[letter]
                                : BARCODE_DECODER_NONE;
    }
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
        if (moved >= BARCODE_BACK)
        {
            /* A wheel rolled back or reversed, as when the car stops: the
             * distance can dip between two motion steps. Stay here until
             * the car has rolled past this point again. */
            moved = 0u;
        }
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
        g_check.samples += count; /* 10 reads of under a ring each: fits. */
        g_check.wakes++;
        if (g_check.wakes >= BARCODE_CHECK_WAKES)
        {
            if (g_check.samples < BARCODE_CHECK_MIN)
            {
                app_fault_set(APP_FAULT_BARCODE_SENSOR);
            }
            else
            {
                app_fault_clear(APP_FAULT_BARCODE_SENSOR);
            }
            g_check.samples = 0u;
            g_check.wakes = 0u;
        }

        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_raw.min = min;
        g_raw.max = max;
        g_raw.samples += count;
        g_barcode.b_reading =
            (g_code.b_in_code) && (g_code.elements >= BARCODE_READING);
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
        g_code.b_black ? (level > BARCODE_WHITE) : (level > BARCODE_BLACK);
    uint32_t const width = pos - g_run_start;

    if (b_black != g_code.b_black)
    {
        if ((g_code.b_black) && (!g_code.b_in_code))
        {
            g_code.b_in_code = true; /* The first bar has ended. */
            g_code.elements = 0u;
        }
        if ((g_code.b_in_code) && (width > BARCODE_END))
        {
            g_code.b_in_code = false; /* A bar this long is not a barcode. */
        }
        if ((g_code.b_in_code) && (g_code.elements < CODE39_KEPT))
        {
            /* In a barcode, width is at most BARCODE_END: it fits. */
            g_widths[g_code.elements] = (uint16_t)width;
            g_code.elements++; /* Reset when a barcode starts. */
        }
        g_run_start = pos;
        g_code.b_black = b_black;
    }
    else if ((g_code.b_in_code) && (!g_code.b_black) && (width > BARCODE_END))
    {
        barcode_decoder_end(odometer_mm);
        g_code.b_in_code = false;
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
     * print) are ignored: only the first 31 are kept. */
    uint32_t const letter = code39_decode(g_widths, g_code.elements);
    /* 1..26 for A..Z; 0 for a misread or a digit, which have no action.
     * (uint32_t): a character constant is a small positive number. */
    uint32_t const index =
        ((letter >= (uint32_t)'A') && (letter <= (uint32_t)'Z'))
            ? ((letter - (uint32_t)'A') + 1u)
            : 0u;
    uint32_t command = BARCODE_DECODER_NONE;
#if APP_TEST
    uint32_t element = 0u;

    /* Bench evidence for S3-U14: what the sensor measured, in 0.1 mm. */
    if (g_code.elements >= CODE39_PER_CHR)
    {
        /* (int): letter is 0 or a Code 39 character. */
        APP_PRINTF("[barcode] elements=%u letter=%c widths_0.1mm=",
                   g_code.elements, (0u != letter) ? (int)letter : '-');
        for (element = 0u; element < g_code.elements; element++)
        {
            APP_PRINTF(" %u", g_widths[element] / 100u);
        }
        APP_PRINTF("\n");
    }
#endif

    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    if (0u != index)
    {
        command = g_actions[index - 1u];
    }
    if (BARCODE_DECODER_NONE != command)
    {
        g_barcode.command = command;
        g_barcode.letter = index;
        g_barcode.count++;
        g_barcode.distance_mm = odometer_mm;
    }
    else if (g_code.elements >= CODE39_PER_CHR)
    {
        g_barcode.letter = index;
        g_barcode.rejects++; /* Shorter runs are specks, or the line. */
    }
    else
    {
        /* Ignored. */
    }
    (void)tk_unl_mtx(gh_mutex);
    if (g_code.elements >= CODE39_PER_CHR)
    {
        /* Once per barcode read. (int): letter is 0 or a Code 39
         * character. */
        APP_PRINTF("[barcode] letter=%c command=%u\n",
                   (0u != letter) ? (int)letter : '-', command);
    }
}

/*** end of file ***/

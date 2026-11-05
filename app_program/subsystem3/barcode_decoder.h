/** @file barcode_decoder.h
 *
 * @brief Barcode Decoder API (interface I2). The Barcode Decoder task reads
 * the barcode sensor every 10 ms, measures each bar against the distance
 * the car has rolled (I5), decodes Code 39 and publishes the command of
 * each letter listed in g_actions (barcode_decoder.c).
 *
 * Poll barcode_decoder_get() and act when count changes. A barcode that
 * is read but not valid (a letter not in g_actions, a misread) only bumps
 * rejects; an unvalidated command is never published.
 */

#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H

#include <stdbool.h>
#include <stdint.h>

/* barcode_decoder_t.command */
#define BARCODE_DECODER_NONE     0u
#define BARCODE_DECODER_LEFT     1u
#define BARCODE_DECODER_RIGHT    2u
#define BARCODE_DECODER_STRAIGHT 3u
#define BARCODE_DECODER_UTURN    4u

/* Last validated barcode. */
typedef struct
{
    unsigned int command : 3;      /* BARCODE_DECODER_..., NONE until one. */
    unsigned int count : 4;        /* Barcodes validated, wraps. */
    unsigned int rejects : 4;      /* Barcodes read but rejected, wraps. */
    unsigned int distance_mm : 24; /* Odometer at its end (I5), wraps. */
} barcode_decoder_t;

/* Raw sensor level over the last 10 ms, for calibration. */
typedef struct
{
    unsigned int min : 12; /* ADC counts, 0..4095. */
    unsigned int max : 12;
    unsigned int samples : 16; /* Samples taken, wraps. */
} barcode_decoder_raw_t;

bool barcode_decoder_init(void);
void barcode_decoder_get(barcode_decoder_t * p_barcode);
void barcode_decoder_get_raw(barcode_decoder_raw_t * p_raw);

#endif /* BARCODE_DECODER_H */

/*** end of file ***/

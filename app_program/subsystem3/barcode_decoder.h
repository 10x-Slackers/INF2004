/** @file barcode_decoder.h
 *
 * @brief Barcode Decoder API (interface I2). The Barcode Decoder task reads
 * the barcode sensor every 10 ms, measures each bar against the distance
 * the car has rolled (I5), decodes Code 39 and publishes the command
 * g_actions (barcode_decoder.c) gives its letter. The supervisor can replace
 * g_actions over MQTT (docs/TELEMETRY.md).
 *
 * Poll barcode_decoder_get() and act when count changes. A barcode that
 * is read but not valid (a letter set to NONE, a digit, a misread) only bumps
 * rejects; an unvalidated command is never published. b_reading is set
 * while the sensor is part way through what looks like a barcode.
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

#define BARCODE_DECODER_LETTERS 26u /* g_actions: A to Z. */

/* Last validated barcode. */
typedef struct
{
    unsigned int command : 3;      /* BARCODE_DECODER_..., NONE until one. */
    unsigned int count : 4;        /* Barcodes validated, wraps. */
    unsigned int rejects : 4;      /* Barcodes read but rejected, wraps. */
    unsigned int letter : 5;       /* Last read or rejected: 1..26 A..Z, 0
                                    * a misread or a digit. */
    unsigned int distance_mm : 24; /* Odometer at its end (I5), wraps. */
    bool b_reading : 1;            /* Over a barcode now. */
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
void barcode_decoder_set_actions(const uint8_t * p_actions);

#endif /* BARCODE_DECODER_H */

/*** end of file ***/

/** @file barcode_decoder.h
 *
 * @brief Barcode Decoder API. The Barcode Decoder task reads the barcode
 * sensor every 10 ms. For now it publishes the raw level, for calibration;
 * decoding comes next.
 */

#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H

#include <stdbool.h>
#include <stdint.h>

/* Raw sensor level over the last 10 ms. */
typedef struct
{
    unsigned int min : 12; /* ADC counts, 0..4095. */
    unsigned int max : 12;
    unsigned int samples : 16; /* Samples taken, wraps. */
} barcode_decoder_raw_t;

bool barcode_decoder_init(void);
void barcode_decoder_get_raw(barcode_decoder_raw_t * p_raw);

#endif /* BARCODE_DECODER_H */

/*** end of file ***/

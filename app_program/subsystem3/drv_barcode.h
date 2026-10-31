/** @file drv_barcode.h
 *
 * @brief Barcode IR module AO on GP26 (ADC0), sampled at 2 ksample/s by
 * the ADC and moved by DMA channel 11 into a ring buffer. Call only from
 * the Barcode Decoder task, which owns the ADC.
 */

#ifndef DRV_BARCODE_H
#define DRV_BARCODE_H

#include <stdint.h>

#define DRV_BARCODE_RING 256u /* Samples: 128 ms of history. */

void drv_barcode_init(void);
uint32_t drv_barcode_read(uint16_t * p_samples);

#endif /* DRV_BARCODE_H */

/*** end of file ***/

/** @file code39.h
 *
 * @brief Code 39 decoding of one three-character symbol, *X*, from its
 * element widths. Pure functions, no hardware.
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>

/* Elements of *X*: three characters of 5 bars and 4 spaces, and the two
 * narrow spaces between them. */
#define CODE39_ELEMENTS 29u
#define CODE39_PER_CHR  9u /* Elements in one character. */

uint32_t code39_decode(const uint16_t * p_widths);

#endif /* CODE39_H */

/*** end of file ***/

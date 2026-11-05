/** @file code39.c
 *
 * @brief Code 39 decoding of one three-character symbol, *X*. Each
 * character is 9 elements, bar first, of which exactly 3 are wide (about
 * 3 x narrow). All 43 characters and '*' are known; the patterns were
 * checked against the Code 39 table on Wikipedia.
 */

#include "code39.h"
#include <stdbool.h>

#define CODE39_STRIDE 10u /* 9 elements and the gap after them. */
#define CODE39_WIDE   3u

/* Patterns, first element in the most significant of 9 bits, 1 = wide,
 * in the order of g_letters. */
static const uint16_t g_patterns[] = {
    0x034u, 0x121u, 0x061u, 0x160u, 0x031u, 0x130u, 0x070u, 0x025u, 0x124u,
    0x064u, 0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00Du, 0x10Cu,
    0x04Cu, 0x01Cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u,
    0x106u, 0x046u, 0x016u, 0x181u, 0x0C1u, 0x1C0u, 0x091u, 0x190u, 0x0D0u,
    0x085u, 0x184u, 0x0C4u, 0x0A8u, 0x0A2u, 0x08Au, 0x02Au, 0x094u};
static const char g_letters[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";

static uint32_t code39_one_way(const uint16_t * p_widths,
                               const uint32_t * p_threshold);
static uint32_t code39_char(const uint16_t * p_widths,
                            const uint32_t * p_threshold);

/*!
 * @brief Decode *X*, read in either direction. Bars and spaces are split
 * into narrow and wide separately, halfway between the narrowest and widest
 * of each: the sensor's spot makes bars read narrower and spaces wider.
 *
 * @param[in] p_widths CODE39_ELEMENTS widths in any one unit, in the order
 *                     the sensor saw them, first bar first.
 * @return The character X, or 0 if the widths are not a valid *X*.
 */
uint32_t code39_decode(const uint16_t * p_widths)
{
    uint16_t reversed[CODE39_ELEMENTS] = {0};
    uint32_t min[2] = {UINT16_MAX, UINT16_MAX}; /* Bars [0], spaces [1]. */
    uint32_t max[2] = {0u, 0u};
    uint32_t threshold[2] = {0u, 0u};
    uint32_t letter = 0u;
    uint32_t element = 0u;
    uint32_t kind = 0u;

    /* Elements alternate from a bar, and 29 is odd, so reversing keeps
     * every element's kind. */
    for (element = 0u; element < CODE39_ELEMENTS; element++)
    {
        kind = element % 2u;
        min[kind] =
            (p_widths[element] < min[kind]) ? p_widths[element] : min[kind];
        max[kind] =
            (p_widths[element] > max[kind]) ? p_widths[element] : max[kind];
        reversed[CODE39_ELEMENTS - 1u - element] = p_widths[element];
    }
    threshold[0] = (min[0] + max[0]) / 2u;
    threshold[1] = (min[1] + max[1]) / 2u;
    letter = code39_one_way(p_widths, threshold);
    if (0u == letter)
    {
        letter = code39_one_way(reversed, threshold);
    }
    return letter;
}

/*!
 * @brief Decode *X* read forwards.
 *
 * @return The character X, or 0.
 */
static uint32_t code39_one_way(const uint16_t * p_widths,
                               const uint32_t * p_threshold)
{
    uint32_t const start = code39_char(&p_widths[0], p_threshold);
    uint32_t const letter = code39_char(&p_widths[CODE39_STRIDE], p_threshold);
    uint32_t const stop =
        code39_char(&p_widths[2u * CODE39_STRIDE], p_threshold);
    bool const b_gaps =
        (p_widths[CODE39_PER_CHR] <= p_threshold[1]) &&
        (p_widths[CODE39_STRIDE + CODE39_PER_CHR] <= p_threshold[1]);

    return (b_gaps && ('*' == start) && ('*' == stop) && ('*' != letter))
               ? letter
               : 0u;
}

/*!
 * @brief Decode one character.
 *
 * @return The character, or 0 if unknown or not exactly 3 wide.
 */
static uint32_t code39_char(const uint16_t * p_widths,
                            const uint32_t * p_threshold)
{
    uint32_t pattern = 0u;
    uint32_t wide = 0u;
    uint32_t letter = 0u;
    uint32_t element = 0u;
    uint32_t known = 0u; /* Index into g_patterns and g_letters. */

    for (element = 0u; element < CODE39_PER_CHR; element++)
    {
        pattern <<= 1;
        if (p_widths[element] > p_threshold[element % 2u]) /* Bar first. */
        {
            pattern |= 1u;
            wide++;
        }
    }
    for (known = 0u; (CODE39_WIDE == wide) &&
                     (known < (sizeof(g_patterns) / sizeof(g_patterns[0])));
         known++)
    {
        letter =
            (g_patterns[known] == pattern) ? (uint8_t)g_letters[known] : letter;
    }
    return letter;
}

/*** end of file ***/

/** @file drv_line.c
 *
 * @brief Left and right line IR modules. Each module's LM393 compares the
 * reflected IR against its potentiometer and drives DO; set the
 * potentiometers at the final mounting height, over the real track.
 */

#include "drv_line.h"
#include "../app.h"

#define DRV_LINE_LEFT_PIN  6u
#define DRV_LINE_RIGHT_PIN 7u
#define DRV_LINE_BLACK     1u /* DO level over black: confirm on the track. */

/*!
 * @brief Set both DO pins as inputs.
 */
void drv_line_init(void)
{
    (void)gpio_set_pin(DRV_LINE_LEFT_PIN, GPIO_MODE_IN);
    (void)gpio_set_pin(DRV_LINE_RIGHT_PIN, GPIO_MODE_IN);
}

/*!
 * @brief Read both sensors.
 *
 * @return DRV_LINE_LEFT and DRV_LINE_RIGHT bits, set over black.
 */
uint32_t drv_line_read(void)
{
    uint32_t bits = 0u;

    if (DRV_LINE_BLACK == gpio_get_val(DRV_LINE_LEFT_PIN))
    {
        bits |= DRV_LINE_LEFT;
    }
    if (DRV_LINE_BLACK == gpio_get_val(DRV_LINE_RIGHT_PIN))
    {
        bits |= DRV_LINE_RIGHT;
    }
    return bits;
}

/*** end of file ***/

/** @file libbsp.h
 *
 * @brief Host stand-in for the board helpers: the PWM calls the buzzer uses.
 */

#ifndef LIBBSP_H
#define LIBBSP_H

#include "../tk/tkernel.h"

ER pwm_set_pin(UW no);
ER pwm_set_wrap(UW no, UW wrap);
ER pwm_set_cc(UW no, UW cc);
ER pwm_set_enabled(UW no, BOOL enable);

#endif /* LIBBSP_H */

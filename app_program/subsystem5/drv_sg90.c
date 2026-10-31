/* GP12 = slice 6 channel A. Never configure slices 4/5 (motors). */
#include "drv_sg90.h"
#include <sys/sysdef.h>

ER s5_sg90_angle(unsigned int angle)
{
    unsigned int pulse;
    if (angle < S5_SERVO_MIN_DEG || angle > S5_SERVO_MAX_DEG)
    {
        return E_PAR;
    }
    if (angle <= S5_SERVO_CENTRE_DEG)
    {
        pulse = S5_SERVO_MIN_US + (angle - S5_SERVO_MIN_DEG) *
                                      (S5_SERVO_CENTRE_US - S5_SERVO_MIN_US) /
                                      (S5_SERVO_CENTRE_DEG - S5_SERVO_MIN_DEG);
    }
    else
    {
        pulse =
            S5_SERVO_CENTRE_US + (angle - S5_SERVO_CENTRE_DEG) *
                                     (S5_SERVO_MAX_US - S5_SERVO_CENTRE_US) /
                                     (S5_SERVO_MAX_DEG - S5_SERVO_CENTRE_DEG);
    }
    return pwm_set_cc(S5_SERVO_PIN, pulse);
}

ER s5_sg90_init(void)
{
    uint32_t start = s5_now_ms();
    /* Atomic CLEAR alias: release only PWM, without resetting active slices.
     * Do not enable USE_PTMR (kernel physical timers also consume PWM). */
    out_w(RESETS_RESET + 0x3000U, RESETS_RESET_PWM);
    while (!(in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        if (s5_now_ms() - start > 100U)
        {
            return E_TMOUT;
        }
    }
    out_w(PWM_CH6_BASE + PWM_CHx_CSR, 0);
    /* 125 MHz / 125 = 1 MHz; 20000 counter ticks -> 50 Hz. 8.4 divider. */
    out_w(PWM_CH6_BASE + PWM_CHx_DIV, SYSCLK << 4);
    out_w(PWM_CH6_BASE + PWM_CHx_CTR, 0);
    pwm_set_wrap(S5_SERVO_PIN, 19999);
    s5_sg90_angle(S5_SERVO_CENTRE_DEG);
    pwm_set_pin(S5_SERVO_PIN);
    return pwm_set_enabled(S5_SERVO_PIN, TRUE);
}

/* Called only by the scan task. Timer polling is bounded, not IRQ-masked. */
#include "drv_hcsr04.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>

/* usermain() starts the 1 MHz TIMER tick, so every wait below ends. */
#define S5_TRIGGER_US 10U

ER s5_hcsr04_init(void)
{
#if TM_CONSOLE_UART
    return E_NOSPT; /* GP0/GP1 belong to the UART in this build. */
#else
    gpio_set_val(S5_TRIG_PIN, 0);
    gpio_set_pin(S5_TRIG_PIN, GPIO_MODE_OUT);
    return gpio_set_pin(S5_ECHO_PIN, GPIO_MODE_IN);
#endif
}

/* Wait while ECHO is at level; nonzero if it is still there after 25 ms. */
static unsigned int echo_timeout(unsigned int level, uint32_t * start)
{
    *start = in_w(TIMER_TIMERAWL);
    while (gpio_get_val(S5_ECHO_PIN) == level)
    {
        if ((uint32_t)(in_w(TIMER_TIMERAWL) - *start) >= S5_ECHO_TIMEOUT_US)
        {
            return 1;
        }
    }
    return 0;
}

S5_SAMPLE s5_hcsr04_read(unsigned int angle, unsigned int * faults)
{
    static uint32_t previous_us;
    uint32_t start;
    S5_SAMPLE sample = {0};
    sample.angle_deg = angle;
    /* At least 60 ms after the previous trigger, including first boot. */
    while ((uint32_t)(in_w(TIMER_TIMERAWL) - previous_us) <
           S5_MEASURE_INTERVAL_US)
    {
        tk_dly_tsk(10);
    }
    if (gpio_get_val(S5_ECHO_PIN))
    {
        *faults |= S5_ECHO_TIMEOUT;
        sample.stamp_ms = s5_now_ms();
        return sample;
    }
    /* Not WaitUsec: it uses the per-core SysTick, which is off on core 1. */
    gpio_set_val(S5_TRIG_PIN, 1);
    start = in_w(TIMER_TIMERAWL);
    while ((uint32_t)(in_w(TIMER_TIMERAWL) - start) < S5_TRIGGER_US)
    {
    }
    gpio_set_val(S5_TRIG_PIN, 0);
    previous_us = in_w(TIMER_TIMERAWL);
    unsigned int fault = echo_timeout(0, &start)   ? S5_NO_ECHO
                         : echo_timeout(1, &start) ? S5_ECHO_TIMEOUT
                                                   : 0;
    if (fault)
    {
        *faults |= fault;
        sample.stamp_ms = s5_now_ms();
        return sample;
    }
    return s5_echo_sample((uint32_t)(in_w(TIMER_TIMERAWL) - start), angle,
                          s5_now_ms());
}

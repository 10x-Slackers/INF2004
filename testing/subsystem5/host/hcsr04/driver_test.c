/* Actual HC-SR04 driver with simulated registers and time. */
#include "drv_hcsr04.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <assert.h>
#include "test_report.h"

static uint64_t cpu_us, deadline_us, trigger_high_at, trigger_width_us;
static uint32_t pulse_at;
static unsigned int echo_mode, gpio_calls;

enum
{
    NORMAL_ECHO,
    NO_ECHO,
    HIGH_ECHO
};
static void advance(unsigned int us)
{
    cpu_us += us;
    assert(cpu_us <= deadline_us); /* Catch an unbounded driver wait. */
}
uint32_t s5_now_ms(void)
{
    return (uint32_t)(cpu_us / 1000U);
}
void tk_dly_tsk(unsigned int ms)
{
    advance(ms * 1000U);
}
void WaitUsec(unsigned int us)
{
    (void)us;
    assert(0 && "WaitUsec would stall with core-1 SysTick disabled");
}
uint32_t in_w(uint32_t address)
{
    advance(1);
    assert(address == TIMER_TIMERAWL);
    return (uint32_t)cpu_us;
}
int gpio_set_pin(unsigned int pin, unsigned int mode)
{
    assert((pin == 0 && mode == GPIO_MODE_OUT) ||
           (pin == 1 && mode == GPIO_MODE_IN));
    gpio_calls++;
    return E_OK;
}
int gpio_set_val(unsigned int pin, unsigned int value)
{
    assert(pin == 0);
    if (value)
    {
        trigger_high_at = cpu_us;
    }
    else if (gpio_calls && trigger_high_at)
    {
        trigger_width_us = cpu_us - trigger_high_at;
        trigger_high_at = 0;
        pulse_at = (uint32_t)cpu_us + 100U;
    }
    return E_OK;
}
unsigned int gpio_get_val(unsigned int pin)
{
    assert(pin == 1);
    advance(1);
    return pulse_at && cpu_us >= pulse_at && echo_mode != NO_ECHO &&
           (echo_mode == HIGH_ECHO || cpu_us < pulse_at + 1160U);
}
static S5_SAMPLE read(unsigned int mode, unsigned int * faults)
{
    deadline_us = cpu_us + 1000000U;
    echo_mode = mode;
    pulse_at = 0;
    *faults = 0;
    return s5_hcsr04_read(90, faults);
}
int main(void)
{
    TEST_INIT();
    unsigned int faults;
    cpu_us = deadline_us = 1000000U;
    assert(s5_hcsr04_init() == E_OK && gpio_calls == 2);
    S5_SAMPLE sample = read(NORMAL_ECHO, &faults);
    TEST_CHECK("S5-U31",
               sample.valid && sample.distance_mm >= 198 &&
                   sample.distance_mm <= 202 && !faults,
               "simulated 1160 us echo converts to about 200 mm");
    TEST_CHECK("S5-U38", trigger_width_us >= 10 && trigger_width_us <= 20,
               "10 us trigger pulse without WaitUsec");
    sample = read(NO_ECHO, &faults);
    TEST_CHECK("S5-U32", !sample.valid && faults == S5_NO_ECHO,
               "no echo is invalid");
    sample = read(HIGH_ECHO, &faults);
    TEST_CHECK("S5-U33", !sample.valid && faults == S5_ECHO_TIMEOUT,
               "echo stuck high is invalid");
    TEST_SUMMARY("hcsr04 driver");
    return 0;
}

/* Actual HC-SR04 driver with simulated MMIO/time: no physical test claim. */
#include "drv_hcsr04.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <assert.h>
#include <stdio.h>

static uint64_t cpu_us, deadline_us;
static uint32_t raw_us, tick, pulse_at, last_trigger_raw;
static unsigned int released, paused, frozen, echo_mode, gpio_calls;
static unsigned int freeze_after_trigger;
static uint64_t trigger_high_at, trigger_width_us;

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
    if (tick == 0x20cU && released && !paused && !frozen)
    {
        raw_us += us;
    }
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
    if (address == RESETS_RESET_DONE)
    {
        return released ? RESETS_RESET_TIMER : 0;
    }
    assert(address == TIMER_TIMERAWL);
    return raw_us;
}
void out_w(uint32_t address, uint32_t value)
{
    /* No watchdog CTRL, timer alarm, GPIO or PWM register writes allowed. */
    if (address == 0x4005802cU)
    {
        assert(value == 0x20cU);
        tick = value;
    }
    else if (address == 0x4000f000U)
    {
        assert(value == RESETS_RESET_TIMER);
        released = 1;
    }
    else
    {
        assert(address == TIMER_PAUSE && value == 0);
        paused = 0;
    }
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
        last_trigger_raw = raw_us;
        if (freeze_after_trigger)
        {
            frozen = 1;
        }
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
static void reset_mock(unsigned int freeze)
{
    cpu_us += 1000000U;
    deadline_us = cpu_us + 1000000U;
    raw_us += 1000000U;
    tick = released = paused = pulse_at = gpio_calls = 0;
    frozen = freeze;
    freeze_after_trigger = 0;
    trigger_high_at = trigger_width_us = 0;
    echo_mode = NORMAL_ECHO;
}
int main(void)
{
    unsigned int faults = 0;
    reset_mock(0);
    assert(s5_hcsr04_init() == E_OK && tick == 0x20cU && released &&
           gpio_calls == 2);
    puts("[TEST] S5-U30 PASS timer reference enabled, reset released, "
         "advancing");
    S5_SAMPLE sample = s5_hcsr04_read(90, &faults);
    assert(sample.valid && sample.distance_mm >= 198 &&
           sample.distance_mm <= 202 && !faults);
    puts("[TEST] S5-U31 PASS actual driver converts simulated 1160 us Echo");
    unsigned int shared_timer_pulse =
        trigger_width_us >= 10 && trigger_width_us <= 20;
    echo_mode = NO_ECHO;
    pulse_at = 0;
    faults = 0;
    uint32_t began = s5_now_ms();
    sample = s5_hcsr04_read(90, &faults);
    assert(!sample.valid && faults == S5_NO_ECHO && s5_now_ms() - began <= 100);
    puts("[TEST] S5-U32 PASS no Echo is invalid and bounded");
    echo_mode = HIGH_ECHO;
    pulse_at = 0;
    faults = 0;
    began = s5_now_ms();
    sample = s5_hcsr04_read(90, &faults);
    assert(!sample.valid && faults == S5_ECHO_TIMEOUT &&
           s5_now_ms() - began <= 100);
    puts("[TEST] S5-U33 PASS never-falling Echo is invalid and bounded");
    frozen = 1;
    raw_us = last_trigger_raw + 1;
    faults = 0;
    pulse_at = 0;
    began = s5_now_ms();
    sample = s5_hcsr04_read(90, &faults);
    assert(!sample.valid && faults == S5_SCAN_TIMEOUT &&
           s5_now_ms() - began <= 110);
    puts(
        "[TEST] S5-U34 PASS frozen timer cannot trap measurement-spacing wait");
    reset_mock(1);
    assert(s5_hcsr04_init() == E_TMOUT && gpio_calls == 0);
    puts("[TEST] S5-U35 PASS frozen timer fails initialization before GPIO "
         "setup");
    reset_mock(0);
    assert(s5_hcsr04_init() == E_OK);
    freeze_after_trigger = 1;
    echo_mode = NO_ECHO;
    faults = 0;
    began = s5_now_ms();
    sample = s5_hcsr04_read(90, &faults);
    assert(!sample.valid && faults == S5_NO_ECHO && s5_now_ms() - began <= 40);
    puts("[TEST] S5-U36 PASS independent RTOS deadline bounds frozen "
         "rising-edge wait");
    raw_us += 1000000U;
    frozen = 0;
    pulse_at = 0;
    echo_mode = HIGH_ECHO;
    faults = 0;
    began = s5_now_ms();
    sample = s5_hcsr04_read(90, &faults);
    assert(!sample.valid && faults == S5_ECHO_TIMEOUT &&
           s5_now_ms() - began <= 40);
    puts("[TEST] S5-U37 PASS independent RTOS deadline bounds frozen "
         "high-pulse wait");
    assert(shared_timer_pulse);
    puts("[TEST] S5-U38 PASS 10 us shared-timer trigger without per-core "
         "WaitUsec");
    reset_mock(0);
    assert(s5_hcsr04_init() == E_OK);
    frozen = 1;
    faults = 0;
    began = s5_now_ms();
    sample = s5_hcsr04_read(90, &faults);
    assert(!sample.valid && faults == S5_SCAN_TIMEOUT && !trigger_high_at &&
           s5_now_ms() - began <= 40);
    puts("[TEST] S5-U39 PASS stalled trigger timer returns TRIG LOW and exits");
    puts("[TEST] hcsr04 driver SUMMARY 10 passed, 0 failed (simulated "
         "hardware)");
    return 0;
}

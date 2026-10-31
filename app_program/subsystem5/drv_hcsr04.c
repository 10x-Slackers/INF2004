/* Called only by the scan task. Timer polling is bounded, not IRQ-masked. */
#include "drv_hcsr04.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>

/* RP2040 timer_tick comes from clk_ref (12 MHz), not the kernel SysTick.
 * This make-based firmware does not run Pico SDK clock/timer startup. */
#define S5_WATCHDOG_TICK        0x4005802cU
#define S5_WATCHDOG_TICK_ENABLE (1U << 9)
#define S5_TIMER_START_MS       100U
#define S5_ECHO_BACKUP_MS       (S5_ECHO_TIMEOUT_US / 1000U + 10U)
#define S5_TRIGGER_US           10U

#if !TM_CONSOLE_UART
static ER s5_range_timer_init(void)
{
    uint32_t started = s5_now_ms();
    out_w(S5_WATCHDOG_TICK, S5_WATCHDOG_TICK_ENABLE | XOSC_MHz);
    /* Release TIMER only; never reset a running counter or touch alarms. */
    out_w(RESETS_RESET + 0x3000U, RESETS_RESET_TIMER);
    while (!(in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER))
    {
        if (s5_now_ms() - started >= S5_TIMER_START_MS)
        {
            return E_TMOUT;
        }
        tk_dly_tsk(10);
    }
    out_w(TIMER_PAUSE, 0);
    uint32_t before = in_w(TIMER_TIMERAWL);
    tk_dly_tsk(10);
    return in_w(TIMER_TIMERAWL) != before ? E_OK : E_TMOUT;
}
#endif

ER s5_hcsr04_init(void)
{
#if TM_CONSOLE_UART
    return E_NOSPT; /* GP0/GP1 belong to the UART in this build. */
#else
    ER er = s5_range_timer_init();
    if (er < E_OK)
    {
        return er;
    }
    gpio_set_val(S5_TRIG_PIN, 0);
    gpio_set_pin(S5_TRIG_PIN, GPIO_MODE_OUT);
    return gpio_set_pin(S5_ECHO_PIN, GPIO_MODE_IN);
#endif
}

S5_SAMPLE s5_hcsr04_read(unsigned int angle, unsigned int * faults)
{
    static uint32_t previous_us;
    uint32_t start;
    uint32_t wait_started = s5_now_ms();
    unsigned int polls = 0;
    S5_SAMPLE sample = {0};
    sample.angle_deg = angle;
    /* Delay at least 60 ms after the previous trigger, including first boot.
     * Short waits round upwards to the kernel tick; recheck the raw timer. */
    while ((uint32_t)(in_w(TIMER_TIMERAWL) - previous_us) <
           S5_MEASURE_INTERVAL_US)
    {
        /* The RTOS clock is independent: a stopped range timer must not
         * leave the scanner pending forever in this delay loop. */
        if (s5_now_ms() - wait_started >= S5_TIMER_START_MS)
        {
            *faults |= S5_SCAN_TIMEOUT;
            sample.stamp_ms = s5_now_ms();
            return sample;
        }
        tk_dly_tsk(10);
    }
    if (gpio_get_val(S5_ECHO_PIN))
    {
        *faults |= S5_ECHO_TIMEOUT;
        sample.stamp_ms = s5_now_ms();
        return sample;
    }
    /* WaitUsec uses per-core SysTick, disabled on this SMP port's core 1.
     * Use the shared timer so task migration cannot leave TRIG high. */
    wait_started = s5_now_ms();
    gpio_set_val(S5_TRIG_PIN, 1);
    start = in_w(TIMER_TIMERAWL);
    while ((uint32_t)(in_w(TIMER_TIMERAWL) - start) < S5_TRIGGER_US)
    {
        if ((++polls & 127U) == 0 &&
            s5_now_ms() - wait_started >= S5_ECHO_BACKUP_MS)
        {
            gpio_set_val(S5_TRIG_PIN, 0);
            *faults |= S5_SCAN_TIMEOUT;
            sample.stamp_ms = s5_now_ms();
            return sample;
        }
    }
    gpio_set_val(S5_TRIG_PIN, 0);
    previous_us = in_w(TIMER_TIMERAWL);
    start = in_w(TIMER_TIMERAWL);
    wait_started = s5_now_ms();
    polls = 0;
    while (!gpio_get_val(S5_ECHO_PIN))
    {
        if ((uint32_t)(in_w(TIMER_TIMERAWL) - start) >= S5_ECHO_TIMEOUT_US ||
            ((++polls & 127U) == 0 &&
             s5_now_ms() - wait_started >= S5_ECHO_BACKUP_MS))
        {
            *faults |= S5_NO_ECHO;
            sample.stamp_ms = s5_now_ms();
            return sample;
        }
    }
    start = in_w(TIMER_TIMERAWL);
    wait_started = s5_now_ms();
    polls = 0;
    while (gpio_get_val(S5_ECHO_PIN))
    {
        if ((uint32_t)(in_w(TIMER_TIMERAWL) - start) >= S5_ECHO_TIMEOUT_US ||
            ((++polls & 127U) == 0 &&
             s5_now_ms() - wait_started >= S5_ECHO_BACKUP_MS))
        {
            *faults |= S5_ECHO_TIMEOUT;
            sample.stamp_ms = s5_now_ms();
            return sample;
        }
    }
    return s5_echo_sample((uint32_t)(in_w(TIMER_TIMERAWL) - start), angle,
                          s5_now_ms());
}

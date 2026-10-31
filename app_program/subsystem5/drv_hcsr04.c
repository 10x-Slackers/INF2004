/** @file drv_hcsr04.c
 *
 * @brief HC-SR04 ultrasonic range sensor.
 *
 * ECHO edges arrive through the shared GPIO interrupt, which records
 * their time on the 1 MHz TIMER and wakes the measuring task, so the task
 * sleeps instead of polling while the echo is out.
 */

#include "drv_hcsr04.h"
#include "scan_profile.h"
#include "../app.h"

#define DRV_HCSR04_TRIG     0u
#define DRV_HCSR04_ECHO     1u
#define DRV_HCSR04_TRIG_US  10u    /* Datasheet: at least 10 us. */
#define DRV_HCSR04_CYCLE_US 60000u /* Datasheet: 60 ms between triggers. */
#define DRV_HCSR04_ECHO_MS  50     /* With no target ECHO is high ~38 ms, */
#define DRV_HCSR04_BUSY_MS  250u   /* ...on some modules much longer. */
#define DRV_HCSR04_POLL_MS  10u    /* One kernel tick. */

/* Written by the edge callback, read after it wakes the task. */
static volatile uint32_t g_rise_us;
static volatile uint32_t g_width_us;
static volatile bool g_b_rose;
static volatile bool g_b_fell;
static volatile ID g_task;

static uint32_t g_trigger_us; /* Scan task only. */

static void drv_hcsr04_edge(uint32_t pins);

/*!
 * @brief Set up TRIG low and ECHO as an input, and register for the ECHO
 * edges on the shared GPIO interrupt (app_gpio_register()).
 *
 * @return True if the edge callback was registered.
 */
bool drv_hcsr04_init(void)
{
    (void)gpio_set_val(DRV_HCSR04_TRIG, 0u);
    (void)gpio_set_pin(DRV_HCSR04_TRIG, GPIO_MODE_OUT);
    (void)gpio_set_pin(DRV_HCSR04_ECHO, GPIO_MODE_IN);
    return app_gpio_register(1u << DRV_HCSR04_ECHO, drv_hcsr04_edge);
}

/*!
 * @brief Take one reading. Waits for 60 ms since the last trigger and for
 * ECHO to be low (a trigger while it is high is ignored), then up to 50 ms
 * for the echo.
 *
 * @param[out] p_mm Distance, SCAN_PROFILE_FAR_MM if nothing is in range.
 * @return False if no echo pulse started: the sensor is missing or faulty.
 */
bool drv_hcsr04_measure(uint32_t * p_mm)
{
    uint32_t start_us = 0u;
    uint32_t waited = 0u;

    while (((in_w(TIMER_TIMERAWL) - g_trigger_us) < DRV_HCSR04_CYCLE_US) ||
           ((0u != gpio_get_val(DRV_HCSR04_ECHO)) &&
            (waited < DRV_HCSR04_BUSY_MS)))
    {
        tk_dly_tsk(DRV_HCSR04_POLL_MS);
        waited += DRV_HCSR04_POLL_MS;
    }
    g_b_rose = false;
    g_b_fell = false;
    g_task = tk_get_tid();
    (void)tk_can_wup(TSK_SELF);

    /* Timed on TIMER: WaitUsec uses the SysTick, which is off on core 1. */
    (void)gpio_set_val(DRV_HCSR04_TRIG, 1u);
    start_us = in_w(TIMER_TIMERAWL);
    while ((in_w(TIMER_TIMERAWL) - start_us) < DRV_HCSR04_TRIG_US)
    {
        /* Busy-wait: the trigger pulse is far shorter than a tick. */
    }
    (void)gpio_set_val(DRV_HCSR04_TRIG, 0u);
    g_trigger_us = in_w(TIMER_TIMERAWL);

    (void)tk_slp_tsk(DRV_HCSR04_ECHO_MS);
    *p_mm = g_b_fell ? scan_profile_mm(g_width_us) : SCAN_PROFILE_FAR_MM;
    return g_b_rose;
}

/*!
 * @brief GPIO interrupt callback for ECHO: time the pulse, then wake the
 * measuring task on its falling edge.
 */
static void drv_hcsr04_edge(uint32_t pins)
{
    uint32_t const now_us = in_w(TIMER_TIMERAWL);

    (void)pins;
    if (0u != gpio_get_val(DRV_HCSR04_ECHO))
    {
        g_rise_us = now_us;
        g_b_rose = true;
    }
    else if (g_b_rose && (!g_b_fell))
    {
        g_width_us = now_us - g_rise_us;
        g_b_fell = true;
        (void)tk_wup_tsk(g_task);
    }
    else
    {
        /* A falling edge left over from before the trigger. */
    }
}

/*** end of file ***/

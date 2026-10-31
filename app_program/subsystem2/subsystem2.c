/** @file subsystem2.c
 *
 * @brief Subsystem 2 (motion control) start-up and the 10 ms motion task,
 * on core 0.
 */

#include "subsystem2.h"
#include "drv_motion.h"

#define SUBSYSTEM2_TICK_MAX 0xffffu

/* Longest control step in us. Written only by the motion task. */
static volatile uint16_t g_max_tick_us;

static void subsystem2_motion_task(INT stacd, void * p_exinf);

/*!
 * @brief Set up the motors and encoders, with both motors braked.
 */
ER subsystem2_init(void)
{
    ER result = E_SYS;

    if (drv_motion_init())
    {
        motion_control_init();
        result = E_OK;
    }
    return result;
}

/*!
 * @brief Start the motion task and the 10 ms cyclic handler.
 */
ER subsystem2_start(void)
{
    /* FP is the kernel's generic task pointer type. */
    return app_start_cyclic(app_start_task((FP)subsystem2_motion_task,
                                           S2_PRI_MOTION, TP_PRC1, "s2_motion"),
                            MOTION_CONTROL_PERIOD_MS);
}

/*!
 * @brief Longest control step since boot, in us. Saturates at 65535.
 */
uint32_t subsystem2_max_tick_us(void)
{
    return g_max_tick_us;
}

/*!
 * @brief Motion task: one timed control step per wake-up.
 */
static void subsystem2_motion_task(INT stacd, void * p_exinf)
{
    SYSTIM now = {0};
    uint32_t last_ms = 0u;
    uint32_t start_us = 0u;
    uint32_t elapsed_us = 0u;

    (void)stacd;
    (void)p_exinf;
    (void)tk_get_otm(&now);
    last_ms = now.lo;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        start_us = drv_motion_time_us();
        /* Kernel time since the last step: 10 ms, or 20 ms then 0 ms when a
         * late wake-up was queued. The low word wraps after 49 days. */
        (void)tk_get_otm(&now);
        motion_control_tick(now.lo - last_ms);
        last_ms = now.lo;
        elapsed_us = drv_motion_time_us() - start_us;
        if (elapsed_us > g_max_tick_us)
        {
            /* Saturated at 0xffff first, so the cast is exact. */
            g_max_tick_us = (elapsed_us > SUBSYSTEM2_TICK_MAX)
                                ? SUBSYSTEM2_TICK_MAX
                                : (uint16_t)elapsed_us;
        }
    }
}

/*** end of file ***/

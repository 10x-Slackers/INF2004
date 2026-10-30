/** @file subsystem2.c
 *
 * @brief Subsystem 2 (motion control) start-up, the 10 ms motion task and
 * the console task. Both tasks run on core 0.
 */

#include "subsystem2.h"
#include "drv_motion.h"
#include "console_motion.h"

#define SUBSYSTEM2_PERIOD_MS (MOTION_CONTROL_PERIOD_US / 1000u)
#define SUBSYSTEM2_TICK_MAX  0xffffu

static ID g_motion_task;

/* Longest control step in us. Written only by the motion task. */
static volatile uint16_t g_max_tick_us;

static void subsystem2_wake(void * p_exinf);
static void subsystem2_motion_task(INT stacd, void * p_exinf);

/*!
 * @brief Set up the motors and encoders, with both motors braked.
 */
EXPORT ER subsystem2_init(void)
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
 * @brief Start the motion and console tasks and the 10 ms cyclic handler.
 */
EXPORT ER subsystem2_start(void)
{
    T_CCYC cyclic = {0};
    ID console_task = 0;
    ID cyclic_id = 0;
    ER result = E_OK;

    /* FP is the kernel's generic task and handler pointer type. */
    g_motion_task = app_start_task((FP)subsystem2_motion_task, S2_PRI_MOTION,
                                   TP_PRC1, "s2_motion");
    console_task = app_start_task((FP)console_motion_task, S2_PRI_CONSOLE,
                                  TP_PRC1, "s2_console");
    cyclic.cycatr = TA_HLNG | TA_STA;
    cyclic.cychdr = (FP)subsystem2_wake; /* Kernel handler type, as above. */
    cyclic.cyctim = SUBSYSTEM2_PERIOD_MS;
    cyclic.cycphs = SUBSYSTEM2_PERIOD_MS;
    cyclic_id = tk_cre_cyc(&cyclic);

    if (g_motion_task < E_OK)
    {
        result = g_motion_task;
    }
    else if (console_task < E_OK)
    {
        result = console_task;
    }
    else if (cyclic_id < E_OK)
    {
        result = cyclic_id;
    }
    else
    {
        /* All created. */
    }
    return result;
}

/*!
 * @brief Longest control step since boot, in us. Saturates at 65535.
 */
uint32_t subsystem2_max_tick_us(void)
{
    return g_max_tick_us;
}

/*!
 * @brief Cyclic handler: wake the motion task every 10 ms.
 */
static void subsystem2_wake(void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_motion_task);
}

/*!
 * @brief Motion task: one timed control step per wake-up.
 */
static void subsystem2_motion_task(INT stacd, void * p_exinf)
{
    uint32_t start_us = 0u;
    uint32_t elapsed_us = 0u;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        start_us = drv_motion_time_us();
        motion_control_tick(start_us);
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

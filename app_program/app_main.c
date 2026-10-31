/** @file app_main.c
 *
 * @brief START HERE. usermain() below is the program's entry point.
 *
 * Reset -> the kernel initialises itself -> creates the "initial task" ->
 * the initial task calls usermain(), at priority 1 (the highest). On the
 * dual-core build the OTHER core may start a new task at once, so
 * usermain() works in two phases:
 *   1. every subsystem creates its kernel objects (subsystemN_init)
 *   2. every subsystem starts its tasks (subsystemN_start)
 * and then puts the initial task to sleep forever. If usermain() RETURNS,
 * the kernel shuts the whole system down.
 *
 * One subsystem per folder (app_program/subsystemN/), one per student.
 * The interfaces between them are listed in TEAM.md section 5: the
 * subsystem that RECEIVES or OWNS the data creates the kernel object and
 * publishes its ID in its header; the other side just uses it.
 *
 * Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "shared/vehicle_controller.h"

#define APP_ANY_CORE 0u

/*!
 * @brief Create and start one task. prc = TP_PRC1 (core 0), TP_PRC2
 * (core 1), or 0 for "any core".
 */
ID app_start_task(FP p_entry, PRI pri, UW prc, const char * p_name)
{
    T_CTSK task = {0};
    ID task_id = 0;

    task.task = p_entry;              /* Function the task runs. */
    task.itskpri = pri;               /* Initial priority (1 = highest). */
    task.stksz = APP_TASK_STACK_SIZE; /* The kernel allocates the stack. */
    task.tskatr = TA_HLNG | TA_RNG3;  /* C code, user protection level. */
#if TK_SUPPORT_SMP
    if (APP_ANY_CORE != prc)
    {
        task.tskatr |= TA_ASSPRC; /* Honour the affinity below. */
        task.assprc = prc;        /* Which core(s) it may run on. */
    }
#else
    (void)prc; /* Single-core build: no affinity. */
#endif

    /* tk_cre_tsk creates the task DORMANT; tk_sta_tsk makes it READY. */
    task_id = tk_cre_tsk(&task);
    if (task_id < E_OK)
    {
        app_check(task_id, p_name);
    }
    else
    {
        app_check(tk_sta_tsk(task_id, 0), p_name);
    }
    return task_id;
}

/*!
 * @brief Print a readable message if a kernel call failed.
 */
void app_check(ER result, const char * p_what)
{
    if (result < E_OK)
    {
        tm_printf((UB *)"ERROR %s: %d\n", p_what, result);
    }
}

/*!
 * @brief Entry point, called by the kernel's initial task.
 */
INT usermain(void)
{
    T_RVER version = {0};

    (void)tk_ref_ver(&version);
    tm_printf((UB *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
                    " (spec 0x%04x, product 0x%04x) ===\n",
              version.spver, version.prver);
#if TK_SUPPORT_SMP
    tm_printf((UB *)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1); /* tk_get_prc() counts from 1. */
#else
    tm_printf((UB *)"single-core build\n");
#endif

    /* The kernel leaves the PWM block in reset (USE_PTMR = 0 in
     * config/config.h). Release it once here for every PWM pin: motors,
     * servo, buzzer. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* Wait for the reset to finish. */
    }

    /* Phase 1: every subsystem creates its kernel objects. */
    app_check(subsystem1_init(), "subsystem1_init");
    app_check(subsystem2_init(), "subsystem2_init");
    app_check(subsystem3_init(), "subsystem3_init");
    app_check(subsystem4_init(), "subsystem4_init");
    app_check(subsystem5_init(), "subsystem5_init");

    /* Phase 2: every subsystem starts its tasks. */
    app_check(subsystem1_start(), "subsystem1_start");
    app_check(subsystem2_start(), "subsystem2_start");
    app_check(subsystem3_start(), "subsystem3_start");
    app_check(subsystem4_start(), "subsystem4_start");
    app_check(subsystem5_start(), "subsystem5_start");
#if !APP_TEST
    /* Test builds keep GP20 for the avoidance bench. */
    app_check(vehicle_controller_start(), "vehicle_controller_start");
#endif

    tm_printf((UB *)"all subsystems started\n\n");

#if APP_TEST
    /* Test build (make TEST=1): run every subsystem's tests in turn, here
     * in the initial task, while the system is running. Tests may call
     * tk_dly_tsk() to let the other tasks work for a while. */
    tm_printf((UB *)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf((UB *)"[TEST] === test run end ===\n");
#endif

    /* Sleep forever. Returning from usermain() would shut down. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/

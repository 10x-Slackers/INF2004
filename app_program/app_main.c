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
#include "shared/safety_monitor.h"
#include "shared/vehicle_controller.h"

#define APP_ANY_CORE 0u

/* GPIO bank 0 interrupt: RP2040 datasheet section 2.19. */
#define APP_GPIO_IRQ           13u /* IO_IRQ_BANK0 */
#define APP_GPIO_IRQ_PRIORITY  2
#define APP_GPIO_BANKS         4u
#define APP_GPIO_PINS_PER_BANK 8u
#define APP_GPIO_INTR          (IO_BANK0_BASE + 0xf0u)
#define APP_GPIO_INTE          (IO_BANK0_BASE + 0x100u)
#define APP_GPIO_INTS          (IO_BANK0_BASE + 0x120u)
#define APP_GPIO_EDGE_BOTH     0xcu /* Falling and rising edge bits. */
#define APP_GPIO_EDGE_BITS     0xccccccccu
#define APP_GPIO_CALLBACKS     2u /* Encoders (S2) and echo (S5). */

static ID g_fault_flag; /* The shared fault bitmask (app_fault.h). */

/* GPIO edge callbacks. pins is 0 in a free slot. */
static struct
{
    uint32_t pins;
    app_gpio_callback_t p_callback;
} g_gpio[APP_GPIO_CALLBACKS];

static void app_wake(void * p_exinf);
static void app_gpio_isr(UINT intno);

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
 * @brief Input with the pull-up only (both pulls on is a bus keeper).
 */
void app_pull_up(UW pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE);
}

/*!
 * @brief Wake a task every period_ms from a cyclic handler.
 *
 * @return The cyclic handler ID, or a negative error code.
 */
ER app_start_cyclic(ID task_id, RELTIM period_ms)
{
    T_CCYC cyclic = {0};

    cyclic.exinf = (void *)task_id; /* Handed to app_wake(). */
    cyclic.cycatr = TA_HLNG | TA_STA;
    cyclic.cychdr = (FP)app_wake; /* FP is the kernel's handler type. */
    cyclic.cyctim = period_ms;
    cyclic.cycphs = period_ms;
    return (task_id < E_OK) ? task_id : tk_cre_cyc(&cyclic);
}

/*!
 * @brief Call p_callback from the GPIO interrupt on edges of these pins.
 *
 * @param[in] pins Bit n set for each GPIO n to watch.
 * @return True if registered.
 */
bool app_gpio_register(uint32_t pins, app_gpio_callback_t p_callback)
{
    uint32_t taken = 0u;
    uint32_t i = 0u;
    uint32_t bank = 0u;
    uint32_t pin = 0u;
    uint32_t mask = 0u;
    bool b_accepted = false;

    for (i = 0u; i < APP_GPIO_CALLBACKS; i++)
    {
        taken |= g_gpio[i].pins;
    }
    for (i = 0u; (i < APP_GPIO_CALLBACKS) && !b_accepted; i++)
    {
        b_accepted = (0u == g_gpio[i].pins) && (0u == (pins & taken));
        if (b_accepted)
        {
            g_gpio[i].p_callback = p_callback;
            g_gpio[i].pins = pins;
        }
    }

    /* Clear old edge flags on these pins, then enable their interrupts. */
    for (bank = 0u; b_accepted && (bank < APP_GPIO_BANKS); bank++)
    {
        mask = 0u;
        for (pin = 0u; pin < APP_GPIO_PINS_PER_BANK; pin++)
        {
            if (0u != (pins & (1u << ((bank * APP_GPIO_PINS_PER_BANK) + pin))))
            {
                mask |= APP_GPIO_EDGE_BOTH << (pin * 4u);
            }
        }
        out_w(APP_GPIO_INTR + (bank * 4u), mask);
        set_w(APP_GPIO_INTE + (bank * 4u), mask);
    }
    return b_accepted;
}

/*!
 * @brief Set fault bits (APP_FAULT_..., app_fault.h). Safe from interrupts.
 */
void app_fault_set(uint32_t bits)
{
    (void)tk_set_flg(g_fault_flag, bits);
}

/*!
 * @brief Clear fault bits (APP_FAULT_...).
 */
void app_fault_clear(uint32_t bits)
{
    (void)tk_clr_flg(g_fault_flag, ~bits); /* Keeps the bits it is given. */
}

/*!
 * @brief Read the shared fault bitmask.
 *
 * @return APP_FAULT_... bits currently set.
 */
uint32_t app_faults(void)
{
    T_RFLG flag = {0};

    (void)tk_ref_flg(g_fault_flag, &flag);
    return flag.flgptn;
}

/*!
 * @brief Entry point, called by the kernel's initial task.
 */
INT usermain(void)
{
    T_RVER version = {0};
    T_CFLG flag = {0};
    T_DINT handler = {0};

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

    /* The fault bitmask first: any subsystem may set a fault. */
    g_fault_flag = tk_cre_flg(&flag);
    app_check(g_fault_flag, "fault flag");

    /* The GPIO interrupt next, on this core (0): subsystems register with
     * app_gpio_register() in their _init(). */
    handler.intatr = TA_HLNG;
    handler.inthdr = (FP)app_gpio_isr; /* Kernel handler type, as above. */
    app_check(tk_def_int(APP_GPIO_IRQ, &handler), "GPIO interrupt");
    EnableInt(APP_GPIO_IRQ, APP_GPIO_IRQ_PRIORITY);

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
    /* Test builds keep GP20 for the avoidance bench, and the benches drive
     * without a safety latch. The monitor first: the controller uses it. */
    app_check(safety_monitor_start(), "safety_monitor_start");
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

/*!
 * @brief Cyclic handler of app_start_cyclic(): wake the task in p_exinf.
 */
static void app_wake(void * p_exinf)
{
    (void)tk_wup_tsk((ID)p_exinf);
}

/*!
 * @brief GPIO bank 0 interrupt: clear the edge flags, then pass each
 * callback the edges on its pins.
 */
static void app_gpio_isr(UINT intno)
{
    uint32_t pins = 0u;
    uint32_t bank = 0u;
    uint32_t pin = 0u;
    uint32_t edges = 0u;
    uint32_t i = 0u;

    (void)intno;
    for (bank = 0u; bank < APP_GPIO_BANKS; bank++)
    {
        edges = in_w(APP_GPIO_INTS + (bank * 4u)) & APP_GPIO_EDGE_BITS;
        if (0u != edges) /* Usually only one bank has edges. */
        {
            out_w(APP_GPIO_INTR + (bank * 4u), edges);
            for (pin = 0u; pin < APP_GPIO_PINS_PER_BANK; pin++)
            {
                if (0u != (edges & (APP_GPIO_EDGE_BOTH << (pin * 4u))))
                {
                    pins |= 1u << ((bank * APP_GPIO_PINS_PER_BANK) + pin);
                }
            }
        }
    }

    /* Callbacks run after the flags are cleared, so a later edge is not
     * lost. Only enabled pins have edges, so a slot is never half set. */
    for (i = 0u; i < APP_GPIO_CALLBACKS; i++)
    {
        if (0u != (pins & g_gpio[i].pins))
        {
            g_gpio[i].p_callback(pins & g_gpio[i].pins);
        }
    }
}

/*** end of file ***/

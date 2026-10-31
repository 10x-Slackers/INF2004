/** @file app.h
 *
 * @brief Definitions shared by the whole team. Changed only by team
 * agreement. Anything that belongs to one subsystem goes in its own header
 * (app_program/subsystemN/subsystemN.h).
 *
 * Pins: free for projects are GP0..GP22 and GP26..GP28. GP0/GP1 are taken
 * by UART0 when built with "make CRASH_REPORT=1" (see README). Never use
 * GP23/24/25/29: they are wired to the Pico W radio. Claim every pin in
 * TEAM.md section 2 before using it.
 *
 * Task priorities: 1 is the highest; the kernel's USB console (3) and WiFi
 * (4) tasks sit above every application task. Each subsystem has its own
 * band (TEAM.md section 4), defined in its own header.
 *
 * The application code follows BARR-C:2018. Deviations shared by every
 * subsystem:
 * - 3.1.j, 3.2: layout is the repository's .clang-format.
 * - 4.2.a: the subsystemN_test.c files have no header; this file declares
 *   their functions.
 * - 5.2: kernel calls, task entries and handlers use kernel types (ER, ID,
 *   INT, UB, ...), as the kernel's prototypes do.
 * - 6.3.a: the TEST_* helpers in test_report.h are macros, because they
 *   report the caller's file and line.
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>  /* micro T-Kernel API (tk_xxx_yyy). */
#include <tm/tmonitor.h> /* T-Monitor API: tm_printf() console output. */
#include <bsp/libbsp.h>  /* Board helpers: gpio_set_pin/val, pwm_... */

#define APP_TASK_STACK_SIZE 2048 /* Bytes; tm_printf needs ~1 KB. */

/* Create and start a task. prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
 * 0 for "any core". */
ID app_start_task(FP p_entry, PRI pri, UW prc, const char * p_name);

/* Print a readable message if a kernel call failed (negative code). */
void app_check(ER result, const char * p_what);

/* Every subsystem provides these. usermain() calls ALL the _init()
 * functions first and only then ALL the _start() functions, so every
 * kernel object exists before any task can use it.
 *   subsystemN_init()   create this subsystem's kernel objects
 *   subsystemN_start()  create and start this subsystem's tasks
 *   subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 */
ER subsystem1_init(void);
ER subsystem2_init(void);
ER subsystem3_init(void);
ER subsystem4_init(void);
ER subsystem5_init(void);

ER subsystem1_start(void);
ER subsystem2_start(void);
ER subsystem3_start(void);
ER subsystem4_start(void);
ER subsystem5_start(void);

#if APP_TEST
void subsystem1_test(void);
void subsystem2_test(void);
void subsystem3_test(void);
void subsystem4_test(void);
void subsystem5_test(void);
#endif

#endif /* APP_H */

/*** end of file ***/

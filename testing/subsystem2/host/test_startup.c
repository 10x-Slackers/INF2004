/* Exercise the real startup selection without creating hardware tasks. */
#define APP_H
#define EXPORT
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

typedef int32_t INT;
typedef int32_t ID;
typedef int32_t ER;
typedef int32_t PRI;
typedef uint32_t UW;
typedef uint8_t UB;
typedef void (*FP)(void);
typedef struct
{
    FP task;
    PRI itskpri;
    uint32_t stksz;
    uint32_t tskatr;
    uint32_t assprc;
} T_CTSK;
typedef struct
{
    uint16_t spver;
    uint16_t prver;
} T_RVER;

#define E_OK 0
#define TMO_FEVR (-1)
#define TA_HLNG 1U
#define TA_RNG3 0U
#define TA_ASSPRC 2U
#define TASK_STACK_SIZE 2048U
#define CHECK(result, label) \
    do { (void)(label); assert((result) == E_OK); } while (0)

static uint32_t g_events[20];
static uint32_t g_used;
static jmp_buf g_asleep;

static ER tk_ref_ver(T_RVER * p_version)
{
    p_version->spver = 0x300U;
    p_version->prver = 0x300U;
    return E_OK;
}
#if TK_SUPPORT_SMP
static INT tk_get_prc(void)
{
    return 1;
}
#endif
static ID tk_cre_tsk(const T_CTSK * p_definition)
{
    (void)p_definition;
    return 1;
}
static ER tk_sta_tsk(ID task, INT code)
{
    (void)task;
    (void)code;
    return E_OK;
}
static void tk_slp_tsk(INT timeout)
{
    assert(TMO_FEVR == timeout);
    g_events[g_used++] = 900U;
    longjmp(g_asleep, 1);
}
static INT tm_printf(const UB * p_format, ...)
{
    (void)p_format;
    return 0;
}

/* A link failure catches any unguarded dependency on a skipped subsystem. */
#if !APP_S2_STANDALONE
static ER subsystem1_init(void) { g_events[g_used++] = 101U; return E_OK; }
static ER subsystem3_init(void) { g_events[g_used++] = 103U; return E_OK; }
static ER subsystem4_init(void) { g_events[g_used++] = 104U; return E_OK; }
static ER subsystem5_init(void) { g_events[g_used++] = 105U; return E_OK; }
static ER subsystem1_start(void) { g_events[g_used++] = 201U; return E_OK; }
static ER subsystem3_start(void) { g_events[g_used++] = 203U; return E_OK; }
static ER subsystem4_start(void) { g_events[g_used++] = 204U; return E_OK; }
static ER subsystem5_start(void) { g_events[g_used++] = 205U; return E_OK; }
#if APP_TEST
static void subsystem1_test(void) { g_events[g_used++] = 301U; }
static void subsystem3_test(void) { g_events[g_used++] = 303U; }
static void subsystem4_test(void) { g_events[g_used++] = 304U; }
static void subsystem5_test(void) { g_events[g_used++] = 305U; }
#endif
#endif
static ER subsystem2_init(void) { g_events[g_used++] = 102U; return E_OK; }
static ER subsystem2_start(void) { g_events[g_used++] = 202U; return E_OK; }
#if APP_TEST
static void subsystem2_test(void) { g_events[g_used++] = 302U; }
#endif

/* APP_H skips only the platform header; the startup body is the real source. */
#include "../../../app_program/app_main.c"

int main(void)
{
    static const uint32_t expected[] = {
#if APP_S2_STANDALONE
        102U, 202U,
#if APP_TEST
        302U,
#endif
#else
        101U, 102U, 103U, 104U, 105U,
        201U, 202U, 203U, 204U, 205U,
#if APP_TEST
        301U, 302U, 303U, 304U, 305U,
#endif
#endif
        900U,
    };
    if (0 == setjmp(g_asleep))
    {
        (void)usermain();
        assert(0); /* Startup must sleep rather than return. */
    }
    assert(g_used == sizeof(expected) / sizeof(expected[0]));
    for (uint32_t index = 0U; index < g_used; index++)
    {
        assert(g_events[index] == expected[index]);
    }
    printf("PASS startup: SMP=%d TEST=%d S2_STANDALONE=%d\n",
           TK_SUPPORT_SMP, APP_TEST, APP_S2_STANDALONE);
    return 0;
}

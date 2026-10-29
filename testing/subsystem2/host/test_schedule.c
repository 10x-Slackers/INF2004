/* Exercise the real task and initialization with a tick/flag model. */
#define APP_H
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../../../app_program/subsystem2/motion_control.h"
#include "../../../app_program/subsystem2/drv_motion.h"

typedef int32_t INT;
typedef int32_t ID;
typedef int32_t ER;
typedef int32_t PRI;
typedef uint32_t UINT;
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
#define TASK_STACK_SIZE 2048u
#define TA_RNG3 0u
typedef struct { uint32_t flgatr; uint32_t iflgptn; } T_CFLG;
typedef struct
{
    uint32_t cycatr;
    FP cychdr;
    uint32_t cyctim;
    uint32_t cycphs;
    uint32_t assprc;
} T_CCYC;
#define E_OK 0
#define E_SYS (-5)
#define E_OBJ (-41)
#define TMO_FEVR (-1)
#define TWF_BITCLR 32u
#define TA_HLNG 1u
#define TA_STA 2u
#define TA_ASSPRC 65536u
#define TP_PRC1 1u

static uint32_t g_now;
static uint32_t g_flag;
static uint32_t g_ticks;
static uint32_t g_recorded;
static uint32_t g_safety;
static uint32_t g_waits;
static uint32_t g_delay;
static ID g_create_flag = 1;
static ID g_create_cycle = 2;
static uint32_t g_deleted;
static T_CCYC g_cycle;
static jmp_buf g_done;

static ID tk_cre_flg(const T_CFLG * p_flag)
{
    assert(0u == p_flag->iflgptn);
    return g_create_flag;
}
static ID tk_cre_cyc(const T_CCYC * p_cycle)
{
    g_cycle = *p_cycle;
    return g_create_cycle;
}
static ER tk_del_flg(ID flag)
{
    assert(1 == flag);
    g_deleted++;
    return E_OK;
}
static ER tk_set_flg(ID flag, UINT bits)
{
    assert(1 == flag);
    g_flag |= bits;
    return E_OK;
}
static void tk_ext_tsk(void)
{
    longjmp(g_done, 1);
}
static ER tk_wai_flg(ID flag, UINT bits, UINT mode,
                     UINT * p_pattern, INT timeout);
static ID g_task_create = 3;
static ER g_task_start = E_OK;
static ID tk_cre_tsk(const T_CTSK * p_task)
{
    assert(2048u == p_task->stksz);
    assert(6 == p_task->itskpri || 22 == p_task->itskpri);
#if TK_SUPPORT_SMP
    assert(TP_PRC1 == p_task->assprc);
#endif
    return g_task_create;
}
static ER tk_sta_tsk(ID task, INT code)
{
    (void)task;
    (void)code;
    return g_task_start;
}
static ER tk_del_tsk(ID task)
{
    (void)task;
    return E_OK;
}
static void tk_dis_dsp(void) {}
static void tk_ena_dsp(void) {}
static void tk_dly_tsk(uint32_t delay) { (void)delay; assert(0); }
static char g_output[256];
static INT tm_printf(const UB * p_format, ...)
{
    va_list arguments;
    INT length = 0;
    va_start(arguments, p_format);
    length = vsnprintf(g_output, sizeof(g_output),
                        (const char *)p_format, arguments);
    va_end(arguments);
    return length;
}
int32_t motion_control_decode_count(uint32_t value)
{
    return (int32_t)value - 8388608;
}
int32_t motion_control_decode_speed(uint32_t value)
{
    return (int32_t)value - 2048;
}
void drv_motion_snapshot(drv_motion_encoders_t * p_counts)
{
    p_counts->left = 8388607u;
    p_counts->right = 8388610u;
    p_counts->left_phase = 2u;
    p_counts->right_phase = 3u;
    p_counts->left_invalid = 4u;
    p_counts->right_invalid = 5u;
    p_counts->invalid = 9u;
    p_counts->overflow = 1u;
}
uint32_t drv_motion_time_us(void) { return g_now; }
void motion_control_init(void) {}
void motion_control_get_config(motion_control_config_t * p_config)
{
    memset(p_config, 0, sizeof(*p_config));
}

void motion_control_get_status(motion_control_status_t * p_status)
{
    p_status->faults = 0u;
}
void motion_control_safety_stop(uint32_t faults) { g_safety |= faults; }
void motion_control_tick(uint32_t now)
{
    assert(now == g_now);
    g_ticks++;
    g_now += 210u;
}
void motion_control_record_execution(uint32_t elapsed)
{
    assert(210u == elapsed);
    g_recorded++;
}
int32_t console_motion_getchar(void) { return -1; }
bool console_motion_execute(const char * p_line)
{
    (void)p_line;
    return false;
}

#include "../../../app_program/subsystem2/subsystem2.c"

static ER tk_wai_flg(ID flag, UINT bits, UINT mode,
                     UINT * p_pattern, INT timeout)
{
    assert(1 == flag && 1u == bits && TWF_BITCLR == mode);
    assert(10 == timeout);
    if (5u == g_waits)
    {
        return -1; /* Check failure stops, brakes through tick, then exits. */
    }
    /* Pending notifications need not provide a full period between wakes. */
    g_now = ((1u == g_waits) ? 20500u :
             (20000u + ((g_waits > 1u) ? g_waits - 1u : 0u) * 10000u)) +
            g_delay;
    subsystem2_release(NULL);
    subsystem2_release(NULL); /* Coalesce notifications, never replay ticks. */
    assert(1u == g_flag);
    *p_pattern = g_flag;
    g_flag &= ~bits;
    g_waits++;
    return E_OK;
}

int main(void)
{
    console_motion_encoder_report();
    assert(0 == strcmp(g_output,
        "[encoder] raw_counts=-1,2 phase=2,3 invalid=4,5 "
        "total_invalid=9 overflow=1\n"));
    assert(0u == g_ticks && 0u == g_safety);
    assert(E_OBJ == subsystem2_start());
    assert(E_OK == subsystem2_init());
    assert(10u == g_cycle.cyctim && 1u == g_cycle.cycphs);
    assert(0u != (g_cycle.cycatr & TA_STA));
#if TK_SUPPORT_SMP
    assert(TP_PRC1 == g_cycle.assprc);
    assert(0u != (g_cycle.cycatr & TA_ASSPRC));
#endif
    assert(E_OK == subsystem2_start());
    assert(E_OBJ == subsystem2_start());
    g_start.started = false; /* Isolate startup failure, not a supported retry. */
    g_task_create = -7;
    assert(-7 == subsystem2_start());
    g_task_create = 3;
    g_start.started = false;
    g_task_start = -8;
    assert(-8 == subsystem2_start());
    assert(0u != (g_safety & MOTION_CONTROL_INIT));
    g_safety = 0u;
    if (0 == setjmp(g_done))
    {
        subsystem2_motion_task(0, NULL);
        assert(0);
    }
    assert(5u == g_recorded && 6u == g_ticks);
    assert(MOTION_CONTROL_TIMING == g_safety);
    assert(E_OBJ == subsystem2_init());
    g_motion_flag = 0; /* Isolate a new boot with object-creation failure. */
    g_create_cycle = -5;
    assert(-5 == subsystem2_init());
    assert(1u == g_deleted);
    g_create_flag = -6;
    assert(-6 == subsystem2_init());
    printf("PASS cyclic task/flag model SMP=%d\n", TK_SUPPORT_SMP);
    return 0;
}

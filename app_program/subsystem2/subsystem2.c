#include "subsystem2.h"
#include "drv_motion.h"
#include "console_motion.h"

#define SUBSYSTEM2_RX_BUDGET 64u

static ID g_motion_flag;
static struct
{
    unsigned int started : 1;
} g_start;

/* Signal only; control and application locks stay in the motion task. */
static void subsystem2_release(void * p_exinf)
{
    (void)p_exinf;
    (void)tk_set_flg(g_motion_flag, 1u);
}

static void subsystem2_motion_task(INT stacd, void * p_exinf)
{
    uint32_t started_us = 0u;
    UINT pattern = 0u;
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        if (E_OK != tk_wai_flg(g_motion_flag, 1u, TWF_BITCLR,
                               &pattern, MOTION_CONTROL_PERIOD_US / 1000u))
        {
            motion_control_safety_stop(MOTION_CONTROL_TIMING);
            motion_control_tick(drv_motion_time_us());
            tk_ext_tsk();
        }
        started_us = drv_motion_time_us();
        motion_control_tick(started_us);
        motion_control_record_execution(drv_motion_time_us() - started_us);
    }
}

ER subsystem2_init(void)
{
    motion_control_status_t status = {0};
    T_CFLG flag = {0};
    T_CCYC cycle = {0};
    ID cycle_id = 0;

    if (g_motion_flag > E_OK)
    {
        /* Startup-only: never replace live objects or IRQ state. */
        return E_OBJ;
    }
    motion_control_init();
    motion_control_get_status(&status);
    if (0u != status.faults)
    {
        return E_SYS;
    }
    g_motion_flag = tk_cre_flg(&flag);
    if (g_motion_flag < E_OK)
    {
        motion_control_safety_stop(MOTION_CONTROL_INIT);
        return g_motion_flag;
    }
    cycle.cycatr = TA_HLNG | TA_STA;
#if TK_SUPPORT_SMP
    cycle.cycatr |= TA_ASSPRC;
    cycle.assprc = TP_PRC1;
#endif
    /* FP adapts the kernel callback ABI; the first release is tick-rounded. */
    cycle.cychdr = (FP)subsystem2_release;
    cycle.cyctim = MOTION_CONTROL_PERIOD_US / 1000u;
    cycle.cycphs = 1u;
    cycle_id = tk_cre_cyc(&cycle);
    if (cycle_id < E_OK)
    {
        (void)tk_del_flg(g_motion_flag);
        g_motion_flag = 0;
        motion_control_safety_stop(MOTION_CONTROL_INIT);
    }
    return (cycle_id < E_OK) ? cycle_id : E_OK;
}

static ID subsystem2_start_task(FP entry, PRI priority)
{
    T_CTSK task = {0};
    ID task_id = 0;
    ER result = E_OK;

    task.task = entry;
    task.itskpri = priority;
    task.stksz = TASK_STACK_SIZE;
    task.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    task.tskatr |= TA_ASSPRC;
    task.assprc = TP_PRC1;
#endif
    task_id = tk_cre_tsk(&task);
    if (task_id < E_OK)
    {
        return task_id;
    }
    result = tk_sta_tsk(task_id, 0);
    if (result < E_OK)
    {
        (void)tk_del_tsk(task_id);
        return result;
    }
    return task_id;
}

ER subsystem2_start(void)
{
    ID task_id = 0;
    if ((g_motion_flag <= E_OK) || g_start.started)
    {
        return E_OBJ;
    }
    g_start.started = true;
    /* FP adapts the kernel's legacy unprototyped task callback type. */
    task_id = subsystem2_start_task((FP)subsystem2_motion_task, S2_PRI_MOTION);
    if (task_id >= E_OK)
    {
        task_id = subsystem2_start_task((FP)subsystem2_console_task,
                                        S2_PRI_CONSOLE);
    }
    if (task_id < E_OK)
    {
        motion_control_safety_stop(MOTION_CONTROL_INIT);
    }
    return (task_id < E_OK) ? task_id : E_OK;
}
void console_motion_config_report(void)
{
    motion_control_config_t config = {0};
    motion_control_get_config(&config);
    tm_printf((const UB *)
        "[config] circ=%u track=%u cpr=%u lcpr=%u rcpr=%u "
        "kp=%u ki=%u kd=%u speed=%u limit=%u timeout=%u stall=%u "
        "mlrev=%u mrrev=%u elrev=%u errev=%u confirm=%u\n",
        config.circumference_mm, config.track_mm, config.counts_per_rev,
        config.left_counts_per_rev, config.right_counts_per_rev,
        config.kp_q8, config.ki_q8, config.kd_q8, config.speed_mm_s,
        config.duty_limit, config.timeout_ms, config.stall_ms,
        config.motor_left_reverse, config.motor_right_reverse,
        config.encoder_left_reverse, config.encoder_right_reverse,
        config.confirmed);
}

void console_motion_encoder_report(void)
{
    drv_motion_encoders_t counts = {0};
    drv_motion_snapshot(&counts);
    tm_printf((const UB *)
        "[encoder] raw_counts=%d,%d phase=%u,%u invalid=%u,%u "
        "total_invalid=%u overflow=%u\n",
        motion_control_decode_count(counts.left),
        motion_control_decode_count(counts.right),
        counts.left_phase, counts.right_phase,
        counts.left_invalid, counts.right_invalid,
        counts.invalid, counts.overflow);
}

void console_motion_report(void)
{
    motion_control_status_t status = {0};
    motion_control_get_status(&status);
    tm_printf((const UB *)
        "[motion] state=%u armed=%u faults=%u counts=%d,%d "
        "mm=%d,%d avg=%d speed_mm_s=%d,%d stamp_us=%u "
        "max_us=%u period_us=%u misses=%u\n",
        status.state, status.armed, status.faults,
        motion_control_decode_count(status.left_count),
        motion_control_decode_count(status.right_count),
        motion_control_decode_count(status.left_mm),
        motion_control_decode_count(status.right_mm),
        motion_control_decode_count(status.average_mm),
        motion_control_decode_speed(status.left_mm_s),
        motion_control_decode_speed(status.right_mm_s),
        status.timestamp_us, status.max_execution_us,
        status.max_period_us, status.missed_periods);
}

void subsystem2_console_task(INT stacd, void * p_exinf)
{
    char line[64] = {0};
    uint32_t used = 0u;
    INT character = 0;
    bool overflow = false;
    uint32_t budget = 0u;
    bool accepted = false;
    (void)stacd;
    (void)p_exinf;

    tm_printf((const UB *)
        "[motion] wiring: left=M1 GP16/17 Grove4; "
        "right=M2 GP2/3 Grove2\n");
#if S2_DIAGNOSTIC
    tm_printf((const UB *)
        "[BUDDY2 DIAGNOSTIC] physical left=GP16/17; "
        "physical right=GP2/3; calibration required; "
        "explicit arm required\n");
#endif
    tm_printf((const UB *)
        "[motion] console: status, encoder, config (disarmed), arm, "
        "disarm, reset, stop, speed, forward, backward, left, right, "
        "motor. ! = latched stop\n");
    for (;;)
    {
        for (budget = 0u; budget < SUBSYSTEM2_RX_BUDGET; budget++)
        {
            character = console_motion_getchar();
            if (character < 0)
            {
                break;
            }
            if ('!' == character)
            {
                motion_control_safety_stop(0u);
                used = 0u;
                overflow = false;
            }
            else if (('\n' == character) || ('\r' == character))
            {
                if ((used > 0u) || overflow)
                {
                    line[used] = '\0';
                    /* Serialize a whole command against the USB stop hook. */
                    tk_dis_dsp();
                    accepted = (!overflow) && console_motion_execute(line);
                    tk_ena_dsp();
                    tm_printf((const UB *)"[motion] %s\n",
                               accepted ? "OK" : "REJECTED");
                }
                used = 0u;
                overflow = false;
            }
            else if ((character >= 32) && (character <= 126))
            {
                if (used < sizeof(line) - 1u)
                {
                    line[used] = (char)character; /* Printable ASCII only. */
                    used++;
                }
                else
                {
                    overflow = true;
                }
            }
            else
            {
                overflow = true;
            }
        }
        tk_dly_tsk(10);
    }
}
/* End of file. */

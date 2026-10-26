#include "subsystem2.h"
#include "drv_motion.h"
#include "console_motion.h"

#define SUBSYSTEM2_RX_BUDGET 64u

static void subsystem2_motion_task(INT stacd, void * p_exinf)
{
    uint32_t next_us = drv_motion_time_us();
    uint32_t started_us = 0u;
    uint32_t now_us = 0u;
    int32_t remaining_us = 0;
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        started_us = drv_motion_time_us();
        motion_control_tick(started_us);
        motion_control_record_execution(drv_motion_time_us() - started_us);
        next_us += MOTION_CONTROL_PERIOD_US;
        now_us = drv_motion_time_us();
        /* Deadlines are less than 2^31 us apart, including timer wrap. */
        remaining_us = (int32_t)(next_us - now_us);
        if (remaining_us > 0)
        {
            tk_dly_tsk((RELTIM)((remaining_us + 999) / 1000));
        }
        else
        {
            next_us = now_us;
            tk_dly_tsk(1);
        }
    }
}

ER subsystem2_init(void)
{
    motion_control_status_t status = {0};
    motion_control_init();
    motion_control_get_status(&status);
    return (0u == status.faults) ? E_OK : E_SYS;
}

ER subsystem2_start(void)
{
    /* FP adapts the kernel's legacy unprototyped task callback type. */
    ID task_id = app_start_task((FP)subsystem2_motion_task,
                                S2_PRI_MOTION, TP_PRC1, "wheel_motion");
    if (task_id >= E_OK)
    {
        task_id = app_start_task((FP)subsystem2_console_task,
                                 S2_PRI_CONSOLE, TP_PRC1, "console_motion");
    }
    return (task_id < E_OK) ? task_id : E_OK;
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

    tm_printf((const UB *)"[motion] console: status, config, arm, "
                            "disarm, reset, stop, speed, forward, backward, "
                            "left, right, motor. ! = latched stop\n");
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

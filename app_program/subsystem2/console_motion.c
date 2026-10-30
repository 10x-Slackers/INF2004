/** @file console_motion.c
 *
 * @brief USB console for testing, calibrating and tuning motion control.
 *
 * One command per line at 115200 8N1; see docs/MOTION_CONTROL.md. A
 * status line is printed whenever a command finishes.
 */

#include "console_motion.h"
#include "motion_control.h"
#include "drv_motion.h"
#include "subsystem2.h"
#include <stdbool.h>

#define CONSOLE_MOTION_RX_BYTES   64u
#define CONSOLE_MOTION_LINE_BYTES 32u
#define CONSOLE_MOTION_MAX_ARGS   3u
#define CONSOLE_MOTION_POLL_MS    10
#define CONSOLE_MOTION_NUMBER_MAX 1000000 /* Stops int32_t overflow. */

/* NOTE: The USB task and this task both run on core 0, so this one-writer
 * ring needs no lock. */
static volatile uint8_t g_rx_bytes[CONSOLE_MOTION_RX_BYTES];
static volatile uint8_t g_rx_head; /* Written only by the USB task. */
static volatile uint8_t g_rx_tail; /* Written only by the console task. */

/* Last accepted "drive" speeds, resent every poll. Console task only. */
static int16_t g_drive_left;
static int16_t g_drive_right;

static void console_motion_report(void);
static int32_t console_motion_number(const char * p_text);
static bool console_motion_is(const char * p_line, const char * p_word);
static bool console_motion_execute(const char * p_line);

/*!
 * @brief Store one received byte. Dropped if the buffer is full.
 */
void tm_console_receive_byte(uint8_t byte)
{
    /* The remainder is below 64, so it fits uint8_t. */
    uint8_t next = (uint8_t)((g_rx_head + 1u) % CONSOLE_MOTION_RX_BYTES);

    if (next != g_rx_tail)
    {
        g_rx_bytes[g_rx_head] = byte;
        g_rx_head = next;
    }
}

/*!
 * @brief Console task: run each line, renew "drive", print trace and
 * results.
 */
void console_motion_task(INT stacd, void * p_exinf)
{
    char line[CONSOLE_MOTION_LINE_BYTES] = {0};
    uint8_t used = 0u;
    uint8_t byte = 0u;
    bool b_trace = false;
    bool b_ok = false;
    bool b_renew = false;
    motion_control_status_t status = {0};
    uint32_t last_state = MOTION_CONTROL_IDLE;

    (void)stacd;
    (void)p_exinf;

    /* tm_printf() takes UB *; the format strings are plain ASCII. */
    tm_printf((UB *)"[motion] ready: forward/backward mm, left/right deg, "
                    "drive l r, stop, speed, track, pid, status, trace\n");
    for (;;)
    {
        while (g_rx_tail != g_rx_head)
        {
            byte = g_rx_bytes[g_rx_tail];
            /* The remainder is below 64, so it fits uint8_t. */
            g_rx_tail = (uint8_t)((g_rx_tail + 1u) % CONSOLE_MOTION_RX_BYTES);
            if (('\r' == byte) || ('\n' == byte))
            {
                line[used] = '\0';
                if (console_motion_is(line, "trace"))
                {
                    b_trace = !b_trace;
                }
                else if (used > 0u)
                {
                    b_ok = console_motion_execute(line);
                    if (!console_motion_is(line, "status"))
                    {
                        /* Keep an accepted drive going until the next
                         * command. */
                        b_renew = b_ok && console_motion_is(line, "drive");
                    }
                    /* (UB *): tm_printf() takes unsigned chars. */
                    tm_printf((UB *)"[motion] %s\n", b_ok ? "OK" : "REJECTED");
                }
                else
                {
                    /* Empty line. */
                }
                used = 0u;
            }
            else if (used < (CONSOLE_MOTION_LINE_BYTES - 1u))
            {
                line[used] = (char)byte; /* Typed text is ASCII. */
                used++;
            }
            else
            {
                /* Line too long: drop the rest. */
            }
        }

        motion_control_get_status(&status);
        b_renew = b_renew && (MOTION_CONTROL_RUNNING == status.state);
        if (b_renew)
        {
            (void)motion_control_drive(g_drive_left, g_drive_right);
        }
        if (b_trace && (MOTION_CONTROL_RUNNING == status.state))
        {
            /* (UB *): tm_printf() takes unsigned chars. */
            tm_printf((UB *)"[trace] %u %d %d\n", drv_motion_time_us() / 1000u,
                      motion_control_decode_speed(status.left_speed),
                      motion_control_decode_speed(status.right_speed));
        }
        if ((MOTION_CONTROL_RUNNING == last_state) &&
            (MOTION_CONTROL_RUNNING != status.state))
        {
            console_motion_report();
        }
        last_state = status.state;
        tk_dly_tsk(CONSOLE_MOTION_POLL_MS);
    }
}

/*!
 * @brief Print the motion status on one line.
 */
static void console_motion_report(void)
{
    motion_control_status_t status = {0};

    motion_control_get_status(&status);
    /* (UB *): tm_printf() takes unsigned chars. */
    tm_printf((UB *)"[motion] state=%u cmd=%u faults=%u heading=%u "
                    "dist_mm=%u counts=%u,%u speed=%d,%d max_tick_us=%u\n",
              status.state, status.command, status.faults, status.heading_deg,
              status.distance_mm, status.left_count, status.right_count,
              motion_control_decode_speed(status.left_speed),
              motion_control_decode_speed(status.right_speed),
              subsystem2_max_tick_us());
}

/*!
 * @brief Read a signed decimal number, stopping at the first non-digit.
 */
static int32_t console_motion_number(const char * p_text)
{
    int32_t value = 0;
    int32_t sign = 1;

    if ('-' == *p_text)
    {
        sign = -1;
        p_text++;
    }
    while ((*p_text >= '0') && (*p_text <= '9') &&
           (value < CONSOLE_MOTION_NUMBER_MAX))
    {
        value = (value * 10) + (*p_text - '0');
        p_text++;
    }
    return sign * value;
}

/*!
 * @brief True if the first word of p_line is p_word.
 */
static bool console_motion_is(const char * p_line, const char * p_word)
{
    while (('\0' != *p_word) && (*p_line == *p_word))
    {
        p_line++;
        p_word++;
    }
    return (('\0' == *p_word) && ((' ' == *p_line) || ('\0' == *p_line)));
}

/*!
 * @brief Run one line: a command word, then numbers after single spaces.
 *
 * @return True if the command was known and accepted.
 */
static bool console_motion_execute(const char * p_line)
{
    const char * p_text = NULL;
    int32_t args[CONSOLE_MOTION_MAX_ARGS] = {0};
    uint32_t count = 0u;
    bool b_ok = false;

    for (p_text = p_line; '\0' != *p_text; p_text++)
    {
        if ((' ' == *p_text) && (count < CONSOLE_MOTION_MAX_ARGS))
        {
            args[count] = console_motion_number(p_text + 1);
            count++;
        }
    }

    /* A negative number cast to uint32_t becomes huge, and is rejected. */
    if (console_motion_is(p_line, "forward"))
    {
        b_ok = motion_control_move_forward((uint32_t)args[0]);
    }
    else if (console_motion_is(p_line, "backward"))
    {
        b_ok = motion_control_move_backward((uint32_t)args[0]);
    }
    else if (console_motion_is(p_line, "left"))
    {
        b_ok = motion_control_turn_left((uint32_t)args[0]);
    }
    else if (console_motion_is(p_line, "right"))
    {
        b_ok = motion_control_turn_right((uint32_t)args[0]);
    }
    else if (console_motion_is(p_line, "drive"))
    {
        b_ok = motion_control_drive(args[0], args[1]);
        if (b_ok)
        {
            /* Accepted speeds are within -1000..1000, so they fit. */
            g_drive_left = (int16_t)args[0];
            g_drive_right = (int16_t)args[1];
        }
    }
    else if (console_motion_is(p_line, "speed"))
    {
        b_ok = motion_control_set_speed((uint32_t)args[0]);
    }
    else if (console_motion_is(p_line, "track"))
    {
        b_ok = motion_control_set_track((uint32_t)args[0]);
    }
    else if (console_motion_is(p_line, "pid"))
    {
        b_ok = motion_control_set_pid((uint32_t)args[0], (uint32_t)args[1],
                                      (uint32_t)args[2]);
    }
    else if (console_motion_is(p_line, "stop"))
    {
        motion_control_stop();
        b_ok = true;
    }
    else if (console_motion_is(p_line, "status"))
    {
        console_motion_report();
        b_ok = true;
    }
    else
    {
        b_ok = false;
    }
    return b_ok;
}

/*** end of file ***/

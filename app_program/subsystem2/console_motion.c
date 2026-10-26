#include "console_motion.h"
#include <string.h>
#include "motion_control.h"
#include "drv_motion.h"

#define CONSOLE_MOTION_TOKEN_BYTES 16u
#define CONSOLE_MOTION_RX_BYTES 128u

static struct
{
    uint8_t bytes[CONSOLE_MOTION_RX_BYTES];
    unsigned int head : 7;
    unsigned int tail : 7;
} g_rx;

/* Called only by the USB service task, never from the hardware IRQ. */
void tm_console_receive_byte(uint8_t byte)
{
    uint32_t saved = drv_motion_lock();
    uint32_t next = (g_rx.head + 1u) % CONSOLE_MOTION_RX_BYTES;
    bool stopped = ((next == g_rx.tail) || ('!' == byte));
    if (stopped)
    {
        /* A stop invalidates older console commands, including queued arm. */
        g_rx.tail = 0u;
        g_rx.head = 1u;
        g_rx.bytes[0] = '!';
    }
    else
    {
        g_rx.bytes[g_rx.head] = byte;
        g_rx.head = next;
    }
    drv_motion_unlock(saved);
    if (stopped)
    {
        motion_control_safety_stop(0u);
    }
}

int32_t console_motion_getchar(void)
{
    uint32_t saved = drv_motion_lock();
    int32_t byte = -1;
    if (g_rx.head != g_rx.tail)
    {
        byte = g_rx.bytes[g_rx.tail];
        g_rx.tail = (g_rx.tail + 1u) % CONSOLE_MOTION_RX_BYTES;
    }
    drv_motion_unlock(saved);
    return byte;
}



/* Decimal parsing rejects trailing tokens and overflow. */
static bool console_motion_number(const char ** pp_text, int32_t * p_value)
{
    const char * p_text = *pp_text;
    int32_t value = 0;
    int32_t sign = 1;
    bool digits = false;

    while (' ' == *p_text)
    {
        p_text++;
    }
    if ('-' == *p_text)
    {
        sign = -1;
        p_text++;
    }
    while ((*p_text >= '0') && (*p_text <= '9'))
    {
        digits = true;
        if (value > 100000)
        {
            return false;
        }
        value = value * 10 + (*p_text - '0');
        p_text++;
    }
    *p_value = value * sign;
    *pp_text = p_text;
    return digits;
}

static bool console_motion_config(const char * p_key, uint32_t value)
{
    motion_control_config_t config = {0};
    bool valid = true;
    motion_control_get_config(&config);
    config.confirmed = 0u;

    if ((0 == strcmp(p_key, "circ")) && (value <= 1000u))
    {
        config.circumference_mm = value;
    }
    else if ((0 == strcmp(p_key, "track")) && (value <= 1000u))
    {
        config.track_mm = value;
    }
    else if ((0 == strcmp(p_key, "cpr")) && (value <= 65535u))
    {
        config.counts_per_rev = value;
    }
    else if ((0 == strcmp(p_key, "kp")) && (value <= 4095u))
    {
        config.kp_q8 = value;
    }
    else if ((0 == strcmp(p_key, "ki")) && (value <= 4095u))
    {
        config.ki_q8 = value;
    }
    else if ((0 == strcmp(p_key, "kd")) && (value <= 4095u))
    {
        config.kd_q8 = value;
    }
    else if ((0 == strcmp(p_key, "limit")) && (value <= 1000u))
    {
        config.duty_limit = value;
    }
    else if ((0 == strcmp(p_key, "timeout")) && (value <= 60000u))
    {
        config.timeout_ms = value;
    }
    else if ((0 == strcmp(p_key, "stall")) && (value <= 10000u))
    {
        config.stall_ms = value;
    }
    else if ((0 == strcmp(p_key, "ml")) && (value <= 1u))
    {
        config.motor_left_reverse = value;
    }
    else if ((0 == strcmp(p_key, "mr")) && (value <= 1u))
    {
        config.motor_right_reverse = value;
    }
    else if ((0 == strcmp(p_key, "el")) && (value <= 1u))
    {
        config.encoder_left_reverse = value;
    }
    else if ((0 == strcmp(p_key, "er")) && (value <= 1u))
    {
        config.encoder_right_reverse = value;
    }
    else if ((0 == strcmp(p_key, "confirm")) && (value <= 1u))
    {
        config.confirmed = value;
    }
    else
    {
        valid = false;
    }
    /* An unconfirmed staging configuration can be completed field by field. */
    return valid && motion_control_configure(&config);
}

bool console_motion_execute(const char * p_line)
{
    char command[CONSOLE_MOTION_TOKEN_BYTES] = {0};
    char key[CONSOLE_MOTION_TOKEN_BYTES] = {0};
    uint32_t index = 0u;
    int32_t first = 0;
    int32_t second = 0;
    int32_t third = 0;
    bool accepted = false;

    while ((' ' != *p_line) && ('\0' != *p_line) &&
           (index < CONSOLE_MOTION_TOKEN_BYTES - 1u))
    {
        command[index] = *p_line;
        index++;
        p_line++;
    }
    if ((0 == strcmp(command, "stop")) && ('\0' == *p_line))
    {
        motion_control_safety_stop(0u);
        return true;
    }
    if ((0 == strcmp(command, "status")) && ('\0' == *p_line))
    {
        console_motion_report();
        return true;
    }
    if ((0 == strcmp(command, "arm")) && ('\0' == *p_line))
    {
        return motion_control_arm();
    }
    if ((0 == strcmp(command, "disarm")) && ('\0' == *p_line))
    {
        motion_control_disarm();
        return true;
    }
    if ((0 == strcmp(command, "reset")) && ('\0' == *p_line))
    {
        return motion_control_reset_faults();
    }
    if (0 == strcmp(command, "config"))
    {
        while (' ' == *p_line)
        {
            p_line++;
        }
        index = 0u;
        while ((' ' != *p_line) && ('\0' != *p_line) &&
           (index < CONSOLE_MOTION_TOKEN_BYTES - 1u))
        {
            key[index] = *p_line;
            index++;
            p_line++;
        }
        if (console_motion_number(&p_line, &first) && (first >= 0) &&
            ('\0' == *p_line))
        {
            accepted = console_motion_config(key, (uint32_t)first);
        }
        return accepted;
    }
    if (!console_motion_number(&p_line, &first))
    {
        return false;
    }
    if (0 == strcmp(command, "motor"))
    {
        if (console_motion_number(&p_line, &second) &&
            console_motion_number(&p_line, &third) &&
            ('\0' == *p_line) && (first >= 0) && (third > 0))
        {
            accepted = motion_control_test_motor((uint32_t)first, second,
                                                   (uint32_t)third);
        }
    }
    else if ((first > 0) && ('\0' == *p_line))
    {
        if (0 == strcmp(command, "forward"))
        {
            accepted = motion_control_move_forward((uint32_t)first);
        }
        else if (0 == strcmp(command, "backward"))
        {
            accepted = motion_control_move_backward((uint32_t)first);
        }
        else if (0 == strcmp(command, "left"))
        {
            accepted = motion_control_turn_left((uint32_t)first);
        }
        else if (0 == strcmp(command, "right"))
        {
            accepted = motion_control_turn_right((uint32_t)first);
        }
        else if (0 == strcmp(command, "speed"))
        {
            accepted = motion_control_set_speed((uint32_t)first);
        }
        else
        {
            accepted = false;
        }
    }
    else
    {
        accepted = false;
    }
    return accepted;
}

/* End of file. */

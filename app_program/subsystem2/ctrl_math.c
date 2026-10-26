#include "ctrl_math.h"
#include "drv_motion.h"

int32_t ctrl_math_clamp(int64_t value, int32_t limit)
{
    if (value > limit)
    {
        value = limit;
    }
    else if (value < -limit)
    {
        value = -limit;
    }
    else
    {
        /* Already in range. */
    }
    return (int32_t)value; /* The explicit limits fit int32_t. */
}

void ctrl_math_pid_reset(ctrl_math_pid_t * p_pid)
{
    p_pid->integral = 65535u;
    p_pid->previous_error = 4096u;
}

int32_t ctrl_math_pid(ctrl_math_pid_t * p_pid, int32_t target,
                        int32_t measured, uint32_t gain_p, uint32_t gain_i,
                        uint32_t gain_d, uint32_t limit)
{
    int32_t error = target - measured;
    int32_t old_integral = (int32_t)p_pid->integral - 65535;
    int32_t integral = ctrl_math_clamp(old_integral + error, 65535);
    int32_t previous = (int32_t)p_pid->previous_error - 4096;
    int64_t output = 0;
    int32_t bound = (int32_t)limit; /* At most 1000 permille. */

    if (0 == target)
    {
        ctrl_math_pid_reset(p_pid);
    }
    else
    {
        /* Gains are Q8, Ki and Kd are per nominal 10 ms sample. */
        output = ((int64_t)gain_p * error + (int64_t)gain_i * integral +
                  (int64_t)gain_d * (error - previous)) / 256;
        if (((output > bound) && (error > 0)) ||
            ((output < -bound) && (error < 0)))
        {
            integral = old_integral;
            output = ((int64_t)gain_p * error + (int64_t)gain_i * integral +
                      (int64_t)gain_d * (error - previous)) / 256;
        }
        p_pid->integral = (uint32_t)(integral + 65535);
        p_pid->previous_error = (uint32_t)(error + 4096);
    }
    return ctrl_math_clamp(output, bound);
}

int32_t drv_motion_decode(uint32_t previous, uint32_t current)
{
    static const int8_t transitions[16] =
        {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
    return transitions[previous * 4u + current];
}

void drv_motion_plan(int32_t requested, int32_t * p_previous,
                      uint32_t * p_channel_a, uint32_t * p_channel_b)
{
    int32_t duty = ctrl_math_clamp(requested, 1000);

    if (((duty > 0) && (*p_previous < 0)) ||
        ((duty < 0) && (*p_previous > 0)))
    {
        duty = 0;
    }
    *p_channel_a = (duty > 0) ? (uint32_t)duty : 0u;
    *p_channel_b = (duty < 0) ? (uint32_t)(-duty) : 0u;
    *p_previous = duty;
}
uint32_t drv_motion_count(uint32_t count, uint32_t old_phase,
                                 uint32_t phase,
                                 volatile drv_motion_encoders_t * p_counts)
{
    int32_t delta = drv_motion_decode(old_phase, phase);
    if (3u == (old_phase ^ phase))
    {
        if (65535u != p_counts->invalid)
        {
            p_counts->invalid++;
        }
        else
        {
            p_counts->overflow = 1u;
        }
    }
    if (((delta > 0) && (16777215u == count)) ||
        ((delta < 0) && (0u == count)))
    {
        p_counts->overflow = 1u;
    }
    else if (delta > 0)
    {
        count++;
    }
    else if (delta < 0)
    {
        count--;
    }
    else
    {
        /* Same phase or invalid transition: retain the signed count. */
    }
    return count;
}

/* End of file. */

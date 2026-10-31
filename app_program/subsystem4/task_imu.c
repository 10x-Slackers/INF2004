/** @file task_imu.c
 *
 * @brief Subsystem 4's IMU task. Owns I2C0 and the GY-511 on it.
 *
 * Every S4_IMU_MS a cyclic handler wakes the task. The first S4_CAL_N
 * readings, taken with the car still, set the offsets that make the start
 * surface read x = y = 0, z = +1000 mg. After that, pitch and roll come
 * from the low-pass filtered reading, in integer maths, with the
 * small-angle rule angle [rad] ~ acceleration [g]: within 0.2 degree up to
 * 15 degrees, reading low beyond about 25 degrees. Sensor axes on the car:
 * -Y forward, +X left, +Z up.
 *
 * Faults, latched until reset: impact when X or Y jumps more than
 * S4_IMPACT_MG from its smoothed value (slow changes such as tilt do not
 * count), tilt when pitch or roll stays beyond S4_TILT_MAX for S4_TILT_MS.
 * Each prints a line and sounds the buzzer.
 *
 * Humps pass under the right wheel only, so its height is
 * track width x sin(roll). A hump starts when that height stays at
 * S4_HUMP_ON_MM or more for S4_HUMP_MIN_MS, which ignores jolts, and ends,
 * with its peak logged, when it falls below S4_HUMP_OFF_MM.
 */

#include "subsystem4.h"
#include "drv_gy511.h"

#define IMU_MG_PER_G      1000
#define IMU_SMOOTH        8   /* Low-pass weight: ~8 samples (80 ms). */
#define IMU_AVERAGE       256 /* Slow average weight: ~2.5 s. */
#define IMU_DECIDEG_PER_G 573 /* Small angles: 1 g ~ 1 rad = 57.3 deg. */
#define IMU_DESCEND_MM    2   /* This far below the peak: descending. */
#define IMU_PRINT_MS      100
#define IMU_PWM_CLOCK_HZ  125000000u /* System clock, divider 1. */

/* Written only by the IMU task, under g_s4_mutex. */
static s4_accel_t g_accel;
static s4_tilt_t g_tilt;
static s4_hump_t g_hump;
static s4_motion_t g_motion;

static const char * const g_motion_name[] = {
    "steady", "accelerating", "braking", "climbing", "descending", "impact"};

static int32_t imu_clamp(int32_t value, int32_t limit);
static bool imu_beyond(int32_t value, int32_t limit);
static bool imu_moved(int32_t value, int32_t sum, int32_t count);
static void imu_beep_on(void);

/*!
 * @brief Copy the latest reading, under g_s4_mutex.
 */
void s4_get_accel(s4_accel_t * p_accel)
{
    (void)tk_loc_mtx(g_s4_mutex, TMO_FEVR);
    *p_accel = g_accel;
    (void)tk_unl_mtx(g_s4_mutex);
}

/*!
 * @brief Copy the latest tilt, under g_s4_mutex.
 */
void s4_get_tilt(s4_tilt_t * p_tilt)
{
    (void)tk_loc_mtx(g_s4_mutex, TMO_FEVR);
    *p_tilt = g_tilt;
    (void)tk_unl_mtx(g_s4_mutex);
}

/*!
 * @brief Copy the latest hump record, under g_s4_mutex.
 */
void s4_get_hump(s4_hump_t * p_hump)
{
    (void)tk_loc_mtx(g_s4_mutex, TMO_FEVR);
    *p_hump = g_hump;
    (void)tk_unl_mtx(g_s4_mutex);
}

/*!
 * @brief Copy the latest motion class, under g_s4_mutex.
 */
void s4_get_motion(s4_motion_t * p_motion)
{
    (void)tk_loc_mtx(g_s4_mutex, TMO_FEVR);
    *p_motion = g_motion;
    (void)tk_unl_mtx(g_s4_mutex);
}

/*!
 * @brief IMU task: calibrate, then one sample per wake-up.
 */
void s4_imu_task(INT stacd, void * p_exinf)
{
    int32_t x_mg = 0;
    int32_t y_mg = 0;
    int32_t z_mg = 0;
    int32_t cal_count = 0;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    int32_t offset_x = 0;
    int32_t offset_y = 0;
    int32_t offset_z = 0;
    int32_t smooth_x = 0; /* Low-pass x and y, 8 x mg. */
    int32_t smooth_y = 0;
    int32_t pitch = 0; /* 0.1 degree. */
    int32_t roll = 0;
    int32_t tilted_count = 0; /* Samples in a row beyond S4_TILT_MAX. */
    int32_t height_mm = 0;    /* Right wheel height. */
    int32_t peak_mm = 0;      /* Highest point of the current hump. */
    int32_t high_count = 0;   /* Samples in a row at S4_HUMP_ON_MM. */
    int32_t avg_y = 0;        /* Slow average of y, 256 x mg. */
    int32_t push_mg = 0;      /* Forward push against that average. */
    int32_t pushed_count = 0; /* Samples in a row beyond S4_ACCEL_MG. */
    s4_motion_class_t push_class = S4_STEADY;
    s4_motion_class_t motion_class = S4_STEADY;
    bool b_ready = true; /* Last read worked: print the next failure. */
    bool b_jolt = false;
    bool b_new_impact = false;
    bool b_new_tilt = false;
    uint32_t faults = 0u; /* APP_FAULT_IMU bits latched; only this task. */
    bool b_hump_over = false;
    bool b_descending = false; /* Past the top of the current hump. */
    bool b_changed = false;
    int32_t print_count = 0;
    int32_t beep_left = 0; /* Samples until the buzzer stops. */
    ER result = E_OK;

    (void)stacd;
    (void)p_exinf;

    app_check(drv_gy511_init(), "drv_gy511_init");

    app_check(app_start_cyclic(tk_get_tid(), S4_IMU_MS), "tk_cre_cyc");

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        result = drv_gy511_read(&x_mg, &y_mg, &z_mg);
        if (b_ready)
        {
            app_check(result, "drv_gy511_read"); /* Not 100 a second. */
        }
        b_ready = (result >= E_OK);

        if (b_ready && (cal_count < S4_CAL_N))
        {
            if ((imu_moved(x_mg, sum_x, cal_count)) ||
                (imu_moved(y_mg, sum_y, cal_count)) ||
                (imu_moved(z_mg, sum_z, cal_count)))
            {
                /* Car moved: start again. */
                cal_count = 0;
                sum_x = 0;
                sum_y = 0;
                sum_z = 0;
            }
            sum_x += x_mg;
            sum_y += y_mg;
            sum_z += z_mg;
            cal_count++;
            if (S4_CAL_N == cal_count)
            {
                offset_x = sum_x / cal_count;
                offset_y = sum_y / cal_count;
                offset_z = (sum_z / cal_count) - IMU_MG_PER_G;
                /* (UB *): tm_printf() takes unsigned chars. */
                tm_printf((UB *)"[imu] calibrated: offsets x=%d y=%d z=%d mg\n",
                          offset_x, offset_y, offset_z);
                imu_beep_on();
                beep_left = S4_BEEP_MS / S4_IMU_MS;
            }
        }
        else if (b_ready)
        {
            if (1 == beep_left)
            {
                (void)pwm_set_enabled(S4_BUZZER_PIN, FALSE);
            }
            if (beep_left > 0)
            {
                beep_left--;
            }

            x_mg -= offset_x;
            y_mg -= offset_y;
            z_mg -= offset_z;
            b_jolt =
                (imu_beyond(x_mg - (smooth_x / IMU_SMOOTH), S4_IMPACT_MG)) ||
                (imu_beyond(y_mg - (smooth_y / IMU_SMOOTH), S4_IMPACT_MG));
            smooth_x += x_mg - (smooth_x / IMU_SMOOTH);
            smooth_y += y_mg - (smooth_y / IMU_SMOOTH);

            /* Forward push against the slow average, so a lasting tilt does
             * not count. Forward is -Y: speeding up pushes y negative.
             * Switch after S4_ACCEL_MS beyond S4_ACCEL_MG, back below half
             * of it. */
            push_mg = (smooth_y / IMU_SMOOTH) - (avg_y / IMU_AVERAGE);
            avg_y += y_mg - (avg_y / IMU_AVERAGE);
            if (!imu_beyond(push_mg, S4_ACCEL_MG / 2))
            {
                pushed_count = 0;
                push_class = S4_STEADY;
            }
            else if (imu_beyond(push_mg, S4_ACCEL_MG))
            {
                pushed_count++;
                if (pushed_count >= (S4_ACCEL_MS / S4_IMU_MS))
                {
                    push_class = (push_mg < 0) ? S4_ACCELERATING : S4_BRAKING;
                }
            }
            else
            {
                /* Between half and all of S4_ACCEL_MG: keep the class. */
            }

            pitch = imu_clamp((-smooth_y * IMU_DECIDEG_PER_G) /
                                  (IMU_SMOOTH * IMU_MG_PER_G),
                              S4_ANGLE_MAX);
            roll = imu_clamp((smooth_x * IMU_DECIDEG_PER_G) /
                                 (IMU_SMOOTH * IMU_MG_PER_G),
                             S4_ANGLE_MAX);
            tilted_count = ((imu_beyond(pitch, S4_TILT_MAX)) ||
                            (imu_beyond(roll, S4_TILT_MAX)))
                               ? (tilted_count + 1)
                               : 0;

            /* Only the right wheel crosses humps, so the car tips sideways
             * about the line from the left wheel to the caster. */
            height_mm = (-S4_TRACK_MM * smooth_x) / (IMU_SMOOTH * IMU_MG_PER_G);
            high_count = (height_mm >= S4_HUMP_ON_MM) ? (high_count + 1) : 0;
            if (((high_count >= (S4_HUMP_MIN_MS / S4_IMU_MS)) ||
                 (peak_mm > 0)) &&
                (height_mm > peak_mm))
            {
                peak_mm = height_mm; /* On a hump: track its highest point. */
            }
            b_hump_over = (peak_mm > 0) && (height_mm < S4_HUMP_OFF_MM);

            (void)tk_loc_mtx(g_s4_mutex, TMO_FEVR);
            g_accel.x = imu_clamp(x_mg, S4_AXIS_MAX_MG);
            g_accel.y = imu_clamp(y_mg, S4_AXIS_MAX_MG);
            g_accel.z = imu_clamp(z_mg, S4_AXIS_MAX_MG);
            g_accel.count++;
            g_tilt.pitch = pitch;
            g_tilt.roll = roll;
            g_tilt.cal = 1u;
            b_new_impact = b_jolt && (0u == (faults & APP_FAULT_IMPACT));
            b_new_tilt = (tilted_count >= (S4_TILT_MS / S4_IMU_MS)) &&
                         (0u == (faults & APP_FAULT_TILT));
            faults |= (b_new_impact ? APP_FAULT_IMPACT : 0u) |
                      (b_new_tilt ? APP_FAULT_TILT : 0u); /* Latched. */
            g_hump.on = (peak_mm > 0) && (!b_hump_over);
            if (b_hump_over)
            {
                g_hump.peak = imu_clamp(peak_mm, S4_HUMP_MAX_MM);
                g_hump.max =
                    (g_hump.peak > g_hump.max) ? g_hump.peak : g_hump.max;
                g_hump.count++;
            }

            /* On a hump the car also tips nose down, so humps take
             * precedence. A hump goes up once and down once: once
             * descending, stay so. */
            b_descending =
                (g_hump.on) &&
                (b_descending || (height_mm <= (peak_mm - IMU_DESCEND_MM)));
            motion_class = push_class;
            if (0u != (faults & APP_FAULT_IMPACT))
            {
                motion_class = S4_IMPACT;
            }
            else if (g_hump.on)
            {
                motion_class = b_descending ? S4_DESCENDING : S4_CLIMBING;
            }
            else
            {
                /* No impact or hump: the push class stands. */
            }
            b_changed = (motion_class != g_motion.cls);
            g_motion.cls = motion_class;
            (void)tk_unl_mtx(g_s4_mutex);

            if (b_changed)
            {
                /* (UB *): tm_printf() takes unsigned chars. */
                tm_printf((UB *)"[imu] motion: %s\n",
                          g_motion_name[motion_class]);
            }

            if (b_new_impact || b_new_tilt)
            {
                app_fault_set(faults);
                /* (UB *): tm_printf() takes unsigned chars. */
                tm_printf((UB *)"[imu] FAULT: %s\n",
                          b_new_impact ? "impact" : "tilt");
                imu_beep_on();
                beep_left = S4_ALARM_MS / S4_IMU_MS;
            }

            if (b_hump_over)
            {
                /* (UB *): tm_printf() takes unsigned chars. */
                tm_printf((UB *)"[imu] hump: peak %d mm\n", peak_mm);
                peak_mm = 0;
            }

            /* TEST=1 only: it would bury the [vc] and [safety] lines. */
            print_count = (print_count + 1) % (IMU_PRINT_MS / S4_IMU_MS);
            if (APP_TEST && (0 == print_count))
            {
                /* (UB *): tm_printf() takes unsigned chars. */
                tm_printf((UB *)"[imu] x=%d y=%d z=%d mg pitch=%d roll=%d "
                                "(0.1 deg) h=%d mm\n",
                          x_mg, y_mg, z_mg, pitch, roll, height_mm);
            }
        }
        else
        {
            /* The read failed: skip this sample. */
        }
    }
}

/*!
 * @brief Limit value to -limit..limit.
 */
static int32_t imu_clamp(int32_t value, int32_t limit)
{
    int32_t clamped = value;

    if (value > limit)
    {
        clamped = limit;
    }
    else if (value < -limit)
    {
        clamped = -limit;
    }
    else
    {
        /* Already in range. */
    }
    return clamped;
}

/*!
 * @brief True if value lies outside -limit..limit.
 */
static bool imu_beyond(int32_t value, int32_t limit)
{
    return (value > limit) || (value < -limit);
}

/*!
 * @brief True if value is more than S4_CAL_MG from the mean so far.
 */
static bool imu_moved(int32_t value, int32_t sum, int32_t count)
{
    return (count > 0) && (imu_beyond(value - (sum / count), S4_CAL_MG));
}

/*!
 * @brief Start a square wave on the buzzer.
 */
static void imu_beep_on(void)
{
    (void)pwm_set_pin(S4_BUZZER_PIN);
    (void)pwm_set_wrap(S4_BUZZER_PIN, (IMU_PWM_CLOCK_HZ / S4_BEEP_HZ) - 1u);
    (void)pwm_set_cc(S4_BUZZER_PIN, (IMU_PWM_CLOCK_HZ / S4_BEEP_HZ) / 2u);
    (void)pwm_set_enabled(S4_BUZZER_PIN, TRUE);
}

/*** end of file ***/

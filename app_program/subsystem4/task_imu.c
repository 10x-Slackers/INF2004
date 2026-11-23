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
 * Faults, latched until reset: tilt when pitch or roll stays beyond
 * S4_TILT_MAX for S4_TILT_MS, and upside down when Z stays below 0 mg (over
 * 90 degrees from the start surface) for S4_UPSIDE_MS. Lifting the car and
 * turning it over ends a demo run; pitch and roll cannot see that, as they
 * read near 0 again once the car is fully inverted. Lifted is not latched:
 * it is set once Z stays over 1 g by S4_LIFT_MG for S4_LIFT_MS, which only
 * a quick lift gives, and clears once Z settles and the 1 s beep ends, so a
 * car carried to the start needs one GP20 press to clear the stop. Each
 * fault prints a line and sounds the buzzer; the Safety Monitor then stops
 * the motors.
 *
 * Sensor fault, latched until reset too: S4_FAIL_MS of failed reads in a
 * row, or S4_FROZEN_MS of readings with no axis changed (a working GY-511
 * moves by a few mg even at rest). Checked from power-on, and no reading
 * is used once it is set, so a car reset with the sensor still out or
 * frozen faults again and never calibrates.
 *
 * Humps pass under the right wheel only, so its height is
 * track width x sin(roll). A hump starts when that height stays at
 * S4_HUMP_ON_MM or more for S4_HUMP_MIN_MS, which ignores jolts, and ends,
 * with its peak logged, when it falls below S4_HUMP_OFF_MM. None are tracked
 * while an IMU fault is set: the car is being handled, not driven.
 */

#include "subsystem4.h"
#include "drv_gy511.h"

#define IMU_MG_PER_G      1000
#define IMU_SMOOTH        8   /* Low-pass weight: ~8 samples (80 ms). */
#define IMU_AVERAGE       256 /* Slow average weight: ~2.5 s. */
#define IMU_DECIDEG_PER_G 573 /* Small angles: 1 g ~ 1 rad = 57.3 deg. */
#define IMU_DESCEND_MM    2   /* This far below the peak: descending. */
#define IMU_PRINT_MS      100
#define IMU_COUNT_MAX     63u
#define IMU_PWM_CLOCK_HZ  125000000u /* System clock, divider 1. */

#if ((S4_TILT_MS / S4_IMU_MS) >= IMU_COUNT_MAX) ||                             \
    ((S4_UPSIDE_MS / S4_IMU_MS) >= IMU_COUNT_MAX) ||                           \
    ((S4_HUMP_MIN_MS / S4_IMU_MS) >= IMU_COUNT_MAX) ||                         \
    ((S4_ACCEL_MS / S4_IMU_MS) >= IMU_COUNT_MAX) ||                            \
    ((S4_LIFT_MS / S4_IMU_MS) >= IMU_COUNT_MAX) ||                             \
    ((S4_FAIL_MS / S4_IMU_MS) >= IMU_COUNT_MAX) ||                             \
    ((S4_FROZEN_MS / S4_IMU_MS) >= IMU_COUNT_MAX)
#error "A samples-in-a-row threshold needs more than 6 bits"
#endif

/* Written under gh_s4_mutex. */
static s4_accel_t g_accel;
static s4_tilt_t g_tilt;
static s4_hump_t g_hump;
static s4_motion_t g_motion;

/* Kept between samples. IMU task only. Each field is sized to its range
 * (docs/DATA_REPRESENTATION.md): readings are -2048..2047 mg, so a sample
 * less its offset is -4095..4095 mg. */
static struct
{
    unsigned int cal_count : 8; /* Calibration samples so far, 0..200. */
    signed int sum_x : 20;      /* Up to 200 readings: +-409 600 mg. */
    signed int sum_y : 20;
    signed int sum_z : 20;
    signed int offset_x : 12; /* -2048..2047 mg. */
    signed int offset_y : 12;
    signed int offset_z : 13; /* Less 1 g: -3048..1047 mg. */
    signed int smooth_x : 17; /* Low-pass x and y, 8 x mg: +-32 767. */
    signed int smooth_y : 17;
    signed int avg_y : 22; /* Slow average of y, 256 x mg: +-1 048 575. */
    unsigned int pushed_count : 6; /* Samples in a row beyond S4_ACCEL_MG, */
    unsigned int tilted_count : 6; /* beyond S4_TILT_MAX, */
    unsigned int upside_count : 6; /* with Z below 0 mg, */
    unsigned int lift_count : 6;   /* over S4_LIFT_MG, */
    unsigned int high_count : 6;   /* at S4_HUMP_ON_MM; imu_count() caps. */
    unsigned int peak_mm : 9;      /* Top of the current hump, 0..466 mm. */
    unsigned int print_count : 4;  /* 0..9. */
    unsigned int beep_left : 7;    /* Samples until the buzzer stops. */
    unsigned int faults : 12;    /* Latched APP_FAULT_IMU bits (not lifted). */
    unsigned int push_class : 3; /* s4_motion_class_t. */
    unsigned int b_descending : 1; /* Past the top of the current hump. */
    unsigned int fail_count : 6;   /* Reads in a row that failed, */
    unsigned int same_count : 6;   /* that matched the one before. */
    signed int last_x : 12;        /* The reading before: -2048..2047 mg. */
    signed int last_y : 12;
    signed int last_z : 12;
} g_imu;

/* One calibrated sample, passed from imu_sample() to imu_publish(). */
typedef struct
{
    int32_t x_mg;
    int32_t y_mg;
    int32_t z_mg;
    int32_t pitch; /* 0.1 degree. */
    int32_t roll;
    int32_t height_mm; /* Right wheel height. */
    bool b_hump_over;
} imu_sample_t;

static const char * const g_motion_name[] = {
    "steady", "accelerating", "braking", "climbing", "descending"};

static void imu_step(bool b_ready, int32_t x_mg, int32_t y_mg, int32_t z_mg);
static void imu_calibrate(int32_t x_mg, int32_t y_mg, int32_t z_mg);
static void imu_sample(int32_t x_mg, int32_t y_mg, int32_t z_mg);
static void imu_push(int32_t y_mg);
static void imu_publish(const imu_sample_t * p_sample);
static int32_t imu_clamp(int32_t value, int32_t limit);
static bool imu_beyond(int32_t value, int32_t limit);
static bool imu_moved(int32_t value, int32_t sum, int32_t count);
static uint32_t imu_count(uint32_t count, bool b_holds);
static void imu_beep_on(void);

/*!
 * @brief Copy the latest reading, under gh_s4_mutex.
 */
void s4_get_accel(s4_accel_t * p_accel)
{
    (void)tk_loc_mtx(gh_s4_mutex, TMO_FEVR);
    *p_accel = g_accel;
    (void)tk_unl_mtx(gh_s4_mutex);
}

/*!
 * @brief Copy the latest tilt, under gh_s4_mutex.
 */
void s4_get_tilt(s4_tilt_t * p_tilt)
{
    (void)tk_loc_mtx(gh_s4_mutex, TMO_FEVR);
    *p_tilt = g_tilt;
    (void)tk_unl_mtx(gh_s4_mutex);
}

/*!
 * @brief Copy the latest hump record, under gh_s4_mutex.
 */
void s4_get_hump(s4_hump_t * p_hump)
{
    (void)tk_loc_mtx(gh_s4_mutex, TMO_FEVR);
    *p_hump = g_hump;
    (void)tk_unl_mtx(gh_s4_mutex);
}

/*!
 * @brief Clear the highest hump, under gh_s4_mutex.
 */
void s4_reset_hump_max(void)
{
    (void)tk_loc_mtx(gh_s4_mutex, TMO_FEVR);
    g_hump.max = 0u;
    (void)tk_unl_mtx(gh_s4_mutex);
}

/*!
 * @brief Copy the latest motion class, under gh_s4_mutex.
 */
void s4_get_motion(s4_motion_t * p_motion)
{
    (void)tk_loc_mtx(gh_s4_mutex, TMO_FEVR);
    *p_motion = g_motion;
    (void)tk_unl_mtx(gh_s4_mutex);
}

/*!
 * @brief IMU task: one read per wake-up.
 */
void s4_imu_task(INT stacd, void * p_exinf)
{
    int32_t x_mg = 0;
    int32_t y_mg = 0;
    int32_t z_mg = 0;
    bool b_ready = true; /* Last read worked: print the next failure. */
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
        imu_step(b_ready, x_mg, y_mg, z_mg);
    }
}

/*!
 * @brief One wake-up: time the beep, latch a sensor fault if the reads keep
 * failing or the reading has stopped changing, else calibrate or sample.
 *
 * @param[in] b_ready The read worked; if not, x, y and z are ignored.
 */
static void imu_step(bool b_ready, int32_t x_mg, int32_t y_mg, int32_t z_mg)
{
    bool const b_same = b_ready && (x_mg == g_imu.last_x) &&
                        (y_mg == g_imu.last_y) && (z_mg == g_imu.last_z);

    if (1 == g_imu.beep_left)
    {
        (void)pwm_set_enabled(S4_BUZZER_PIN, FALSE);
    }
    if (g_imu.beep_left > 0)
    {
        g_imu.beep_left--;
    }

    g_imu.fail_count = imu_count(g_imu.fail_count, !b_ready);
    g_imu.same_count = imu_count(g_imu.same_count, b_same);
    if (((g_imu.fail_count >= (S4_FAIL_MS / S4_IMU_MS)) ||
         (g_imu.same_count >= (S4_FROZEN_MS / S4_IMU_MS))) &&
        (0u == (g_imu.faults & APP_FAULT_IMU_SENSOR)))
    {
        g_imu.faults |= APP_FAULT_IMU_SENSOR; /* Latched. */
        app_fault_set(APP_FAULT_IMU_SENSOR);
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[imu] FAULT: sensor %s\n",
                  b_ready ? "frozen" : "not answering");
        imu_beep_on();
        g_imu.beep_left = S4_ALARM_MS / S4_IMU_MS;
    }

    /* A failed read skips the sample; a sensor fault ends them. */
    if (b_ready && (0u == (g_imu.faults & APP_FAULT_IMU_SENSOR)))
    {
        g_imu.last_x = x_mg;
        g_imu.last_y = y_mg;
        g_imu.last_z = z_mg;
        if (g_imu.cal_count < S4_CAL_N)
        {
            imu_calibrate(x_mg, y_mg, z_mg);
        }
        else
        {
            imu_sample(x_mg, y_mg, z_mg);
        }
    }
}

/*!
 * @brief Add one start-up sample; restart if the car moved. After
 * S4_CAL_N samples, set the offsets and beep.
 */
static void imu_calibrate(int32_t x_mg, int32_t y_mg, int32_t z_mg)
{
    if ((imu_moved(x_mg, g_imu.sum_x, g_imu.cal_count)) ||
        (imu_moved(y_mg, g_imu.sum_y, g_imu.cal_count)) ||
        (imu_moved(z_mg, g_imu.sum_z, g_imu.cal_count)))
    {
        /* Car moved: start again. */
        g_imu.cal_count = 0;
        g_imu.sum_x = 0;
        g_imu.sum_y = 0;
        g_imu.sum_z = 0;
    }
    g_imu.sum_x += x_mg;
    g_imu.sum_y += y_mg;
    g_imu.sum_z += z_mg;
    g_imu.cal_count++;
    if (S4_CAL_N == g_imu.cal_count)
    {
        g_imu.offset_x = g_imu.sum_x / g_imu.cal_count;
        g_imu.offset_y = g_imu.sum_y / g_imu.cal_count;
        g_imu.offset_z = (g_imu.sum_z / g_imu.cal_count) - IMU_MG_PER_G;
        /* (UB *): tm_printf() takes unsigned chars. */
        tm_printf((UB *)"[imu] calibrated: offsets x=%d y=%d z=%d mg\n",
                  g_imu.offset_x, g_imu.offset_y, g_imu.offset_z);
        imu_beep_on();
        g_imu.beep_left = S4_BEEP_MS / S4_IMU_MS;
    }
}

/*!
 * @brief One calibrated sample: filter it, then work out tilt and humps.
 */
static void imu_sample(int32_t x_mg, int32_t y_mg, int32_t z_mg)
{
    imu_sample_t sample = {0};

    sample.x_mg = x_mg - g_imu.offset_x;
    sample.y_mg = y_mg - g_imu.offset_y;
    sample.z_mg = z_mg - g_imu.offset_z;
    g_imu.upside_count = imu_count(g_imu.upside_count, (sample.z_mg < 0));
    g_imu.lift_count = imu_count(g_imu.lift_count,
                                 (sample.z_mg > (IMU_MG_PER_G + S4_LIFT_MG)));
    g_imu.smooth_x += sample.x_mg - (g_imu.smooth_x / IMU_SMOOTH);
    g_imu.smooth_y += sample.y_mg - (g_imu.smooth_y / IMU_SMOOTH);
    imu_push(sample.y_mg);

    sample.pitch = imu_clamp((-g_imu.smooth_y * IMU_DECIDEG_PER_G) /
                                 (IMU_SMOOTH * IMU_MG_PER_G),
                             S4_ANGLE_MAX);
    sample.roll = imu_clamp((g_imu.smooth_x * IMU_DECIDEG_PER_G) /
                                (IMU_SMOOTH * IMU_MG_PER_G),
                            S4_ANGLE_MAX);
    g_imu.tilted_count =
        imu_count(g_imu.tilted_count, (imu_beyond(sample.pitch, S4_TILT_MAX) ||
                                       imu_beyond(sample.roll, S4_TILT_MAX)));

    /* Only the right wheel crosses humps, so the car tips sideways about
     * the line from the left wheel to the caster. */
    sample.height_mm =
        (-S4_TRACK_MM * g_imu.smooth_x) / (IMU_SMOOTH * IMU_MG_PER_G);
    g_imu.high_count =
        imu_count(g_imu.high_count, (sample.height_mm >= S4_HUMP_ON_MM));
    if (0u != (app_faults() & APP_FAULT_IMU))
    {
        g_imu.high_count = 0; /* Picked up or turned over: not a hump. */
        g_imu.peak_mm = 0;
    }
    if (((g_imu.high_count >= (S4_HUMP_MIN_MS / S4_IMU_MS)) ||
         (g_imu.peak_mm > 0)) &&
        (sample.height_mm > g_imu.peak_mm))
    {
        g_imu.peak_mm = (uint32_t)sample.height_mm; /* On a hump: its top. */
    }
    sample.b_hump_over =
        (g_imu.peak_mm > 0) && (sample.height_mm < S4_HUMP_OFF_MM);
    imu_publish(&sample);
}

/*!
 * @brief Forward push against the slow average of y, so a lasting tilt
 * does not count. Forward is -Y: speeding up pushes y negative. Switch
 * after S4_ACCEL_MS beyond S4_ACCEL_MG, back below half of it.
 */
static void imu_push(int32_t y_mg)
{
    int32_t const push_mg =
        (g_imu.smooth_y / IMU_SMOOTH) - (g_imu.avg_y / IMU_AVERAGE);

    g_imu.avg_y += y_mg - (g_imu.avg_y / IMU_AVERAGE);
    if (!imu_beyond(push_mg, S4_ACCEL_MG / 2))
    {
        g_imu.pushed_count = 0;
        g_imu.push_class = S4_STEADY;
    }
    else if (imu_beyond(push_mg, S4_ACCEL_MG))
    {
        g_imu.pushed_count = imu_count(g_imu.pushed_count, true);
        if (g_imu.pushed_count >= (S4_ACCEL_MS / S4_IMU_MS))
        {
            g_imu.push_class = (push_mg < 0) ? S4_ACCELERATING : S4_BRAKING;
        }
    }
    else
    {
        /* Between half and all of S4_ACCEL_MG: keep the class. */
    }
}

/*!
 * @brief Publish a sample, latch new faults, classify the motion, then
 * print and beep what changed.
 */
static void imu_publish(const imu_sample_t * p_sample)
{
    s4_motion_class_t motion_class = (s4_motion_class_t)g_imu.push_class;
    bool b_new_upside = false;
    bool b_new_tilt = false;
    bool b_changed = false;

    (void)tk_loc_mtx(gh_s4_mutex, TMO_FEVR);
    g_accel.x = imu_clamp(p_sample->x_mg, S4_AXIS_MAX_MG);
    g_accel.y = imu_clamp(p_sample->y_mg, S4_AXIS_MAX_MG);
    g_accel.z = imu_clamp(p_sample->z_mg, S4_AXIS_MAX_MG);
    g_accel.count++;
    g_tilt.pitch = p_sample->pitch;
    g_tilt.roll = p_sample->roll;
    g_tilt.cal = 1u;
    b_new_upside = (g_imu.upside_count >= (S4_UPSIDE_MS / S4_IMU_MS)) &&
                   (0u == (g_imu.faults & APP_FAULT_UPSIDE_DOWN));
    b_new_tilt = (g_imu.tilted_count >= (S4_TILT_MS / S4_IMU_MS)) &&
                 (0u == (g_imu.faults & APP_FAULT_TILT));
    g_imu.faults |= (b_new_upside ? APP_FAULT_UPSIDE_DOWN : 0u) |
                    (b_new_tilt ? APP_FAULT_TILT : 0u); /* Latched. */
    g_hump.on = (g_imu.peak_mm > 0) && (!p_sample->b_hump_over);
    if (p_sample->b_hump_over)
    {
        g_hump.peak = imu_clamp(g_imu.peak_mm, S4_HUMP_MAX_MM);
        g_hump.max = (g_hump.peak > g_hump.max) ? g_hump.peak : g_hump.max;
        g_hump.count++;
    }

    /* On a hump the car also tips nose down, so humps take precedence. A
     * hump goes up once and down once: once descending, stay so. */
    g_imu.b_descending =
        (g_hump.on) &&
        ((g_imu.b_descending) ||
         (p_sample->height_mm <= (g_imu.peak_mm - IMU_DESCEND_MM)));
    if (g_hump.on)
    {
        motion_class = g_imu.b_descending ? S4_DESCENDING : S4_CLIMBING;
    }
    else
    {
        /* No hump: the push class stands. */
    }
    b_changed = (motion_class != g_motion.cls);
    g_motion.cls = motion_class;
    (void)tk_unl_mtx(gh_s4_mutex);

    /* (UB *): tm_printf() takes unsigned chars, in every call below. */
    if (b_changed)
    {
        tm_printf((UB *)"[imu] motion: %s\n", g_motion_name[motion_class]);
    }
    if (g_imu.lift_count >= (S4_LIFT_MS / S4_IMU_MS))
    {
        app_fault_set(APP_FAULT_LIFTED);
    }
    else if (0 == g_imu.beep_left)
    {
        app_fault_clear(APP_FAULT_LIFTED); /* Beep over: GP20 can clear. */
    }
    else
    {
        /* Held while it beeps, so the Safety Monitor cannot miss it. */
    }
    if (b_new_upside || b_new_tilt ||
        ((S4_LIFT_MS / S4_IMU_MS) == g_imu.lift_count))
    {
        app_fault_set(g_imu.faults);
        tm_printf((UB *)"[imu] FAULT: %s\n",
                  b_new_upside ? "upside down"
                               : (b_new_tilt ? "tilt" : "lifted"));
        imu_beep_on();
        g_imu.beep_left = S4_ALARM_MS / S4_IMU_MS;
    }
    if (p_sample->b_hump_over)
    {
        tm_printf((UB *)"[imu] hump: peak %d mm\n", g_imu.peak_mm);
        g_imu.peak_mm = 0;
    }

    /* TEST=1 only: it would bury the [vc] and [safety] lines. */
    g_imu.print_count = (g_imu.print_count + 1) % (IMU_PRINT_MS / S4_IMU_MS);
    if (APP_TEST && (0 == g_imu.print_count))
    {
        tm_printf((UB *)"[imu] x=%d y=%d z=%d mg pitch=%d roll=%d "
                        "(0.1 deg) h=%d mm\n",
                  p_sample->x_mg, p_sample->y_mg, p_sample->z_mg,
                  p_sample->pitch, p_sample->roll, p_sample->height_mm);
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
 * @brief Samples in a row: count + 1 while b_holds, else 0. Stops at
 * IMU_COUNT_MAX, above every threshold, so the 6-bit fields never wrap.
 */
static uint32_t imu_count(uint32_t count, bool b_holds)
{
    return b_holds ? ((count < IMU_COUNT_MAX) ? (count + 1u) : count) : 0u;
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

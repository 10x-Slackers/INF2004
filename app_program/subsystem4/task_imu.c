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
 * Sensor fault: S4_FAIL_MS of failed reads in a row, one read that timed
 * out, or S4_FROZEN_MS of readings with no axis changed (a working GY-511
 * moves by a few mg even at rest). It stays until a Reset calls
 * s4_reset_sensor(); the task then starts again as at power-on, so with
 * the sensor still out or frozen it faults again and never calibrates.
 *
 * Humps pass under the right wheel only, so its height is
 * track width x sin(roll). A hump starts when that height stays at
 * S4_HUMP_ON_MM or more for S4_HUMP_MIN_MS, which ignores jolts, and ends,
 * with its peak logged, when it falls below S4_HUMP_OFF_MM. None are tracked
 * while an IMU fault is set: the car is being handled, not driven.
 */

#include "task_imu.h"
#include "drv_gy511.h"

#define IMU_MG_PER_G      1000
#define IMU_SMOOTH        8   /* Low-pass weight: ~8 samples (80 ms). */
#define IMU_AVERAGE       256 /* Slow average weight: ~2.5 s. */
#define IMU_DECIDEG_PER_G 573 /* Small angles: 1 g ~ 1 rad = 57.3 deg. */
#define IMU_DESCEND_MM    2   /* This far below the peak: descending. */
#define IMU_PRINT_MS      100u
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

/* Kept between samples. IMU task only. Each field is sized to its range
 * (docs/DATA_REPRESENTATION.md): readings are -2048..2047 mg, so a sample
 * less its offset is -4095..4095 mg. */
typedef struct
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
    bool b_descending : 1;       /* Past the top of the current hump. */
    unsigned int fail_count : 6; /* Reads in a row that failed, */
    unsigned int same_count : 6; /* that matched the one before. */
    signed int last_x : 12;      /* The reading before: -2048..2047 mg. */
    signed int last_y : 12;
    signed int last_z : 12;
} imu_state_t;

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

/* Written under gh_s4_mutex. */
static s4_accel_t g_accel = {0};
static s4_tilt_t g_tilt = {0};
static s4_hump_t g_hump = {0};
static s4_motion_t g_motion = {0};

/* s4_reset_sensor() asks the IMU task to start again. */
static volatile bool gb_reset = false;

static imu_state_t g_imu = {0};

static const char * const g_motion_name[] = {
    "steady", "accelerating", "braking", "climbing", "descending"};

static void imu_step(void);
static void imu_read(void);
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
 * @brief On a Reset: if the sensor fault is set, clear it and have the IMU
 * task start again. Not calibrated from now, so the controller waits.
 */
void s4_reset_sensor(void)
{
    if (0u != (app_faults() & APP_FAULT_IMU_SENSOR))
    {
        (void)tk_loc_mtx(gh_s4_mutex, TMO_FEVR);
        g_tilt = (s4_tilt_t){0};
        (void)tk_unl_mtx(gh_s4_mutex);
        /* In this order: the task leaves the bit alone until it sees the
         * request, so it cannot set it again between the two. Lifted too:
         * no sample is taken to clear it while the sensor fault is set. */
        app_fault_clear(APP_FAULT_IMU_SENSOR | APP_FAULT_LIFTED);
        gb_reset = true;
    }
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
    (void)stacd;
    (void)p_exinf;

    app_check(drv_gy511_init(), "drv_gy511_init");

    app_check(app_start_cyclic(tk_get_tid(), S4_IMU_MS), "tk_cre_cyc");

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        imu_step();
    }
}

/*!
 * @brief One wake-up: start again if asked, time the beep, then take a
 * reading unless the sensor fault is set.
 */
static void imu_step(void)
{
    if (gb_reset)
    {
        /* As at power-on, but tilt and upside down stay latched. */
        gb_reset = false;
        g_imu = (imu_state_t){
            .faults = app_faults() & (APP_FAULT_UPSIDE_DOWN | APP_FAULT_TILT)};
        (void)pwm_set_enabled(S4_BUZZER_PIN, FALSE);
        app_check(drv_gy511_start(), "drv_gy511_start");
    }

    if (1u == g_imu.beep_left)
    {
        (void)pwm_set_enabled(S4_BUZZER_PIN, FALSE);
    }
    if (g_imu.beep_left > 0u)
    {
        g_imu.beep_left--;
    }

    /* No read once the fault is set: on a hung bus each one takes 1 s. */
    if (0u == (g_imu.faults & APP_FAULT_IMU_SENSOR))
    {
        imu_read();
    }
}

/*!
 * @brief Read the sensor, then latch a sensor fault, calibrate or sample.
 */
static void imu_read(void)
{
    int32_t x_mg = 0;
    int32_t y_mg = 0;
    int32_t z_mg = 0;
    ER const result = drv_gy511_read(&x_mg, &y_mg, &z_mg);
    bool const b_ready = (result >= E_OK);
    bool const b_same = b_ready && (x_mg == g_imu.last_x) &&
                        (y_mg == g_imu.last_y) && (z_mg == g_imu.last_z);

    if (0u == g_imu.fail_count)
    {
        app_check(result, "drv_gy511_read"); /* Not 100 a second. */
    }
    g_imu.fail_count = imu_count(g_imu.fail_count, !b_ready);
    g_imu.same_count = imu_count(g_imu.same_count, b_same);
    if ((g_imu.fail_count >= (S4_FAIL_MS / S4_IMU_MS)) || (E_TMOUT == result) ||
        (g_imu.same_count >= (S4_FROZEN_MS / S4_IMU_MS)))
    {
        g_imu.faults |= APP_FAULT_IMU_SENSOR; /* Until a Reset. */
        app_fault_set(APP_FAULT_IMU_SENSOR);
        APP_PRINTF("[imu] FAULT: sensor %s\n",
                   b_ready ? "frozen" : "not answering");
        imu_beep_on();
        g_imu.beep_left = S4_ALARM_MS / S4_IMU_MS;
        /* A read that timed out let 1 s of wake-ups queue: drop them, or
         * they would run the beep down at once. */
        (void)tk_can_wup(TSK_SELF);
    }
    else if (b_ready)
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
    else
    {
        /* The read failed: skip this sample. */
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
        g_imu.cal_count = 0u;
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
        /* (int32_t): the count is S4_CAL_N here; a signed division. */
        g_imu.offset_x = g_imu.sum_x / (int32_t)g_imu.cal_count;
        g_imu.offset_y = g_imu.sum_y / (int32_t)g_imu.cal_count;
        g_imu.offset_z =
            (g_imu.sum_z / (int32_t)g_imu.cal_count) - IMU_MG_PER_G;
        APP_PRINTF("[imu] calibrated: offsets x=%d y=%d z=%d mg\n",
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
    g_imu.tilted_count = imu_count(g_imu.tilted_count,
                                   ((imu_beyond(sample.pitch, S4_TILT_MAX)) ||
                                    (imu_beyond(sample.roll, S4_TILT_MAX))));

    /* Only the right wheel crosses humps, so the car tips sideways about
     * the line from the left wheel to the caster. */
    sample.height_mm =
        (-S4_TRACK_MM * g_imu.smooth_x) / (IMU_SMOOTH * IMU_MG_PER_G);
    g_imu.high_count =
        imu_count(g_imu.high_count, (sample.height_mm >= S4_HUMP_ON_MM));
    if (0u != (app_faults() & APP_FAULT_IMU))
    {
        g_imu.high_count = 0u; /* Picked up or turned over: not a hump. */
        g_imu.peak_mm = 0u;
    }
    if (((g_imu.high_count >= (S4_HUMP_MIN_MS / S4_IMU_MS)) ||
         (g_imu.peak_mm > 0u)) &&
        (sample.height_mm > (int32_t)g_imu.peak_mm)) /* 9 bits: fits. */
    {
        /* On a hump: its top. (uint32_t): above the old peak, so positive. */
        g_imu.peak_mm = (uint32_t)sample.height_mm;
    }
    sample.b_hump_over =
        (g_imu.peak_mm > 0u) && (sample.height_mm < S4_HUMP_OFF_MM);
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
        g_imu.pushed_count = 0u;
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
    /* push_class only ever holds an s4_motion_class_t. */
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
    g_tilt.b_cal = true;
    b_new_upside = (g_imu.upside_count >= (S4_UPSIDE_MS / S4_IMU_MS)) &&
                   (0u == (g_imu.faults & APP_FAULT_UPSIDE_DOWN));
    b_new_tilt = (g_imu.tilted_count >= (S4_TILT_MS / S4_IMU_MS)) &&
                 (0u == (g_imu.faults & APP_FAULT_TILT));
    g_imu.faults |= (b_new_upside ? APP_FAULT_UPSIDE_DOWN : 0u) |
                    (b_new_tilt ? APP_FAULT_TILT : 0u); /* Latched. */
    g_hump.b_on = (g_imu.peak_mm > 0u) && (!p_sample->b_hump_over);
    if (p_sample->b_hump_over)
    {
        g_hump.peak = imu_clamp(g_imu.peak_mm, S4_HUMP_MAX_MM);
        g_hump.max = (g_hump.peak > g_hump.max) ? g_hump.peak : g_hump.max;
        g_hump.count++;
    }

    /* On a hump the car also tips nose down, so humps take precedence. A
     * hump goes up once and down once: once descending, stay so.
     * (int32_t): peak_mm is 9 bits, and the difference may be negative. */
    g_imu.b_descending =
        (g_hump.b_on) &&
        ((g_imu.b_descending) ||
         (p_sample->height_mm <= ((int32_t)g_imu.peak_mm - IMU_DESCEND_MM)));
    if (g_hump.b_on)
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

    if (b_changed)
    {
        APP_PRINTF("[imu] motion: %s\n", g_motion_name[motion_class]);
    }
    if (g_imu.lift_count >= (S4_LIFT_MS / S4_IMU_MS))
    {
        app_fault_set(APP_FAULT_LIFTED);
    }
    else if (0u == g_imu.beep_left)
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
        APP_PRINTF("[imu] FAULT: %s\n", b_new_upside
                                            ? "upside down"
                                            : (b_new_tilt ? "tilt" : "lifted"));
        imu_beep_on();
        g_imu.beep_left = S4_ALARM_MS / S4_IMU_MS;
    }
    if (p_sample->b_hump_over)
    {
        APP_PRINTF("[imu] hump: peak %d mm\n", g_imu.peak_mm);
        g_imu.peak_mm = 0u;
    }

    /* TEST=1 only: it would bury the [vc] and [safety] lines. */
    g_imu.print_count = (g_imu.print_count + 1u) % (IMU_PRINT_MS / S4_IMU_MS);
    if (APP_TEST && (0u == g_imu.print_count))
    {
        APP_PRINTF("[imu] x=%d y=%d z=%d mg pitch=%d roll=%d "
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

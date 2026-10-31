/*
 *      task_imu.c
 *      Subsystem 4 -- the IMU task.  Owns I2C0 and the GY-511 on it.
 *
 *      Every S4_IMU_MS a cyclic handler wakes the task.  The first S4_CAL_N
 *      readings, taken with the car still, set the offsets that make the
 *      start surface read x = y = 0, z = +1000 mg.  After that, pitch and
 *      roll come from the low-pass filtered reading, in integer maths, with
 *      the small-angle rule angle [rad] ~ acceleration [g]: within 0.2
 *      degree up to 15 degrees, reading low beyond about 25 degrees.
 *      Sensor axes on the car: -Y forward, +X left, +Z up.
 *
 *      Faults, latched until reset: impact when X or Y jumps more than
 *      S4_IMPACT_MG from its smoothed value (slow changes such as tilt do
 *      not count), tilt when pitch or roll stays beyond S4_TILT_MAX for
 *      S4_TILT_MS.  Each prints a line and sounds the buzzer.
 *
 *      Humps pass under the right wheel only, so its height is
 *      track width x sin(roll).  A hump starts when that height stays at
 *      S4_HUMP_ON_MM or more for S4_HUMP_MIN_MS, which ignores jolts, and
 *      ends, with its peak logged, when it falls below S4_HUMP_OFF_MM.
 */

#include "subsystem4.h"
#include "drv_gy511.h"

LOCAL S4_ACCEL accel; /* written only by imu_task, under s4_mtx_accel */
LOCAL S4_TILT tilt;   /* likewise */
LOCAL S4_FAULT fault; /* likewise */
LOCAL S4_HUMP hump;   /* likewise */

EXPORT void s4_get_accel(S4_ACCEL * out)
{
    tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
    *out = accel;
    tk_unl_mtx(s4_mtx_accel);
}

EXPORT void s4_get_fault(S4_FAULT * out)
{
    tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
    *out = fault;
    tk_unl_mtx(s4_mtx_accel);
}

EXPORT void s4_get_hump(S4_HUMP * out)
{
    tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
    *out = hump;
    tk_unl_mtx(s4_mtx_accel);
}

EXPORT void s4_get_tilt(S4_TILT * out)
{
    tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
    *out = tilt;
    tk_unl_mtx(s4_mtx_accel);
}

LOCAL W clamp(W v, W lim)
{
    return (v > lim) ? lim : (v < -lim) ? -lim : v;
}

LOCAL BOOL beyond(W v, W lim)
{
    return (v > lim) || (v < -lim);
}

/* TRUE if v is more than S4_CAL_MG from the mean so far, sum / n */
LOCAL BOOL moved(W v, W sum, W n)
{
    return (n > 0) && beyond(v - sum / n, S4_CAL_MG);
}

/* Start a square wave on the buzzer (PWM clock 125 MHz, divider 1) */
LOCAL void beep_on(void)
{
    pwm_set_pin(S4_BUZZER_PIN);
    pwm_set_wrap(S4_BUZZER_PIN, 125000000 / S4_BEEP_HZ - 1);
    pwm_set_cc(S4_BUZZER_PIN, 125000000 / S4_BEEP_HZ / 2);
    pwm_set_enabled(S4_BUZZER_PIN, TRUE);
}

LOCAL void imu_tick(void * exinf)
{
    tk_wup_tsk((ID)exinf);
}

EXPORT void imu_task(INT stacd, void * exinf)
{
    T_CCYC ccyc = {0};
    W x, y, z;
    W n = 0, sx = 0, sy = 0, sz = 0; /* calibration count and sums */
    W ox = 0, oy = 0, oz = 0;        /* offsets, in mg */
    W fx = 0, fy = 0;                /* filtered x and y, 8 x mg */
    W pitch, roll;                   /* 0.1 degree */
    W tilted = 0;                    /* samples in a row beyond S4_TILT_MAX */
    BOOL jolt, new_impact, new_tilt;
    W h, peak = 0; /* right wheel height, hump peak, mm */
    W high = 0;    /* samples in a row at S4_HUMP_ON_MM */
    BOOL done;     /* a hump just ended */
    UB print = 0;
    UB beep = 0; /* samples left until the buzzer stops */
    ER er;

    (void)stacd;
    (void)exinf;

    CHECK(gy511_init(), "gy511_init");

    ccyc.exinf = (void *)tk_get_tid();
    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)imu_tick;
    ccyc.cyctim = S4_IMU_MS;
    CHECK(tk_cre_cyc(&ccyc), "tk_cre_cyc");

    while (1)
    {
        tk_slp_tsk(TMO_FEVR);
        er = gy511_read(&x, &y, &z);
        CHECK(er, "gy511_read");
        if (er < E_OK)
        {
            continue;
        }

        if (n < S4_CAL_N)
        {
            if (moved(x, sx, n) || moved(y, sy, n) || moved(z, sz, n))
            {
                n = sx = sy = sz = 0; /* car moved: start again */
            }
            sx += x;
            sy += y;
            sz += z;
            if (++n == S4_CAL_N)
            {
                ox = sx / n;
                oy = sy / n;
                oz = sz / n - 1000;
                tm_printf((UB *)"[imu] calibrated: offsets x=%d y=%d z=%d mg\n",
                          ox, oy, oz);
                beep_on();
                beep = S4_BEEP_MS / S4_IMU_MS;
            }
            continue;
        }

        if (beep > 0 && --beep == 0)
        {
            pwm_set_enabled(S4_BUZZER_PIN, FALSE);
        }

        x -= ox;
        y -= oy;
        z -= oz;
        jolt = beyond(x - fx / 8, S4_IMPACT_MG) ||
               beyond(y - fy / 8, S4_IMPACT_MG); /* far from the smoothed */
        fx += x - fx / 8; /* low-pass, time constant ~8 samples (80 ms) */
        fy += y - fy / 8;
        pitch = clamp(-fy * 573 / 8000, 900); /* 0.1 deg per mg */
        roll = clamp(fx * 573 / 8000, 900);
        tilted = (beyond(pitch, S4_TILT_MAX) || beyond(roll, S4_TILT_MAX))
                     ? tilted + 1
                     : 0;

        /* Only the right wheel crosses humps, so the car tips sideways
           about the line from the left wheel to the caster */
        h = -S4_TRACK_MM * fx / 8000; /* right wheel height, mm */
        high = (h >= S4_HUMP_ON_MM) ? high + 1 : 0;
        if ((high >= S4_HUMP_MIN_MS / S4_IMU_MS || peak > 0) && h > peak)
        {
            peak = h; /* on a hump: track its highest point */
        }
        done = (peak > 0) && (h < S4_HUMP_OFF_MM); /* back on flat ground */

        tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
        accel.x = clamp(x, 2047);
        accel.y = clamp(y, 2047);
        accel.z = clamp(z, 2047);
        accel.count++;
        tilt.pitch = pitch;
        tilt.roll = roll;
        tilt.cal = 1;
        new_impact = jolt && !fault.impact;
        new_tilt = (tilted >= S4_TILT_MS / S4_IMU_MS) && !fault.tilt;
        fault.impact |= new_impact; /* latched until reset */
        fault.tilt |= new_tilt;
        hump.on = (peak > 0) && !done;
        if (done)
        {
            hump.peak = clamp(peak, 63);
            hump.max = (hump.peak > hump.max) ? hump.peak : hump.max;
            hump.count++;
        }
        tk_unl_mtx(s4_mtx_accel);

        if (new_impact || new_tilt)
        {
            tm_printf((UB *)"[imu] FAULT: %s\n",
                      new_impact ? "impact" : "tilt");
            beep_on();
            beep = S4_ALARM_MS / S4_IMU_MS;
        }

        if (done)
        {
            tm_printf((UB *)"[imu] hump: peak %d mm\n", peak);
            peak = 0;
        }
        if (++print == 100 / S4_IMU_MS)
        {
            print = 0;
            tm_printf((UB *)"[imu] x=%d y=%d z=%d mg pitch=%d roll=%d (0.1 "
                            "deg) h=%d mm\n",
                      x, y, z, (W)tilt.pitch, (W)tilt.roll, h);
        }
    }
}

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
 */

#include "subsystem4.h"
#include "drv_gy511.h"

LOCAL S4_ACCEL accel; /* written only by imu_task, under s4_mtx_accel */
LOCAL S4_TILT tilt;   /* likewise */

EXPORT void s4_get_accel(S4_ACCEL * out)
{
    tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
    *out = accel;
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

/* TRUE if v is more than S4_CAL_MG from the mean so far, sum / n */
LOCAL BOOL moved(W v, W sum, W n)
{
    W d = (n > 0) ? v - sum / n : 0;
    return (d > S4_CAL_MG) || (d < -S4_CAL_MG);
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
    UB print = 0;
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
            }
            continue;
        }

        x -= ox;
        y -= oy;
        z -= oz;
        fx += x - fx / 8; /* low-pass, time constant ~8 samples (80 ms) */
        fy += y - fy / 8;

        tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
        accel.x = clamp(x, 2047);
        accel.y = clamp(y, 2047);
        accel.z = clamp(z, 2047);
        accel.count++;
        tilt.pitch = clamp(-fy * 573 / 8000, 900); /* 0.1 deg per mg */
        tilt.roll = clamp(fx * 573 / 8000, 900);
        tilt.cal = 1;
        tk_unl_mtx(s4_mtx_accel);

        if (++print == 100 / S4_IMU_MS)
        {
            print = 0;
            tm_printf(
                (UB *)"[imu] x=%d y=%d z=%d mg pitch=%d roll=%d (0.1 deg)\n", x,
                y, z, (W)tilt.pitch, (W)tilt.roll);
        }
    }
}

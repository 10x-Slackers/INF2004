/*
 *      task_imu.c
 *      Subsystem 4 -- the IMU task.  Owns I2C0 and the GY-511 on it.
 */

#include "subsystem4.h"
#include "drv_gy511.h"

LOCAL S4_ACCEL accel; /* written only by imu_task, under s4_mtx_accel */

EXPORT void s4_get_accel(S4_ACCEL * out)
{
    tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
    *out = accel;
    tk_unl_mtx(s4_mtx_accel);
}

EXPORT void imu_task(INT stacd, void * exinf)
{
    W x, y, z;
    ER er;

    (void)stacd;
    (void)exinf;

    CHECK(gy511_init(), "gy511_init");

    while (1)
    {
        er = gy511_read(&x, &y, &z);
        CHECK(er, "gy511_read");
        if (er == E_OK)
        {
            tk_loc_mtx(s4_mtx_accel, TMO_FEVR);
            accel.x = x;
            accel.y = y;
            accel.z = z;
            accel.count++;
            tk_unl_mtx(s4_mtx_accel);
            tm_printf((UB *)"[imu] x=%d y=%d z=%d mg\n", x, y, z);
        }
        tk_dly_tsk(S4_IMU_MS);
    }
}

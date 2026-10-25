/*
 *      task_imu.c
 *      Subsystem 4 -- the IMU task.  Owns I2C0 and the GY-511 on it.
 */

#include "subsystem4.h"
#include "drv_gy511.h"

EXPORT S4_ACCEL s4_accel;
EXPORT UW s4_reads_ok;

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
            s4_accel.x = x;
            s4_accel.y = y;
            s4_accel.z = z;
            s4_reads_ok++;
            tm_printf((UB *)"[imu] x=%d y=%d z=%d mg\n", x, y, z);
        }
        tk_dly_tsk(S4_IMU_MS);
    }
}

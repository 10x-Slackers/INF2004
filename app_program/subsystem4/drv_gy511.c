/*
 *      drv_gy511.c
 *      GY-511 (LSM303DLHC) accelerometer driver.  Register details are in
 *      docs/HARDWARE.md.
 */

#include "drv_gy511.h"
#include "../../device/include/dev_i2c.h"

#define ACC_ADDR    0x19 /* 7-bit accelerometer address */
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define OUT_XYZ_A   0xA8 /* OUT_X_L_A with the auto-increment bit set */

LOCAL ID dd_i2c;

EXPORT ER gy511_init(void)
{
    ER er;

    dd_i2c = tk_opn_dev((UB *)"iica", TD_UPDATE);
    if (dd_i2c < E_OK)
    {
        return dd_i2c;
    }
    er = i2c_write_reg(dd_i2c, ACC_ADDR, CTRL_REG1_A, 0x57); /* 100 Hz, XYZ */
    if (er < E_OK)
    {
        return er;
    }
    return i2c_write_reg(dd_i2c, ACC_ADDR, CTRL_REG4_A, 0x08); /* +-2 g, HR */
}

EXPORT ER gy511_read(W * x, W * y, W * z)
{
    UB reg = OUT_XYZ_A;
    UB raw[6];
    SZ asz;
    T_I2C_EXEC exec = {ACC_ADDR, 1, &reg, sizeof(raw), raw};
    ER er;

    /* Write the register address, then read X, Y, Z (low byte first) */
    er = tk_swri_dev(dd_i2c, TDN_I2C_EXEC, &exec, sizeof(exec), &asz);
    if (er < E_OK)
    {
        return er;
    }

    /* 12-bit values, left-justified: 1 mg per LSB at +-2 g */
    *x = (H)(raw[0] | (raw[1] << 8)) >> 4;
    *y = (H)(raw[2] | (raw[3] << 8)) >> 4;
    *z = (H)(raw[4] | (raw[5] << 8)) >> 4;
    return E_OK;
}

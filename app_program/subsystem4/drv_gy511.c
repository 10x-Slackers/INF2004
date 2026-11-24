/** @file drv_gy511.c
 *
 * @brief GY-511 (LSM303DLHC) accelerometer driver. Register details are in
 * docs/HARDWARE.md.
 */

#include "drv_gy511.h"
#include "../../device/include/dev_i2c.h"

#define DRV_GY511_ADDR       0x19u /* 7-bit accelerometer address. */
#define DRV_GY511_CTRL_REG1  0x20u
#define DRV_GY511_CTRL_REG4  0x23u
#define DRV_GY511_OUT_XYZ    0xA8u /* OUT_X_L_A with auto-increment set. */
#define DRV_GY511_RATE_100HZ 0x57u /* REG1: 100 Hz, X, Y and Z on. */
#define DRV_GY511_RANGE_2G   0x08u /* REG4: +-2 g, high resolution. */
#define DRV_GY511_OUT_BYTES  6u    /* X, Y, Z, low byte first. */
#define DRV_GY511_BYTE_BITS  8u
#define DRV_GY511_JUSTIFY    16     /* 12 bits left-justified in 16. */
#define DRV_GY511_SIGN       32768u /* Bit 15, the sign of a 16-bit value. */

static ID gh_i2c = 0;

static int32_t drv_gy511_axis(const uint8_t * p_raw);

/*!
 * @brief Open I2C0 and start the accelerometer.
 */
ER drv_gy511_init(void)
{
    gh_i2c = tk_opn_dev((UB *)"iica", TD_UPDATE); /* Kernel takes UB names. */
    return (gh_i2c < E_OK) ? gh_i2c : drv_gy511_start();
}

/*!
 * @brief Start the accelerometer: its data rate and range. Also needed
 * after it lost power, which leaves it in power-down.
 */
ER drv_gy511_start(void)
{
    ER result = i2c_write_reg(gh_i2c, DRV_GY511_ADDR, DRV_GY511_CTRL_REG1,
                              DRV_GY511_RATE_100HZ);

    if (result >= E_OK)
    {
        result = i2c_write_reg(gh_i2c, DRV_GY511_ADDR, DRV_GY511_CTRL_REG4,
                               DRV_GY511_RANGE_2G);
    }
    return result;
}

/*!
 * @brief Read the acceleration of each axis, 1 mg per unit at +-2 g.
 */
ER drv_gy511_read(int32_t * p_x_mg, int32_t * p_y_mg, int32_t * p_z_mg)
{
    uint8_t reg = DRV_GY511_OUT_XYZ;
    uint8_t raw[DRV_GY511_OUT_BYTES] = {0};
    SZ size = 0;
    T_I2C_EXEC exec = {DRV_GY511_ADDR, 1, &reg, sizeof(raw), raw};
    ER result = E_OK;

    /* Write the register address, then read X, Y and Z. */
    result = tk_swri_dev(gh_i2c, TDN_I2C_EXEC, &exec, sizeof(exec), &size);
    if (result >= E_OK)
    {
        *p_x_mg = drv_gy511_axis(&raw[0]);
        *p_y_mg = drv_gy511_axis(&raw[2]);
        *p_z_mg = drv_gy511_axis(&raw[4]);
        result = E_OK;
    }
    return result;
}

/*!
 * @brief One axis from its two bytes, low byte first. The low 4 bits are
 * zero, so the division is exact.
 */
static int32_t drv_gy511_axis(const uint8_t * p_raw)
{
    /* Widening casts keep each byte's value: bits is 0..65535. */
    uint32_t const bits =
        (uint32_t)p_raw[0] | ((uint32_t)p_raw[1] << DRV_GY511_BYTE_BITS);

    /* Two's complement without an implementation-defined cast: flipping
     * bit 15 gives 0..65535, which fits int32_t, then -32768..32767. */
    return ((int32_t)(bits ^ DRV_GY511_SIGN) - (int32_t)DRV_GY511_SIGN) /
           DRV_GY511_JUSTIFY;
}

/*** end of file ***/

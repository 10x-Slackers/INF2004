
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include "ctrl_math.h"
#include "drv_motion.h"
#define PWM_CH4_BASE 0x4000u
#define PWM_CH5_BASE 0x5000u
#define PWM_CHx_CC 0xcu
#define DRV_MOTION_TOP 6249u
#ifndef S2_DIAGNOSTIC
#define S2_DIAGNOSTIC 0
#endif
static struct { unsigned int left:11; unsigned int right:11; } g_previous={1000u,1000u};
static uint32_t left_cc,right_cc;
static void out_w(uint32_t address,uint32_t value)
{
    if(address==0x400cu) left_cc=value;
    else { assert(address==0x500cu);right_cc=value; }
}
#include "driver_output.inc"

int main(void)
{
    drv_motion_write(100,-150);assert(left_cc==625u&&right_cc==(937u<<16u));
    drv_motion_write(-100,150);assert(left_cc==0u&&right_cc==0u);
    drv_motion_write(-100,150);assert(left_cc==(625u<<16u)&&right_cc==937u);
    drv_motion_write(0,0);assert(left_cc==0u&&right_cc==0u);
    puts("PASS motor-test endpoint: PWM mapping, zero shutdown and braked reversal unchanged");
    return 0;
}

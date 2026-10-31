#ifndef DRV_HCSR04_H
#define DRV_HCSR04_H
#include "subsystem5.h"
ER s5_hcsr04_init(void);
S5_SAMPLE s5_hcsr04_read(unsigned int angle, unsigned int * faults);
#endif

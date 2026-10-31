#ifndef S5_MOCK_SYSDEF_H
#define S5_MOCK_SYSDEF_H
#include <stdint.h>
#define XOSC_MHz           12
#define RESETS_RESET       0x4000c000U
#define RESETS_RESET_DONE  0x4000c008U
#define RESETS_RESET_TIMER 0x00200000U
#define TIMER_TIMERAWL     0x40054028U
#define TIMER_PAUSE        0x40054030U
#define GPIO_MODE_OUT      1
#define GPIO_MODE_IN       0
uint32_t in_w(uint32_t address);
void out_w(uint32_t address, uint32_t value);
int gpio_set_pin(unsigned int pin, unsigned int mode);
int gpio_set_val(unsigned int pin, unsigned int value);
unsigned int gpio_get_val(unsigned int pin);
#endif

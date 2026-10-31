#ifndef S5_MOCK_SYSDEF_H
#define S5_MOCK_SYSDEF_H
#include <stdint.h>
#define TIMER_TIMERAWL     0x40054028U
#define GPIO_MODE_OUT      1
#define GPIO_MODE_IN       0
uint32_t in_w(uint32_t address);
int gpio_set_pin(unsigned int pin, unsigned int mode);
int gpio_set_val(unsigned int pin, unsigned int value);
unsigned int gpio_get_val(unsigned int pin);
#endif

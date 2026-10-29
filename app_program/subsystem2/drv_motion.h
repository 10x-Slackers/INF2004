/* Hardware boundary, also implemented by the host test fixture. */
#ifndef DRV_MOTION_H
#define DRV_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#ifndef S2_DIAGNOSTIC
#define S2_DIAGNOSTIC 0
#endif

/* Permanent Robo Pico connections: left Grove4, right Grove2. */
#define DRV_MOTION_LEFT_A_PIN 16u
#define DRV_MOTION_LEFT_B_PIN 17u
#define DRV_MOTION_RIGHT_A_PIN 2u
#define DRV_MOTION_RIGHT_B_PIN 3u

typedef struct
{
    unsigned int left : 24;
    unsigned int left_phase : 2;
    unsigned int right : 24;
    unsigned int right_phase : 2;
    unsigned int invalid : 16;
    unsigned int overflow : 1;
    unsigned int left_invalid : 16;
    unsigned int right_invalid : 16;
} drv_motion_encoders_t;

bool drv_motion_init(void);
uint32_t drv_motion_lock(void);
void drv_motion_unlock(uint32_t saved);
void drv_motion_snapshot(drv_motion_encoders_t * p_counts);
/* Counts and timer read with interrupts masked under encoder lock 4. */
uint32_t drv_motion_sample(drv_motion_encoders_t * p_counts);
void drv_motion_write(int32_t left, int32_t right);
uint32_t drv_motion_time_us(void);
/* Shared core-0 IRQ13 dispatcher. Register on core 0 before task start. */
typedef void (*drv_motion_gpio_callback_t)(uint32_t pins);
bool drv_motion_gpio_register(uint32_t pins,
                              drv_motion_gpio_callback_t p_callback_isr);
/* Testable quadrature decoder: states are (A * 2 + B), 0..3. */
int32_t drv_motion_decode(uint32_t previous, uint32_t current);
uint32_t drv_motion_count(uint32_t count, uint32_t old_phase,
                          uint32_t phase,
                          volatile drv_motion_encoders_t * p_counts);
/* IRQ-owned update; caller holds the encoder lock. Phases are 0..3. */
void drv_motion_capture(volatile drv_motion_encoders_t * p_counts,
                         uint32_t left_phase, uint32_t right_phase);
/* Testable output planner; reversal inserts one complete braked cycle. */
void drv_motion_plan(int32_t requested, int32_t * p_previous,
                      uint32_t * p_channel_a, uint32_t * p_channel_b);

#endif /* DRV_MOTION_H */
/* End of file. */

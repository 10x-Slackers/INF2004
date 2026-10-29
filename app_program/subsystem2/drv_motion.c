/* RP2040 registers follow datasheet sections 2.3, 2.19 and 4.5. */
#include "drv_motion.h"
#include "motion_control.h"
#include "../app.h"
#include <tk/syslib.h>
#include <tk/smp.h>

#define DRV_MOTION_IRQ 13u
#define DRV_MOTION_IRQ_PRIORITY 2u
#define DRV_MOTION_GPIO_BANKS 4u
#define DRV_MOTION_PINS_PER_BANK 8u
#define DRV_MOTION_PIN_COUNT 30u
#define DRV_MOTION_MOTOR_FIRST 8u
#define DRV_MOTION_MOTOR_LAST 11u
#define DRV_MOTION_GPIO_INTR 0x400140f0u
#define DRV_MOTION_GPIO_ENABLE 0x40014100u
#define DRV_MOTION_GPIO_STATUS 0x40014120u
#define DRV_MOTION_ENCODER_MASK 0x0003000cu
#define DRV_MOTION_TOP 6249u
#define DRV_MOTION_STATE_LOCK 0xd000010cu
#define DRV_MOTION_ENCODER_LOCK 0xd0000110u

static volatile drv_motion_encoders_t g_counts;
static struct
{
    unsigned int left : 11; /* Signed duty + 1000. */
    unsigned int right : 11;
} g_previous;
static drv_motion_gpio_callback_t g_p_callback_isr = NULL;
static struct
{
    unsigned int pins : 30;
    unsigned int started : 1;
} g_callback;

/* Hardware locks 3/4 are reserved in TEAM.md; kernel uses 0..2. */
static uint32_t drv_motion_enter(uint32_t address)
{
    uint32_t saved = disint();
    while (0u == in_w(address))
    {
        /* Both owners execute only bounded code while holding the lock. */
    }
    __asm__ volatile ("dmb" ::: "memory");
    return saved;
}

static void drv_motion_leave(uint32_t address, uint32_t saved)
{
    __asm__ volatile ("dmb" ::: "memory");
    out_w(address, 0u);
    set_primask(saved);
}

uint32_t drv_motion_lock(void)
{
    T_RSYS system = {0};
    tk_ref_sys(&system);
    tk_dis_dsp();
    while (0u == in_w(DRV_MOTION_STATE_LOCK))
    {
        /* ISR13 never takes this task-only lock. */
    }
    __asm__ volatile ("dmb" ::: "memory");
    return system.sysstat & TSS_DDSP;
}

void drv_motion_unlock(uint32_t saved)
{
    __asm__ volatile ("dmb" ::: "memory");
    out_w(DRV_MOTION_STATE_LOCK, 0u);
    if (0u == saved)
    {
        tk_ena_dsp();
    }
}

uint32_t drv_motion_time_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

uint32_t drv_motion_sample(drv_motion_encoders_t * p_counts)
{
    uint32_t saved = drv_motion_enter(DRV_MOTION_ENCODER_LOCK);
    uint32_t sampled_us = drv_motion_time_us();
    *p_counts = g_counts;
    drv_motion_leave(DRV_MOTION_ENCODER_LOCK, saved);
    return sampled_us;
}

void drv_motion_snapshot(drv_motion_encoders_t * p_counts)
{
    (void)drv_motion_sample(p_counts);
}

/* GPIO edge flags are latches, not a queue. Sample before acknowledging.
 * Read both A/B bits from one GPIO_IN word; never infer missing edges.
 * RP2040 datasheet 2.19: INTR is W1C; PROC0_INTS is enabled status.
 */
static void drv_motion_gpio_isr(UINT intno)
{
    uint32_t bank = 0u;
    uint32_t pin = 0u;
    uint32_t pending = 0u;
    uint32_t pins = 0u;
    uint32_t left_levels = 0u;
    uint32_t right_levels = 0u;
    uint32_t saved = 0u;
    uint32_t left = 0u;
    uint32_t right = 0u;
    (void)intno;

    for (bank = 0u; bank < DRV_MOTION_GPIO_BANKS; bank++)
    {
        pending = in_w(DRV_MOTION_GPIO_STATUS + bank * 4u);
        if ((DRV_MOTION_LEFT_A_PIN / 8u) == bank)
        {
            left_levels = in_w(GPIO_IN);
        }
        else if ((DRV_MOTION_RIGHT_A_PIN / 8u) == bank)
        {
            right_levels = in_w(GPIO_IN);
        }
        else
        {
            /* This bank belongs to the optional other GPIO owner. */
        }
        for (pin = 0u; pin < DRV_MOTION_PINS_PER_BANK; pin++)
        {
            if (0u != (pending & (12u << (pin * 4u))))
            {
                pins |= 1u << (bank * 8u + pin);
            }
        }
        out_w(DRV_MOTION_GPIO_INTR + bank * 4u, pending & 0xccccccccu);
    }
    if (0u != (pins & DRV_MOTION_ENCODER_MASK))
    {
        left = (((left_levels >> DRV_MOTION_LEFT_A_PIN) & 1u) * 2u) +
               ((left_levels >> DRV_MOTION_LEFT_B_PIN) & 1u);
        right = (((right_levels >> DRV_MOTION_RIGHT_A_PIN) & 1u) * 2u) +
                ((right_levels >> DRV_MOTION_RIGHT_B_PIN) & 1u);
        saved = drv_motion_enter(DRV_MOTION_ENCODER_LOCK);
        drv_motion_capture(&g_counts, left, right);
        drv_motion_leave(DRV_MOTION_ENCODER_LOCK, saved);
    }
    if ((NULL != g_p_callback_isr) && (0u != (pins & g_callback.pins)))
    {
        g_p_callback_isr(pins & g_callback.pins);
    }
}

static void drv_motion_enable_edges(uint32_t pins)
{
    uint32_t bank = 0u;
    uint32_t pin = 0u;
    uint32_t mask = 0u;
    for (bank = 0u; bank < DRV_MOTION_GPIO_BANKS; bank++)
    {
        mask = 0u;
        for (pin = 0u; pin < DRV_MOTION_PINS_PER_BANK; pin++)
        {
            if (0u != (pins & (1u << (bank * 8u + pin))))
            {
                mask |= 12u << (pin * 4u);
            }
        }
        out_w(DRV_MOTION_GPIO_INTR + bank * 4u, mask);
        out_w(DRV_MOTION_GPIO_ENABLE + 0x2000u + bank * 4u, mask);
    }
}

bool drv_motion_gpio_register(uint32_t pins,
                              drv_motion_gpio_callback_t p_callback_isr)
{
    uint32_t saved = disint();
    bool owner_core = true;
#if TK_SUPPORT_SMP
    owner_core = (1 == tk_get_prc());
#endif
    bool accepted = (owner_core &&
                     (NULL == g_p_callback_isr) &&
                     (NULL != p_callback_isr) && (0u != pins) &&
                     (0u == (pins & (DRV_MOTION_ENCODER_MASK | 0xc0000000u))));
    if (accepted)
    {
        g_callback.pins = pins;
        g_p_callback_isr = p_callback_isr;
        if (0u != g_callback.started)
        {
            drv_motion_enable_edges(pins);
        }
    }
    set_primask(saved);
    return accepted;
}

bool drv_motion_init(void)
{
    uint32_t pin = 0u;
    uint32_t levels = 0u;
    T_DINT definition = {0};
    ER result = E_OK;

    /* Release PWM reset without disturbing the other slices (servo on 6). */
    out_w(RESETS_BASE + 0x3000u, RESETS_RESET_PWM);
    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* Wait for PWM hardware before configuring its registers. */
    }
    for (pin = DRV_MOTION_MOTOR_FIRST;
         pin <= DRV_MOTION_MOTOR_LAST; pin++)
    {
        gpio_set_val(pin, 0u);
        gpio_set_pin(pin, GPIO_MODE_OUT);
    }
    out_w(PWM_CH4_BASE + PWM_CHx_CSR, 0u);
    out_w(PWM_CH5_BASE + PWM_CHx_CSR, 0u);
    out_w(PWM_CH4_BASE + PWM_CHx_CC, 0u);
    out_w(PWM_CH5_BASE + PWM_CHx_CC, 0u);
    out_w(PWM_CH4_BASE + PWM_CHx_DIV, 16u);
    out_w(PWM_CH5_BASE + PWM_CHx_DIV, 16u);
    out_w(PWM_CH4_BASE + PWM_CHx_TOP, DRV_MOTION_TOP);
    out_w(PWM_CH5_BASE + PWM_CHx_TOP, DRV_MOTION_TOP);
    out_w(PWM_CH4_BASE + PWM_CHx_CTR, 0u);
    out_w(PWM_CH5_BASE + PWM_CHx_CTR, 0u);
    for (pin = DRV_MOTION_MOTOR_FIRST;
         pin <= DRV_MOTION_MOTOR_LAST; pin++)
    {
        pwm_set_pin(pin);
    }
    pwm_set_enabled(8u, TRUE);
    pwm_set_enabled(10u, TRUE);
    g_previous.left = 1000u;
    g_previous.right = 1000u;
    g_counts.left = MOTION_CONTROL_COUNT_BIAS;
    g_counts.right = MOTION_CONTROL_COUNT_BIAS;
    g_counts.invalid = 0u;
    g_counts.left_invalid = 0u;
    g_counts.right_invalid = 0u;
    g_counts.overflow = 0u;
    for (pin = 0u; pin < DRV_MOTION_PIN_COUNT; pin++)
    {
        if (0u != (DRV_MOTION_ENCODER_MASK & (1u << pin)))
        {
            gpio_set_pin(pin, GPIO_MODE_IN);
        }
    }
    levels = in_w(GPIO_IN);
    g_counts.left_phase =
        (((levels >> DRV_MOTION_LEFT_A_PIN) & 1u) * 2u) +
        ((levels >> DRV_MOTION_LEFT_B_PIN) & 1u);
    g_counts.right_phase =
        (((levels >> DRV_MOTION_RIGHT_A_PIN) & 1u) * 2u) +
        ((levels >> DRV_MOTION_RIGHT_B_PIN) & 1u);
    definition.intatr = TA_HLNG;
    /* FP is the kernel's unprototyped interrupt callback type (BARR 6.2). */
    definition.inthdr = (FP)drv_motion_gpio_isr;
    result = tk_def_int(DRV_MOTION_IRQ, &definition);
    if (E_OK == result)
    {
        drv_motion_enable_edges(g_callback.pins | DRV_MOTION_ENCODER_MASK);
        /* IRQ enable and encoder initialization run on core 0 at startup. */
        EnableInt(DRV_MOTION_IRQ, DRV_MOTION_IRQ_PRIORITY);
        g_callback.started = 1u;
    }
    return (E_OK == result);
}

static void drv_motion_output(uint32_t base, int32_t duty,
                               int32_t * p_previous)
{
    uint32_t channel_a = 0u;
    uint32_t channel_b = 0u;
    drv_motion_plan(duty, p_previous, &channel_a, &channel_b);
    channel_a = channel_a * (DRV_MOTION_TOP + 1u) / 1000u;
    channel_b = channel_b * (DRV_MOTION_TOP + 1u) / 1000u;
    /* One whole CC write prevents a transient high/high drive combination. */
    out_w(base + PWM_CHx_CC, channel_a | (channel_b << 16u));
}

void drv_motion_write(int32_t left, int32_t right)
{
    int32_t previous_left = (int32_t)g_previous.left - 1000;
    int32_t previous_right = (int32_t)g_previous.right - 1000;
    drv_motion_output(PWM_CH4_BASE, left, &previous_left);
    drv_motion_output(PWM_CH5_BASE, right, &previous_right);
    g_previous.left = (uint32_t)(previous_left + 1000);
    g_previous.right = (uint32_t)(previous_right + 1000);
}
/* End of file. */

/** @file drv_motion.c
 *
 * @brief Motor PWM and wheel encoder driver for the Robo Pico.
 *
 * Registers: RP2040 datasheet section 2.19 (GPIO) and 4.5 (PWM). The motor
 * driver brakes when both of its inputs are low.
 */

#include "drv_motion.h"
#include "../app.h"

#define DRV_MOTION_IRQ           13u /* IO_IRQ_BANK0 */
#define DRV_MOTION_IRQ_PRIORITY  2
#define DRV_MOTION_GPIO_BANKS    4u
#define DRV_MOTION_PINS_PER_BANK 8u
#define DRV_MOTION_INTR          (IO_BANK0_BASE + 0xf0u)
#define DRV_MOTION_INTE          (IO_BANK0_BASE + 0x100u)
#define DRV_MOTION_INTS          (IO_BANK0_BASE + 0x120u)
#define DRV_MOTION_EDGE_BOTH     0xcu /* Falling and rising edge bits. */
#define DRV_MOTION_EDGE_BITS     0xccccccccu
#define DRV_MOTION_LEFT_A        16u
#define DRV_MOTION_LEFT_B        17u
#define DRV_MOTION_RIGHT_A       2u
#define DRV_MOTION_RIGHT_B       3u
#define DRV_MOTION_MOTOR_FIRST   8u
#define DRV_MOTION_MOTOR_LAST    11u
#define DRV_MOTION_PWM_DIV       16u   /* 8.4 fixed point, divide by 1. */
#define DRV_MOTION_PWM_TOP       6249u /* 125 MHz / 6250 = 20 kHz. */
#define DRV_MOTION_SKIP          2     /* Decode table: both phases changed. */

#define DRV_MOTION_ENCODER_PINS                                                \
    ((1u << DRV_MOTION_LEFT_A) | (1u << DRV_MOTION_LEFT_B) |                   \
     (1u << DRV_MOTION_RIGHT_A) | (1u << DRV_MOTION_RIGHT_B))

/* Wiring polarity measured on the robot (S2-U11, S2-U24). */
#define DRV_MOTION_LEFT_MOTOR_SIGN    (-1)
#define DRV_MOTION_RIGHT_MOTOR_SIGN   1
#define DRV_MOTION_LEFT_ENCODER_SIGN  (-1)
#define DRV_MOTION_RIGHT_ENCODER_SIGN 1

/* Decoder state of one wheel, used only by the ISR. */
typedef struct
{
    unsigned int phase : 2;      /* A * 2 + B */
    unsigned int b_backward : 1; /* Last valid step was backwards. */
} drv_motion_wheel_t;

/* Index: old phase * 4 + new phase. Forward order is 0, 1, 3, 2. */
static const int8_t g_decode[16] = {0,
                                    1,
                                    -1,
                                    DRV_MOTION_SKIP,
                                    -1,
                                    0,
                                    DRV_MOTION_SKIP,
                                    1,
                                    1,
                                    DRV_MOTION_SKIP,
                                    0,
                                    -1,
                                    DRV_MOTION_SKIP,
                                    -1,
                                    1,
                                    0};

/* Written only by the ISR. 8 and 16-bit loads are atomic on Cortex-M0+. */
static volatile uint16_t g_left_count;
static volatile uint16_t g_right_count;
static volatile uint8_t g_skipped;

static drv_motion_wheel_t g_left_wheel;
static drv_motion_wheel_t g_right_wheel;
static drv_motion_gpio_callback_t g_p_callback = NULL;
static uint32_t g_callback_pins;
static ID g_mutex;

static uint32_t drv_motion_phase(uint32_t levels, uint32_t pin_a,
                                 uint32_t pin_b);
static void drv_motion_count(volatile uint16_t * p_count,
                             drv_motion_wheel_t * p_wheel, uint32_t phase,
                             int32_t sign);
static void drv_motion_enable_edges(uint32_t pins);
static void drv_motion_output(uint32_t slice, int32_t duty);
static void drv_motion_gpio_isr(UINT intno);

/*!
 * @brief Set up 20 kHz PWM with both motors braked, the encoder inputs,
 * the GPIO interrupt and the state lock.
 *
 * @return True if the interrupt and the mutex were created.
 */
bool drv_motion_init(void)
{
    T_CMTX mutex = {0};
    T_DINT handler = {0};
    uint32_t pin = 0u;
    uint32_t levels = 0u;
    ER result = E_OK;

    /* usermain() has already taken the PWM block out of reset. */
    out_w(PWM_CH4_BASE + PWM_CHx_DIV, DRV_MOTION_PWM_DIV);
    out_w(PWM_CH5_BASE + PWM_CHx_DIV, DRV_MOTION_PWM_DIV);
    out_w(PWM_CH4_BASE + PWM_CHx_TOP, DRV_MOTION_PWM_TOP);
    out_w(PWM_CH5_BASE + PWM_CHx_TOP, DRV_MOTION_PWM_TOP);
    for (pin = DRV_MOTION_MOTOR_FIRST; pin <= DRV_MOTION_MOTOR_LAST; pin++)
    {
        (void)pwm_set_pin(pin);
    }
    out_w(PWM_CH4_BASE + PWM_CHx_CSR, 1u);
    out_w(PWM_CH5_BASE + PWM_CHx_CSR, 1u);

    /* Pull-ups, in case the encoder outputs are open collector. */
    for (pin = 0u; pin < GPIO_NUM; pin++)
    {
        if (0u != (DRV_MOTION_ENCODER_PINS & (1u << pin)))
        {
            (void)gpio_set_pin(pin, GPIO_MODE_IN);
            clr_w(GPIO(pin), GPIO_PDE);
            set_w(GPIO(pin), GPIO_PUE);
        }
    }
    levels = in_w(GPIO_IN);
    g_left_wheel.phase =
        drv_motion_phase(levels, DRV_MOTION_LEFT_A, DRV_MOTION_LEFT_B);
    g_right_wheel.phase =
        drv_motion_phase(levels, DRV_MOTION_RIGHT_A, DRV_MOTION_RIGHT_B);

    mutex.mtxatr = TA_INHERIT;
    g_mutex = tk_cre_mtx(&mutex);

    handler.intatr = TA_HLNG;
    /* FP is the kernel's generic handler pointer type. */
    handler.inthdr = (FP)drv_motion_gpio_isr;
    result = tk_def_int(DRV_MOTION_IRQ, &handler);
    if (E_OK == result)
    {
        drv_motion_enable_edges(DRV_MOTION_ENCODER_PINS);
        EnableInt(DRV_MOTION_IRQ, DRV_MOTION_IRQ_PRIORITY);
    }
    return ((E_OK == result) && (g_mutex > 0));
}

/*!
 * @brief Share the GPIO interrupt with another subsystem.
 *
 * Call once, on core 0, from that subsystem's init function. The callback
 * runs inside the ISR, so keep it short. Encoder pins are refused.
 *
 * @param[in] pins       Bit n set for each GPIO n to watch.
 * @param[in] p_callback Called with the pins that saw an edge.
 * @return True if registered.
 */
bool drv_motion_gpio_register(uint32_t pins,
                              drv_motion_gpio_callback_t p_callback)
{
    bool b_accepted = ((NULL == g_p_callback) && (NULL != p_callback) &&
                       (0u == (pins & DRV_MOTION_ENCODER_PINS)));

    if (b_accepted)
    {
        g_callback_pins = pins;
        g_p_callback = p_callback;
        drv_motion_enable_edges(pins);
    }
    return b_accepted;
}

/*!
 * @brief Drive both motors.
 *
 * @param[in] left_duty  -1000..1000 permille, positive is forward.
 * @param[in] right_duty -1000..1000 permille.
 */
void drv_motion_write(int32_t left_duty, int32_t right_duty)
{
    drv_motion_output(PWM_CH4_BASE, DRV_MOTION_LEFT_MOTOR_SIGN * left_duty);
    drv_motion_output(PWM_CH5_BASE, DRV_MOTION_RIGHT_MOTOR_SIGN * right_duty);
}

/*!
 * @brief Copy the encoder counts. Forward rotation counts up.
 */
void drv_motion_read(drv_motion_counts_t * p_counts)
{
    p_counts->left = g_left_count;
    p_counts->right = g_right_count;
    p_counts->skipped = g_skipped;
}

/*!
 * @brief Free-running 1 MHz timer, for test timing. usermain() starts its
 * tick. Wraps every 71 minutes.
 */
uint32_t drv_motion_time_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

/*!
 * @brief Take the motion state lock.
 */
void drv_motion_lock(void)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
}

/*!
 * @brief Release the motion state lock.
 */
void drv_motion_unlock(void)
{
    (void)tk_unl_mtx(g_mutex);
}

/*!
 * @brief Encoder phase (A * 2 + B) from a GPIO_IN reading.
 */
static uint32_t drv_motion_phase(uint32_t levels, uint32_t pin_a,
                                 uint32_t pin_b)
{
    return (((levels >> pin_a) & 1u) * 2u) + ((levels >> pin_b) & 1u);
}

/*!
 * @brief Update one wheel's count for a newly read phase.
 */
static void drv_motion_count(volatile uint16_t * p_count,
                             drv_motion_wheel_t * p_wheel, uint32_t phase,
                             int32_t sign)
{
    int32_t step = g_decode[(p_wheel->phase * 4u) + phase];

    if (DRV_MOTION_SKIP == step)
    {
        /* Two edges since the last read: assume the same direction. */
        step = (0u != p_wheel->b_backward) ? -2 : 2;
        g_skipped++;
    }
    else if (0 != step)
    {
        p_wheel->b_backward = (step < 0) ? 1u : 0u;
    }
    else
    {
        /* No change: the edge was on another pin. */
    }

    /* The counter is meant to wrap modulo 65536. */
    *p_count = (uint16_t)(*p_count + (step * sign));
    p_wheel->phase = phase;
}

/*!
 * @brief Clear old edge flags on these pins, then enable their interrupts.
 */
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
            if (0u !=
                (pins & (1u << ((bank * DRV_MOTION_PINS_PER_BANK) + pin))))
            {
                mask |= DRV_MOTION_EDGE_BOTH << (pin * 4u);
            }
        }
        out_w(DRV_MOTION_INTR + (bank * 4u), mask);
        set_w(DRV_MOTION_INTE + (bank * 4u), mask);
    }
}

/*!
 * @brief Set one motor's PWM, duty -1000..1000 permille. Positive duty
 * drives input A, zero brakes.
 */
static void drv_motion_output(uint32_t slice, int32_t duty)
{
    uint32_t magnitude = 0u;
    uint32_t level = 0u;

    /* Duty is within -1000..1000, so its magnitude fits uint32_t. */
    magnitude = (uint32_t)((duty < 0) ? -duty : duty);
    level = (magnitude * (DRV_MOTION_PWM_TOP + 1u)) / 1000u;

    /* One write sets both inputs, so they never change one at a time. */
    out_w(slice + PWM_CHx_CC, (duty > 0) ? level : (level << 16));
}

/*!
 * @brief GPIO bank 0 interrupt: count encoder edges and pass other pins
 * to the registered callback.
 */
static void drv_motion_gpio_isr(UINT intno)
{
    uint32_t pins = 0u;
    uint32_t bank = 0u;
    uint32_t pin = 0u;
    uint32_t edges = 0u;
    uint32_t levels = 0u;

    (void)intno;
    for (bank = 0u; bank < DRV_MOTION_GPIO_BANKS; bank++)
    {
        edges = in_w(DRV_MOTION_INTS + (bank * 4u)) & DRV_MOTION_EDGE_BITS;
        if (0u != edges) /* Usually only one bank has edges. */
        {
            out_w(DRV_MOTION_INTR + (bank * 4u), edges);
            for (pin = 0u; pin < DRV_MOTION_PINS_PER_BANK; pin++)
            {
                if (0u != (edges & (DRV_MOTION_EDGE_BOTH << (pin * 4u))))
                {
                    pins |= 1u << ((bank * DRV_MOTION_PINS_PER_BANK) + pin);
                }
            }
        }
    }

    /* Read the pins after clearing the flags, so a later edge is not lost. */
    levels = in_w(GPIO_IN);
    drv_motion_count(
        &g_left_count, &g_left_wheel,
        drv_motion_phase(levels, DRV_MOTION_LEFT_A, DRV_MOTION_LEFT_B),
        DRV_MOTION_LEFT_ENCODER_SIGN);
    drv_motion_count(
        &g_right_count, &g_right_wheel,
        drv_motion_phase(levels, DRV_MOTION_RIGHT_A, DRV_MOTION_RIGHT_B),
        DRV_MOTION_RIGHT_ENCODER_SIGN);

    if ((NULL != g_p_callback) && (0u != (pins & g_callback_pins)))
    {
        g_p_callback(pins & g_callback_pins);
    }
}

/*** end of file ***/

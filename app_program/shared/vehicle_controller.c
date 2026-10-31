/** @file vehicle_controller.c
 *
 * @brief Vehicle Controller task. Every 10 ms it starts and stops line
 * following, hands the car to obstacle avoidance and takes it back,
 * executes the last validated barcode at the next junction, and latches a
 * safe stop on GP20, an IMU fault or an avoidance safe stop.
 *
 * Every variable here belongs to the controller task, so none is locked.
 */

#include "vehicle_controller.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem3/barcode_decoder.h"
#include "../subsystem3/line_following.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/obstacle_avoidance.h"

#define VC_PERIOD_MS   10u
#define VC_BUTTON      20u  /* Robo Pico button, low when pressed. */
#define VC_RELEASED    5u   /* Cycles released before a press counts. */
#define VC_SPEED_MM_S  200u /* Line following speed. */
#define VC_AXLE_MM     110u /* Line sensors ahead of the axle. */
#define VC_MOVE_CYCLES 500u /* 5 s limit on each junction move. */

typedef enum
{
    VC_INIT,     /* Waiting for the IMU calibration. */
    VC_READY,    /* Stopped, waiting for GP20. */
    VC_FOLLOW,   /* Line following, avoidance watching ahead. */
    VC_AVOID,    /* Avoidance drives the car. */
    VC_TURN,     /* Executing a barcode at a junction. */
    VC_SAFE_STOP /* Latched until GP20 with no IMU fault. */
} vc_state_t;

static const char * const g_names[] = {"INIT",  "READY", "FOLLOW",
                                       "AVOID", "TURN",  "SAFE_STOP"};

static vc_state_t g_state = VC_INIT;
static uint32_t g_cycles;   /* Cycles since the state or move began. */
static uint32_t g_released; /* Cycles GP20 has been released. */
static uint32_t g_pending;  /* BARCODE_DECODER_... for the next junction. */
static bool g_b_turning;    /* TURN: past the roll forward, turning. */

static void vehicle_controller_task(INT stacd, void * p_exinf);
static void vc_enter(vc_state_t state, const char * p_cause);
static bool vc_pressed(void);
static void vc_turn_step(const motion_control_status_t * p_motion);

/*!
 * @brief Set up the GP20 button and start the controller task.
 */
ER vehicle_controller_start(void)
{
    /* Pull-up only (both on is bus keeper): the button grounds the pin. */
    (void)gpio_set_pin(VC_BUTTON, GPIO_MODE_IN);
    clr_w(GPIO(VC_BUTTON), GPIO_PDE);
    set_w(GPIO(VC_BUTTON), GPIO_PUE);

    /* FP is the kernel's generic task pointer type. */
    ID task = app_start_task((FP)vehicle_controller_task, VC_PRI, 0, "vc");

    return (task < E_OK) ? task : E_OK;
}

/*!
 * @brief Controller task: read every input, then step the state machine.
 */
static void vehicle_controller_task(INT stacd, void * p_exinf)
{
    s4_fault_t fault = {0};
    s4_tilt_t tilt = {0};
    barcode_decoder_t barcode = {0};
    line_following_t line = {0};
    obstacle_avoidance_status_t avoid = {0};
    motion_control_status_t motion = {0};
    uint32_t barcodes = 0u;      /* barcode.count last taken. */
    bool b_was_junction = false; /* line.b_junction last taken. */
    bool b_press = false;
    bool b_fault = false;
    bool b_junction = false;
    bool b_barcode = false;
    bool b_moving = false;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        tk_dly_tsk(VC_PERIOD_MS);
        g_cycles++;
        b_press = vc_pressed();
        s4_get_fault(&fault);
        b_fault = (0u != fault.impact) || (0u != fault.tilt);
        barcode_decoder_get(&barcode);
        line_following_get(&line);
        obstacle_avoidance_get_status(&avoid);
        motion_control_get_status(&motion);

        /* Barcodes and junctions are taken only while ready or following,
         * so one seen during a turn or a bypass waits for FOLLOW. */
        b_barcode = (barcode.count != barcodes);
        b_junction = (0u != line.b_junction) && !b_was_junction;
        if ((VC_READY == g_state) || (VC_FOLLOW == g_state))
        {
            barcodes = barcode.count;
            b_was_junction = (0u != line.b_junction);
        }
        b_moving = (g_state >= VC_FOLLOW) && (g_state <= VC_TURN);

        if (b_fault && (VC_SAFE_STOP != g_state))
        {
            vc_enter(VC_SAFE_STOP, "IMU fault");
        }
        else if (b_moving && b_press)
        {
            vc_enter(VC_SAFE_STOP, "GP20");
        }
        else if (b_moving && (OBSTACLE_AVOIDANCE_STOPPED == avoid.state))
        {
            vc_enter(VC_SAFE_STOP, "avoidance stopped");
        }
        else
        {
            switch (g_state)
            {
                case VC_INIT:
                    s4_get_tilt(&tilt);
                    if (0u != tilt.cal)
                    {
                        vc_enter(VC_READY, "IMU calibrated");
                    }
                    break;

                case VC_READY:
                    if (b_press)
                    {
                        vc_enter(VC_FOLLOW, "GP20");
                    }
                    break;

                case VC_FOLLOW:
                    if (b_barcode)
                    {
                        g_pending = barcode.command;
                        tm_printf((UB *)"[vc] barcode %d pending\n",
                                  (int)g_pending);
                    }
                    if ((OBSTACLE_AVOIDANCE_PROFILE == avoid.state) ||
                        (OBSTACLE_AVOIDANCE_BYPASS == avoid.state))
                    {
                        vc_enter(VC_AVOID, "obstacle");
                    }
                    else if (MOTION_CONTROL_FAILED == motion.state)
                    {
                        vc_enter(VC_SAFE_STOP, "wheel stalled");
                    }
                    else if (b_junction && (BARCODE_DECODER_NONE != g_pending))
                    {
                        vc_enter(VC_TURN, "junction");
                    }
                    else
                    {
                        /* Keep following. */
                    }
                    break;

                case VC_AVOID:
                    if (OBSTACLE_AVOIDANCE_MONITOR == avoid.state)
                    {
                        vc_enter(VC_FOLLOW, "obstacle passed");
                    }
                    break;

                case VC_TURN:
                    vc_turn_step(&motion);
                    break;

                case VC_SAFE_STOP:
                    if (b_press && !b_fault)
                    {
                        vc_enter(VC_READY, "GP20");
                    }
                    break;

                default:
                    break;
            }
        }
    }
}

/*!
 * @brief Change state, print why, and run the new state's entry actions.
 */
static void vc_enter(vc_state_t state, const char * p_cause)
{
    s4_hump_t hump = {0};

    tm_printf((UB *)"[vc] %s -> %s: %s\n", g_names[g_state], g_names[state],
              p_cause);
    g_state = state;
    g_cycles = 0u;
    switch (state)
    {
        case VC_READY:
            g_pending = BARCODE_DECODER_NONE;
            break;

        case VC_FOLLOW:
            (void)line_following_start(VC_SPEED_MM_S);
            obstacle_avoidance_start(); /* Nothing if already watching. */
            break;

        case VC_TURN:
            line_following_stop();
            obstacle_avoidance_cancel();
            g_b_turning = false;
            (void)motion_control_move_forward(VC_AXLE_MM);
            break;

        case VC_SAFE_STOP:
            obstacle_avoidance_cancel(); /* First, so it sees no fault. */
            line_following_stop();       /* Also brakes. */
            s4_get_hump(&hump);
            tm_printf((UB *)"[vc] max hump %d mm\n", (int)hump.max);
            break;

        default:
            break;
    }
}

/*!
 * @brief True once per GP20 press, ignoring bounce on release.
 */
static bool vc_pressed(void)
{
    bool b_press = false;

    if (0u != gpio_get_val(VC_BUTTON))
    {
        g_released++;
    }
    else
    {
        b_press = (g_released >= VC_RELEASED);
        g_released = 0u;
    }
    return b_press;
}

/*!
 * @brief TURN: after the roll forward onto the junction, turn for the
 * pending barcode (none for straight on), then follow the line again.
 */
static void vc_turn_step(const motion_control_status_t * p_motion)
{
    if (MOTION_CONTROL_RUNNING == p_motion->state)
    {
        if (g_cycles > VC_MOVE_CYCLES)
        {
            vc_enter(VC_SAFE_STOP, "move timeout");
        }
    }
    else if (MOTION_CONTROL_COMPLETE != p_motion->state)
    {
        vc_enter(VC_SAFE_STOP, "move failed");
    }
    else if (!g_b_turning)
    {
        g_b_turning = true;
        g_cycles = 0u;
        if (BARCODE_DECODER_LEFT == g_pending)
        {
            (void)motion_control_turn_left(90u);
        }
        else if (BARCODE_DECODER_RIGHT == g_pending)
        {
            (void)motion_control_turn_right(90u);
        }
        else if (BARCODE_DECODER_UTURN == g_pending)
        {
            (void)motion_control_turn_left(180u);
        }
        else
        {
            /* Straight on: the next cycle still sees COMPLETE. */
        }
    }
    else
    {
        g_pending = BARCODE_DECODER_NONE;
        vc_enter(VC_FOLLOW, "turn done");
    }
}

/*** end of file ***/

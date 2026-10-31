/** @file vehicle_controller.c
 *
 * @brief Vehicle Controller task. Every 10 ms it starts and stops line
 * following, hands the car to obstacle avoidance and takes it back,
 * executes the last validated barcode at the next junction, and goes to a
 * safe stop when the Safety Monitor latches one. Its own stops (GP20, an
 * avoidance safe stop, a lost line, a failed junction move) latch the Safety
 * Monitor too, so every stop is held the same way.
 *
 * Every variable here belongs to the controller task, so none is locked.
 * Other tasks only read the state, with vehicle_controller_state().
 */

#include "vehicle_controller.h"
#include "safety_monitor.h"
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
#define VC_TURN_DEG    90u  /* Barcode A and B. */
#define VC_UTURN_DEG   180u /* Barcode D. */

typedef enum
{
    VC_INIT,     /* Waiting for the IMU calibration. */
    VC_READY,    /* Stopped, waiting for GP20. */
    VC_FOLLOW,   /* Line following, avoidance watching ahead. */
    VC_AVOID,    /* Avoidance drives the car. */
    VC_TURN,     /* Executing a barcode at a junction. */
    VC_SAFE_STOP /* Until GP20 clears the Safety Monitor. */
} vc_state_t;

static const char * const g_names[] = {"INIT",  "READY", "FOLLOW",
                                       "AVOID", "TURN",  "SAFE_STOP"};

static struct
{
    unsigned int state : 3;     /* vc_state_t, VC_INIT (0) at start. */
    unsigned int cycles : 10;   /* Since the state or move began, wraps. */
    unsigned int released : 3;  /* Cycles GP20 released, up to VC_RELEASED. */
    unsigned int pending : 3;   /* BARCODE_DECODER_... for the junction. */
    unsigned int b_turning : 1; /* TURN: past the roll forward, turning. */
} g_vc;

/* The fields must hold their limits. */
_Static_assert(VC_MOVE_CYCLES < 1023u, "cycles must pass the move limit");
_Static_assert(VC_RELEASED <= 7u, "released must reach VC_RELEASED");

static void vehicle_controller_task(INT stacd, void * p_exinf);
static void vc_step(bool b_press, bool b_barcode, bool b_junction,
                    bool b_searching, uint32_t command, uint32_t avoid_state,
                    const motion_control_status_t * p_motion);
static void vc_enter(vc_state_t state, const char * p_cause);
static bool vc_pressed(void);
static void vc_turn_step(const motion_control_status_t * p_motion);

/*!
 * @brief Set up the GP20 button and start the controller task.
 */
ER vehicle_controller_start(void)
{
    ID task = 0;

    app_pull_up(VC_BUTTON);

    /* FP is the kernel's generic task pointer type. */
    task = app_start_task((FP)vehicle_controller_task, VC_PRI, 0, "vc");

    return (task < E_OK) ? task : E_OK;
}

/*!
 * @brief The controller state (0 INIT to 5 SAFE_STOP), for telemetry. Safe
 * from any task: the controller is the only writer, and this is one load.
 */
uint32_t vehicle_controller_state(void)
{
    return g_vc.state;
}

/*!
 * @brief Controller task: read every input, take a safe stop if one is due,
 * otherwise step the state machine.
 */
static void vehicle_controller_task(INT stacd, void * p_exinf)
{
    barcode_decoder_t barcode = {0};
    line_following_t line = {0};
    obstacle_avoidance_status_t avoid = {0};
    motion_control_status_t motion = {0};
    uint32_t barcodes = 0u;      /* barcode.count last taken. */
    bool b_was_junction = false; /* line.b_junction last taken. */
    bool b_press = false;
    bool b_latched = false; /* Safety stop held by motion control. */
    bool b_junction = false;
    bool b_barcode = false;
    bool b_moving = false;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_dly_tsk(VC_PERIOD_MS);
        g_vc.cycles++;
        b_press = vc_pressed();
        b_latched = motion_control_latched();
        barcode_decoder_get(&barcode);
        line_following_get(&line);
        obstacle_avoidance_get_status(&avoid);
        motion_control_get_status(&motion);

        /* Barcodes and junctions are taken only while ready or following,
         * so one seen during a turn or a bypass waits for FOLLOW. */
        b_barcode = (barcode.count != barcodes);
        b_junction = (0u != line.b_junction) && !b_was_junction;
        if ((VC_READY == g_vc.state) || (VC_FOLLOW == g_vc.state))
        {
            barcodes = barcode.count;
            b_was_junction = (0u != line.b_junction);
        }
        b_moving = (g_vc.state >= VC_FOLLOW) && (g_vc.state <= VC_TURN);

        if (b_latched && (VC_SAFE_STOP != g_vc.state))
        {
            vc_enter(VC_SAFE_STOP, "safety latched");
        }
        else if (b_moving && b_press)
        {
            vc_enter(VC_SAFE_STOP, "GP20");
        }
        else if (b_moving && (OBSTACLE_AVOIDANCE_STOPPED == avoid.state))
        {
            vc_enter(VC_SAFE_STOP, "avoidance stopped");
        }
        else if (b_moving && (0u != line.b_lost))
        {
            vc_enter(VC_SAFE_STOP, "line lost");
        }
        else
        {
            vc_step(b_press, b_barcode, b_junction, (0u != line.b_searching),
                    barcode.command, avoid.state, &motion);
        }
    }
}

/*!
 * @brief One step of the state machine, when no safe stop is due.
 *
 * @param[in] b_press     GP20 was pressed.
 * @param[in] b_barcode   A barcode was validated since FOLLOW last ran.
 * @param[in] b_junction  A junction began since FOLLOW last ran.
 * @param[in] b_searching The line follower is pivoting to find the line.
 * @param[in] command     The last validated barcode command.
 * @param[in] avoid_state Obstacle avoidance state.
 * @param[in] p_motion    Motion status.
 */
static void vc_step(bool b_press, bool b_barcode, bool b_junction,
                    bool b_searching, uint32_t command, uint32_t avoid_state,
                    const motion_control_status_t * p_motion)
{
    s4_tilt_t tilt = {0};

    switch (g_vc.state)
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
                g_vc.pending = command;
                tm_printf((UB *)"[vc] barcode %u pending\n", g_vc.pending);
            }
            /* Pause avoidance while the line follower pivots: the sweep
             * would see the sides as obstacles ahead. */
            if (b_searching && (OBSTACLE_AVOIDANCE_MONITOR == avoid_state))
            {
                obstacle_avoidance_cancel();
            }
            else if (!b_searching && (OBSTACLE_AVOIDANCE_IDLE == avoid_state))
            {
                obstacle_avoidance_start();
            }
            else
            {
                /* Avoidance already matches the line follower. */
            }
            if ((OBSTACLE_AVOIDANCE_PROFILE == avoid_state) ||
                (OBSTACLE_AVOIDANCE_BYPASS == avoid_state))
            {
                vc_enter(VC_AVOID, "obstacle");
            }
            else if (b_junction && (BARCODE_DECODER_NONE != g_vc.pending))
            {
                vc_enter(VC_TURN, "junction");
            }
            else
            {
                /* Keep following. */
            }
            break;

        case VC_AVOID:
            if (OBSTACLE_AVOIDANCE_MONITOR == avoid_state)
            {
                vc_enter(VC_FOLLOW, "obstacle passed");
            }
            break;

        case VC_TURN:
            vc_turn_step(p_motion);
            break;

        case VC_SAFE_STOP:
            if (b_press && safety_monitor_clear())
            {
                vc_enter(VC_INIT, "GP20"); /* READY once calibrated. */
            }
            break;

        default:
            break;
    }
}

/*!
 * @brief Change state, print why, and run the new state's entry actions.
 *
 * @param[in] state   The new state.
 * @param[in] p_cause Why, for the console.
 */
static void vc_enter(vc_state_t state, const char * p_cause)
{
    s4_hump_t hump = {0};

    tm_printf((UB *)"[vc] %s -> %s: %s\n", g_names[g_vc.state], g_names[state],
              p_cause);
    g_vc.state = state;
    g_vc.cycles = 0u;
    switch (state)
    {
        case VC_READY:
            g_vc.pending = BARCODE_DECODER_NONE;
            break;

        case VC_FOLLOW:
            (void)line_following_start(VC_SPEED_MM_S);
            obstacle_avoidance_start(); /* Nothing if already watching. */
            break;

        case VC_TURN:
            line_following_stop();
            obstacle_avoidance_cancel();
            g_vc.b_turning = 0u;
            (void)motion_control_move_forward(VC_AXLE_MM);
            break;

        case VC_SAFE_STOP:
            obstacle_avoidance_cancel(); /* First, so it sees no fault. */
            safety_monitor_stop(SAFETY_MONITOR_CONTROLLER); /* If ours. */
            line_following_stop();                          /* Stop steering. */
            s4_get_hump(&hump);
            tm_printf((UB *)"[vc] max hump %u mm\n", hump.max);
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
        if (g_vc.released < VC_RELEASED)
        {
            g_vc.released++;
        }
    }
    else
    {
        b_press = (g_vc.released >= VC_RELEASED);
        g_vc.released = 0u;
    }
    return b_press;
}

/*!
 * @brief TURN: after the roll forward onto the junction, turn for the
 * pending barcode (none for straight on), then follow the line again.
 *
 * @param[in] p_motion Motion status.
 */
static void vc_turn_step(const motion_control_status_t * p_motion)
{
    if (MOTION_CONTROL_RUNNING == p_motion->state)
    {
        if (g_vc.cycles > VC_MOVE_CYCLES)
        {
            vc_enter(VC_SAFE_STOP, "move timeout");
        }
    }
    else if (MOTION_CONTROL_COMPLETE != p_motion->state)
    {
        vc_enter(VC_SAFE_STOP, "move failed");
    }
    else if (0u == g_vc.b_turning)
    {
        g_vc.b_turning = 1u;
        g_vc.cycles = 0u;
        if (BARCODE_DECODER_LEFT == g_vc.pending)
        {
            (void)motion_control_turn_left(VC_TURN_DEG);
        }
        else if (BARCODE_DECODER_RIGHT == g_vc.pending)
        {
            (void)motion_control_turn_right(VC_TURN_DEG);
        }
        else if (BARCODE_DECODER_UTURN == g_vc.pending)
        {
            (void)motion_control_turn_left(VC_UTURN_DEG);
        }
        else
        {
            /* Straight on: the next cycle still sees COMPLETE. */
        }
    }
    else
    {
        g_vc.pending = BARCODE_DECODER_NONE;
        vc_enter(VC_FOLLOW, "turn done");
    }
}

/*** end of file ***/

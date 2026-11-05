/** @file line_following.c
 *
 * @brief Line Following API and task, the only task that reads the line
 * sensors.
 *
 * Both sensors white means centred or lost, so the line counts as lost
 * only after LINE_FOLLOWING_LOST_MM of all-white travel. The task then
 * pivots towards last_side, then back past the start heading the other
 * way, and resumes following as soon as either sensor sees black. A false
 * alarm on a straight costs a small pivot: the line soon reaches a sensor.
 */

#include "line_following.h"
#include "subsystem3.h"
#include "../subsystem2/motion_control.h"

#define LINE_FOLLOWING_BOTH     (DRV_LINE_LEFT | DRV_LINE_RIGHT)
#define LINE_FOLLOWING_JUNCTION 6u /* Samples: 60 ms, 9 mm at 0.15 m/s. */
/* Wheel speed change, permille of the speed: 1000 stops the inner wheel.
 * It rises towards the max over RISE_MM with one sensor black and falls
 * back to 0 over FALL_MM once neither (or both) is. */
#define LINE_FOLLOWING_TURN_MAX   1000
#define LINE_FOLLOWING_RISE_MM    40
#define LINE_FOLLOWING_FALL_MM    20
#define LINE_FOLLOWING_LOST_MM    80u /* All white this far: lost. */
#define LINE_FOLLOWING_SEARCH_DEG 135 /* Pivot each side of the heading. */
#define LINE_FOLLOWING_PIVOT_MM_S 100 /* Wheel speed while pivoting. */

/* Line search, under gh_mutex. */
typedef struct
{
    unsigned int from_mm : 24;  /* Odometer when a sensor last saw black. */
    unsigned int phase : 2;     /* 0 none, 1 towards last_side, 2 back. */
    unsigned int start_deg : 9; /* Heading when the search began. */
} line_following_search_t;

static ID gh_mutex;
static line_following_t g_line;
static uint16_t g_speed_mm_s; /* Under gh_mutex. */
static line_following_search_t g_search;
static int32_t g_turn;      /* Permille of the speed, +left. Under gh_mutex. */
static uint32_t g_steer_mm; /* Odometer at the last steer. Under gh_mutex. */

static void line_following_steer(uint32_t bits, uint32_t distance_mm);
static void line_following_search(const motion_control_status_t * p_motion);

/*!
 * @brief Create the lock and set up the sensors.
 *
 * @return True if the lock was created.
 */
bool line_following_init(void)
{
    T_CMTX mutex = {0};

    mutex.mtxatr = TA_INHERIT;
    gh_mutex = tk_cre_mtx(&mutex);
    drv_line_init();
    return (gh_mutex > 0);
}

/*!
 * @brief Copy the latest line state.
 */
void line_following_get(line_following_t * p_line)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    *p_line = g_line;
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Start steering the car along the line, and clear b_lost.
 *
 * @param[in] speed_mm_s Forward speed, 1 to LINE_FOLLOWING_MAX_MM_S.
 * @return False if the speed is out of range; nothing changes.
 */
bool line_following_start(uint32_t speed_mm_s)
{
    bool const b_valid =
        (speed_mm_s > 0u) && (speed_mm_s <= LINE_FOLLOWING_MAX_MM_S);

    if (b_valid)
    {
        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_speed_mm_s = (uint16_t)speed_mm_s; /* <= MAX_MM_S: fits. */
        g_line.b_following = true;
        g_line.b_lost = false;
        (void)tk_unl_mtx(gh_mutex);
    }
    return b_valid;
}

/*!
 * @brief Stop steering and brake. The task drives only under the lock, so
 * no drive command can follow the brake.
 */
void line_following_stop(void)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    g_line.b_following = false;
    (void)tk_unl_mtx(gh_mutex);
    motion_control_stop();
}

/*!
 * @brief Line Following task: sample both sensors every 10 ms and, while
 * following, steer along the line or search for it once lost.
 */
void line_following_task(INT stacd, void * p_exinf)
{
    SYSTIM now = {0};
    motion_control_status_t motion = {0};
    uint32_t bits = 0u;
    uint32_t both = 0u; /* Samples in a row with both sensors black. */

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* Woken every 10 ms (subsystem3.c). */
        bits = drv_line_read();
        both = (LINE_FOLLOWING_BOTH == bits) ? (both + 1u) : 0u;
        (void)tk_get_otm(&now);
        motion_control_get_status(&motion);

        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_line.bits = bits;
        if ((DRV_LINE_LEFT == bits) || (DRV_LINE_RIGHT == bits))
        {
            g_line.last_side = bits;
        }
        g_line.b_junction = (both >= LINE_FOLLOWING_JUNCTION);
        g_line.count++;
        g_line.sample_ms = now.lo;
        if ((!g_line.b_following) || (0u != bits))
        {
            g_search.phase = 0u; /* Not following, found, or never lost. */
            g_search.from_mm = motion.distance_mm;
        }
        else if ((0u == g_search.phase) &&
                 (((motion.distance_mm - g_search.from_mm) &
                   MOTION_CONTROL_DISTANCE_MASK) >= LINE_FOLLOWING_LOST_MM))
        {
            g_search.phase = 1u;
            g_search.start_deg = motion.heading_deg;
        }
        else
        {
            /* Centred, or still searching. */
        }
        if ((g_line.b_following) && (0u == g_search.phase))
        {
            line_following_steer(bits, motion.distance_mm);
        }
        else
        {
            g_turn = 0; /* Steering starts again from straight on. */
            g_steer_mm = motion.distance_mm;
            if (g_line.b_following)
            {
                line_following_search(&motion);
            }
        }
        g_line.b_searching = (g_line.b_following) && (0u != g_search.phase);
        (void)tk_unl_mtx(gh_mutex);
    }
}

/*!
 * @brief Steer towards the side whose sensor sees black, the turn rising
 * the further the car rolls with it black and falling once it is not.
 * Call under gh_mutex while following.
 *
 * @param[in] bits        Sensors over black.
 * @param[in] distance_mm Odometer now.
 */
static void line_following_steer(uint32_t bits, uint32_t distance_mm)
{
    int32_t const speed = g_speed_mm_s; /* 16 bits: fits. */
    /* A few mm since the last sample, so the cast keeps it. */
    int32_t const mm =
        (int32_t)((distance_mm - g_steer_mm) & MOTION_CONTROL_DISTANCE_MASK);
    int32_t target = 0; /* Centred, or a junction: straight on. */
    int32_t step = (mm * LINE_FOLLOWING_TURN_MAX) / LINE_FOLLOWING_FALL_MM;

    if (DRV_LINE_LEFT == bits)
    {
        target = LINE_FOLLOWING_TURN_MAX;
    }
    else if (DRV_LINE_RIGHT == bits)
    {
        target = -LINE_FOLLOWING_TURN_MAX;
    }
    else
    {
        /* Straight on. */
    }
    if ((0 != target) && ((target * g_turn) >= 0))
    {
        step = (mm * LINE_FOLLOWING_TURN_MAX) / LINE_FOLLOWING_RISE_MM;
    }
    if (g_turn < target)
    {
        g_turn = ((g_turn + step) < target) ? (g_turn + step) : target;
    }
    else
    {
        g_turn = ((g_turn - step) > target) ? (g_turn - step) : target;
    }
    g_steer_mm = distance_mm;
    step = (speed * g_turn) / 1000; /* Wheel speed change, +left. */
    (void)motion_control_drive(speed - step, speed + step);
}

/*!
 * @brief Pivot towards last_side up to LINE_FOLLOWING_SEARCH_DEG, then the
 * other way to as far past the start heading; then brake and set b_lost.
 * Call under gh_mutex while following.
 *
 * @param[in] p_motion Motion status, for the heading.
 */
static void line_following_search(const motion_control_status_t * p_motion)
{
    int32_t side = (DRV_LINE_RIGHT == g_line.last_side) ? -1 : 1; /* +left. */
    /* Degrees turned anticlockwise since the search began, -180..179. Both
     * headings are 0..359, so the casts keep them. */
    int32_t turned =
        (((int32_t)p_motion->heading_deg - (int32_t)g_search.start_deg + 540) %
         360) -
        180;

    turned *= side; /* Now towards last_side. */
    if ((1u == g_search.phase) && (turned >= LINE_FOLLOWING_SEARCH_DEG))
    {
        g_search.phase = 2u;
    }
    if ((2u == g_search.phase) && (turned <= -LINE_FOLLOWING_SEARCH_DEG))
    {
        g_line.b_following = false;
        g_line.b_lost = true;
        motion_control_stop();
    }
    else
    {
        side = (1u == g_search.phase) ? side : -side;
        (void)motion_control_drive(-side * LINE_FOLLOWING_PIVOT_MM_S,
                                   side * LINE_FOLLOWING_PIVOT_MM_S);
    }
}

/*** end of file ***/

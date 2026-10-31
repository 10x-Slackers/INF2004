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
#define LINE_FOLLOWING_JUNCTION 3u     /* Samples: 30 ms, 6 mm at 0.2 m/s. */
#define LINE_FOLLOWING_TURN_PCT 50     /* Wheel speed change while one sensor */
                                       /* sees black, % of the speed: tune. */
#define LINE_FOLLOWING_LOST_MM    150u /* All white this far: lost. Tune. */
#define LINE_FOLLOWING_SEARCH_DEG 135  /* Pivot each side of the heading. */
#define LINE_FOLLOWING_PIVOT_MM_S 100  /* Wheel speed while pivoting. */
#define LINE_FOLLOWING_ODOMETER   0xFFFFFFu /* distance_mm wraps here. */

/* Line search, under g_mutex. */
typedef struct
{
    unsigned int from_mm : 24;  /* Odometer when a sensor last saw black. */
    unsigned int phase : 2;     /* 0 none, 1 towards last_side, 2 back. */
    unsigned int start_deg : 9; /* Heading when the search began. */
} line_following_search_t;

static ID g_mutex;
static line_following_t g_line;
static uint16_t g_speed_mm_s; /* Under g_mutex. */
static line_following_search_t g_search;

static void line_following_steer(uint32_t bits);
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
    g_mutex = tk_cre_mtx(&mutex);
    drv_line_init();
    return (g_mutex > 0);
}

/*!
 * @brief Copy the latest line state.
 */
void line_following_get(line_following_t * p_line)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    *p_line = g_line;
    (void)tk_unl_mtx(g_mutex);
}

/*!
 * @brief Start steering the car along the line, and clear b_lost.
 *
 * @param[in] speed_mm_s Forward speed, 1 to LINE_FOLLOWING_MAX_MM_S.
 * @return False if the speed is out of range; nothing changes.
 */
bool line_following_start(uint32_t speed_mm_s)
{
    bool b_valid = (speed_mm_s > 0u) && (speed_mm_s <= LINE_FOLLOWING_MAX_MM_S);
    motion_control_status_t motion = {0};

    if (b_valid)
    {
        motion_control_get_status(&motion);
        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        g_speed_mm_s = (uint16_t)speed_mm_s;
        g_line.b_following = true;
        g_line.b_lost = false;
        g_search.phase = 0u;
        g_search.from_mm = motion.distance_mm;
        (void)tk_unl_mtx(g_mutex);
    }
    return b_valid;
}

/*!
 * @brief Stop steering and brake. The task drives only under the lock, so
 * no drive command can follow the brake.
 */
void line_following_stop(void)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    g_line.b_following = false;
    (void)tk_unl_mtx(g_mutex);
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

        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        g_line.bits = bits;
        if ((DRV_LINE_LEFT == bits) || (DRV_LINE_RIGHT == bits))
        {
            g_line.last_side = bits;
        }
        g_line.b_junction = (both >= LINE_FOLLOWING_JUNCTION);
        g_line.count++;
        g_line.sample_ms = now.lo;
        if (0u != bits)
        {
            g_search.phase = 0u; /* Found, or never lost. */
            g_search.from_mm = motion.distance_mm;
        }
        else if ((0u == g_search.phase) &&
                 (((motion.distance_mm - g_search.from_mm) &
                   LINE_FOLLOWING_ODOMETER) >= LINE_FOLLOWING_LOST_MM))
        {
            g_search.phase = 1u;
            g_search.start_deg = motion.heading_deg;
        }
        else
        {
            /* Centred, or still searching. */
        }
        if (!g_line.b_following)
        {
            /* Not steering. */
        }
        else if (0u == g_search.phase)
        {
            line_following_steer(bits);
        }
        else
        {
            line_following_search(&motion);
        }
        (void)tk_unl_mtx(g_mutex);
    }
}

/*!
 * @brief Steer towards the side whose sensor sees black. Call under
 * g_mutex while following.
 *
 * @param[in] bits Sensors over black.
 */
static void line_following_steer(uint32_t bits)
{
    int32_t turn = (g_speed_mm_s * LINE_FOLLOWING_TURN_PCT) / 100; /* +left. */

    if (DRV_LINE_RIGHT == bits)
    {
        turn = -turn;
    }
    else if (DRV_LINE_LEFT != bits)
    {
        turn = 0; /* Centred, or a junction: straight on. */
    }
    else
    {
        /* The line is under the left sensor: turn left. */
    }
    (void)motion_control_drive(g_speed_mm_s - turn, g_speed_mm_s + turn);
}

/*!
 * @brief Pivot towards last_side up to LINE_FOLLOWING_SEARCH_DEG, then the
 * other way to as far past the start heading; then brake and set b_lost.
 * Call under g_mutex while following.
 *
 * @param[in] p_motion Motion status, for the heading.
 */
static void line_following_search(const motion_control_status_t * p_motion)
{
    int32_t side = (DRV_LINE_RIGHT == g_line.last_side) ? -1 : 1; /* +left. */
    /* Degrees turned anticlockwise since the search began, -180..179. */
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

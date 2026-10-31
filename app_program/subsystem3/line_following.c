/** @file line_following.c
 *
 * @brief Line Following API and task, the only task that reads the line
 * sensors.
 */

#include "line_following.h"
#include "subsystem3.h"
#include "../subsystem2/motion_control.h"

#define LINE_FOLLOWING_BOTH     (DRV_LINE_LEFT | DRV_LINE_RIGHT)
#define LINE_FOLLOWING_JUNCTION 3u /* Samples: 30 ms, 6 mm at 0.2 m/s. */
#define LINE_FOLLOWING_TURN_PCT 50 /* Wheel speed change while one sensor */
                                   /* sees black, % of the speed: tune. */

static ID g_mutex;
static line_following_t g_line;
static uint16_t g_speed_mm_s; /* Under g_mutex. */

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
 * @brief Start steering the car along the line.
 *
 * @param[in] speed_mm_s Forward speed, 1 to LINE_FOLLOWING_MAX_MM_S.
 * @return False if the speed is out of range; nothing changes.
 */
bool line_following_start(uint32_t speed_mm_s)
{
    bool b_valid = (speed_mm_s > 0u) && (speed_mm_s <= LINE_FOLLOWING_MAX_MM_S);

    if (b_valid)
    {
        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        g_speed_mm_s = (uint16_t)speed_mm_s;
        g_line.b_following = true;
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
 * following, steer toward the side whose sensor sees black.
 */
void line_following_task(INT stacd, void * p_exinf)
{
    SYSTIM now = {0};
    uint32_t bits = 0u;
    uint32_t both = 0u; /* Samples in a row with both sensors black. */
    int32_t turn = 0;   /* mm/s, positive turns left. */

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* Woken every 10 ms (subsystem3.c). */
        bits = drv_line_read();
        both = (LINE_FOLLOWING_BOTH == bits) ? (both + 1u) : 0u;
        (void)tk_get_otm(&now);

        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        g_line.bits = bits;
        if ((DRV_LINE_LEFT == bits) || (DRV_LINE_RIGHT == bits))
        {
            g_line.last_side = bits;
        }
        g_line.b_junction = (both >= LINE_FOLLOWING_JUNCTION);
        g_line.count++;
        g_line.sample_ms = now.lo;
        if (g_line.b_following)
        {
            turn = (g_speed_mm_s * LINE_FOLLOWING_TURN_PCT) / 100;
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
            (void)motion_control_drive(g_speed_mm_s - turn,
                                       g_speed_mm_s + turn);
        }
        (void)tk_unl_mtx(g_mutex);
    }
}

/*** end of file ***/

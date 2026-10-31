/** @file line_following.c
 *
 * @brief Line Following API and task, the only task that reads the line
 * sensors.
 */

#include "line_following.h"
#include "subsystem3.h"

#define LINE_FOLLOWING_BOTH     (DRV_LINE_LEFT | DRV_LINE_RIGHT)
#define LINE_FOLLOWING_JUNCTION 3u /* Samples: 30 ms, 6 mm at 0.2 m/s. */

static ID g_mutex;
static line_following_t g_line;

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
 * @brief Line Following task: sample both sensors every 10 ms.
 */
void line_following_task(INT stacd, void * p_exinf)
{
    SYSTIM now = {0};
    uint32_t bits = 0u;
    uint32_t both = 0u; /* Samples in a row with both sensors black. */

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
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
        (void)tk_unl_mtx(g_mutex);

        tk_dly_tsk(LINE_FOLLOWING_PERIOD_MS);
    }
}

/*** end of file ***/

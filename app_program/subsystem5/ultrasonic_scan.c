/** @file ultrasonic_scan.c
 *
 * @brief Ultrasonic scan API and the scan task, the only task that drives
 * the HC-SR04 and the servo.
 */

#include "ultrasonic_scan.h"
#include "drv_hcsr04.h"
#include "drv_sg90.h"
#include "subsystem5.h"

#define ULTRASONIC_SCAN_SETTLE_MS 200 /* Servo travel before a reading. */
#define ULTRASONIC_SCAN_READS     3u  /* Readings per ray, for a median. */

static ID gh_mutex;
static ID gh_request; /* Semaphore: a scan was requested. */
static struct
{
    unsigned int mode : 1;    /* ULTRASONIC_SCAN_... */
    unsigned int request : 4; /* Requests made, wraps. */
} g_next;
static ultrasonic_scan_t g_scan;
static uint8_t g_angle = SCAN_PROFILE_AHEAD_DEG; /* Scan task only. */

static void ultrasonic_scan_serve(uint32_t mode, uint32_t request);
static void ultrasonic_scan_aim(uint32_t angle_deg);
static uint16_t ultrasonic_scan_ray(uint32_t ray, bool * pb_fault);

/*!
 * @brief Create the lock and the request semaphore, set up the sensor and
 * point the servo ahead.
 *
 * @return True if everything was created.
 */
bool ultrasonic_scan_init(void)
{
    T_CMTX mutex = {0};
    T_CSEM semaphore = {0};

    mutex.mtxatr = TA_INHERIT;
    gh_mutex = tk_cre_mtx(&mutex);
    semaphore.sematr = TA_TFIFO;
    semaphore.maxsem = 1;
    gh_request = tk_cre_sem(&semaphore);
    drv_sg90_init();
    return (drv_hcsr04_init()) && (gh_mutex > 0) && (gh_request > 0);
}

/*!
 * @brief Ask for a scan. A request made while one runs is served after
 * it; only the latest is kept.
 *
 * @param[in] mode ULTRASONIC_SCAN_FORWARD or ULTRASONIC_SCAN_PROFILE.
 *
 * @return The request number its result will carry in .request.
 */
uint32_t ultrasonic_scan_request(uint32_t mode)
{
    uint32_t request = 0u;

    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    g_next.mode = mode;
    request = ++g_next.request;
    (void)tk_unl_mtx(gh_mutex);
    (void)tk_sig_sem(gh_request, 1);
    return request;
}

/*!
 * @brief Copy the latest results.
 */
void ultrasonic_scan_get(ultrasonic_scan_t * p_scan)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    *p_scan = g_scan;
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Scan task: one scan per request.
 */
void ultrasonic_scan_task(INT stacd, void * p_exinf)
{
    uint32_t mode = 0u;
    uint32_t request = 0u;
    bool b_new = false;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_wai_sem(gh_request, 1, TMO_FEVR);
        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        mode = g_next.mode;
        request = g_next.request;
        /* A request made while the task woke is read with the one before
         * it; its own signal then finds it already served. */
        b_new = (request != g_scan.request);
        (void)tk_unl_mtx(gh_mutex);
        if (b_new)
        {
            ultrasonic_scan_serve(mode, request);
        }
    }
}

/*!
 * @brief Take one scan and publish it with its request number.
 */
static void ultrasonic_scan_serve(uint32_t mode, uint32_t request)
{
    uint16_t rays[SCAN_PROFILE_RAYS] = {0};
    scan_profile_t profile = {0};
    SYSTIM now = {0};
    uint32_t ray = 0u;
    int32_t dir = 0;  /* -1 right, +1 left. */
    int32_t fine = 0; /* Ray of the fine scan. */
    uint32_t range_mm = 0u;
    bool b_fault = false;

    if (ULTRASONIC_SCAN_PROFILE == mode)
    {
        /* Coarse: every 15 degrees. Rays not measured read as open. */
        for (ray = 0u; ray < SCAN_PROFILE_RAYS; ray++)
        {
            rays[ray] = (0u == (ray % SCAN_PROFILE_COARSE))
                            ? ultrasonic_scan_ray(ray, &b_fault)
                            : (uint16_t)SCAN_PROFILE_FAR_MM;
        }
        /* Fine: out from the ray ahead, right then left, while blocked. */
        for (dir = -1; dir <= 1; dir += 2)
        {
            for (fine = (int32_t)SCAN_PROFILE_AHEAD + dir;
                 (fine >= 0) && (fine < (int32_t)SCAN_PROFILE_RAYS) &&
                 (rays[fine - dir] <= SCAN_PROFILE_BLOCK_MM);
                 fine += dir)
            {
                if (0 != (fine % (int32_t)SCAN_PROFILE_COARSE))
                {
                    rays[fine] = ultrasonic_scan_ray((uint32_t)fine, &b_fault);
                }
            }
        }
        scan_profile_build(rays, &profile);
    }
    ultrasonic_scan_aim(SCAN_PROFILE_AHEAD_DEG);
    b_fault = (!drv_hcsr04_measure(&range_mm)) || b_fault;
    (void)tk_get_otm(&now);

    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    g_scan.forward_mm = range_mm;
    g_scan.forward_ms = now.lo;
    /* APP_FAULT_NO_ECHO, before request tells readers it is done. */
    if (b_fault)
    {
        app_fault_set(APP_FAULT_NO_ECHO);
    }
    else
    {
        app_fault_clear(APP_FAULT_NO_ECHO);
    }
    if (ULTRASONIC_SCAN_PROFILE == mode)
    {
        g_scan.profile = profile;
    }
    g_scan.request = request;
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Point the servo, and wait for it to get there if it moved.
 */
static void ultrasonic_scan_aim(uint32_t angle_deg)
{
    if (angle_deg != g_angle)
    {
        drv_sg90_set(angle_deg);
        tk_dly_tsk(ULTRASONIC_SCAN_SETTLE_MS);
        g_angle = (uint8_t)angle_deg; /* 45..135 degrees: fits. */
    }
}

/*!
 * @brief Median of three readings along one ray.
 *
 * @return mm. SCAN_PROFILE_FAR_MM if the sensor gave no echo, so the fine
 * scan stops; *pb_fault reports it.
 */
static uint16_t ultrasonic_scan_ray(uint32_t ray, bool * pb_fault)
{
    uint32_t range_mm[ULTRASONIC_SCAN_READS] = {0};
    bool b_echo = true;
    uint32_t reading = 0u;

    ultrasonic_scan_aim(SCAN_PROFILE_DEG(ray));
    for (reading = 0u; reading < ULTRASONIC_SCAN_READS; reading++)
    {
        b_echo = (drv_hcsr04_measure(&range_mm[reading])) && b_echo;
    }
    *pb_fault = (*pb_fault) || (!b_echo);
    /* At most SCAN_PROFILE_FAR_MM (4000), so it fits. */
    return b_echo ? (uint16_t)scan_profile_median(range_mm[0], range_mm[1],
                                                  range_mm[2])
                  : (uint16_t)SCAN_PROFILE_FAR_MM;
}

/*** end of file ***/

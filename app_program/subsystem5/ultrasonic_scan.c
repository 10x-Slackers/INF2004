/** @file ultrasonic_scan.c
 *
 * @brief Ultrasonic scan API and the scan task, the only task that drives
 * the HC-SR04 and the servo.
 */

#include "ultrasonic_scan.h"
#include "drv_hcsr04.h"
#include "drv_sg90.h"
#include "subsystem5.h"

#define ULTRASONIC_SCAN_AHEAD     90u
#define ULTRASONIC_SCAN_SETTLE_MS 200 /* Servo travel before a reading. */

static ID g_mutex;
static ID g_request; /* Semaphore: a scan was requested. */
static struct
{
    unsigned int mode : 1;    /* ULTRASONIC_SCAN_... */
    unsigned int request : 4; /* Requests made, wraps. */
} g_next;
static ultrasonic_scan_t g_scan;
static uint8_t g_angle = ULTRASONIC_SCAN_AHEAD; /* Scan task only. */

static void ultrasonic_scan_aim(uint32_t angle_deg);
static uint32_t ultrasonic_scan_ray(uint32_t ray, bool * p_b_fault);

/*!
 * @brief Create the lock and the request semaphore, set up the sensor and
 * point the servo ahead. Call after subsystem2_init().
 *
 * @return True if everything was created.
 */
bool ultrasonic_scan_init(void)
{
    T_CMTX mutex = {0};
    T_CSEM semaphore = {0};

    mutex.mtxatr = TA_INHERIT;
    g_mutex = tk_cre_mtx(&mutex);
    semaphore.sematr = TA_TFIFO;
    semaphore.maxsem = 1;
    g_request = tk_cre_sem(&semaphore);
    drv_sg90_init();
    return drv_hcsr04_init() && (g_mutex > 0) && (g_request > 0);
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

    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    g_next.mode = mode;
    request = ++g_next.request;
    (void)tk_unl_mtx(g_mutex);
    (void)tk_sig_sem(g_request, 1);
    return request;
}

/*!
 * @brief Copy the latest results.
 */
void ultrasonic_scan_get(ultrasonic_scan_t * p_scan)
{
    (void)tk_loc_mtx(g_mutex, TMO_FEVR);
    *p_scan = g_scan;
    (void)tk_unl_mtx(g_mutex);
}

/*!
 * @brief Scan task: one scan per request.
 */
void ultrasonic_scan_task(INT stacd, void * p_exinf)
{
    uint16_t rays[SCAN_PROFILE_RAYS] = {0};
    scan_profile_t profile = {0};
    SYSTIM now = {0};
    uint32_t ray = 0u;
    uint32_t mode = 0u;
    uint32_t request = 0u;
    uint32_t mm = 0u;
    bool b_fault = false;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_wai_sem(g_request, 1, TMO_FEVR);
        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        mode = g_next.mode;
        request = g_next.request;
        (void)tk_unl_mtx(g_mutex);

        b_fault = false;
        if (ULTRASONIC_SCAN_PROFILE == mode)
        {
            for (ray = 0u; ray < SCAN_PROFILE_RAYS; ray++)
            {
                rays[ray] = (uint16_t)ultrasonic_scan_ray(ray, &b_fault);
            }
            scan_profile_build(rays, &profile);
        }
        ultrasonic_scan_aim(ULTRASONIC_SCAN_AHEAD);
        b_fault = !drv_hcsr04_measure(&mm) || b_fault;
        (void)tk_get_otm(&now);

        (void)tk_loc_mtx(g_mutex, TMO_FEVR);
        g_scan.forward_mm = mm;
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
        (void)tk_unl_mtx(g_mutex);
    }
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
        g_angle = (uint8_t)angle_deg;
    }
}

/*!
 * @brief Median of three readings along one ray.
 *
 * @return mm, or 0 if the sensor gave no echo.
 */
static uint32_t ultrasonic_scan_ray(uint32_t ray, bool * p_b_fault)
{
    uint32_t mm[3] = {0};
    bool b_echo = true;
    uint32_t i = 0u;

    ultrasonic_scan_aim(SCAN_PROFILE_MIN_DEG + (ray * SCAN_PROFILE_STEP_DEG));
    for (i = 0u; i < 3u; i++)
    {
        b_echo = drv_hcsr04_measure(&mm[i]) && b_echo;
    }
    *p_b_fault = *p_b_fault || !b_echo;
    return b_echo ? scan_profile_median(mm[0], mm[1], mm[2]) : 0u;
}

/*** end of file ***/

/* Two task owners, two bounded copy queues, one snapshot mutex. */
#include "subsystem5.h"
#include "drv_hcsr04.h"
#include "drv_sg90.h"

typedef struct
{
    unsigned int kind : 4;
    S5_SCAN scan;
} S5_MESSAGE;
typedef struct
{
    unsigned int mode : 2;
} S5_REQUEST;

_Static_assert(sizeof(S5_MESSAGE) == 104, "message ABI");
_Static_assert(sizeof(S5_REQUEST) == 4, "request ABI");

static ID inbox, requests, snapshot_mutex;
static S5_HEALTH snapshot;
/* Each queued message needs one extra word for the kernel's header. */
static UW inbox_storage[4 * (sizeof(S5_MESSAGE) / sizeof(UW) + 1)];
static UW request_storage[sizeof(S5_REQUEST) / sizeof(UW) + 1];

uint32_t s5_now_ms(void)
{
    SYSTIM now;
    tk_get_otm(&now);
    return now.lo;
}

void s5_get_health(S5_HEALTH * out)
{
    tk_loc_mtx(snapshot_mutex, TMO_FEVR);
    *out = snapshot;
    tk_unl_mtx(snapshot_mutex);
    out->front_clear =
        out->control.state == S5_MONITOR && out->scan_state == 0 &&
        s5_forward_clear(&out->control.forward, out->servo_angle, s5_now_ms());
    if (!out->front_clear)
    {
        out->control.faults |= S5_STALE_RANGE;
    }
}

ER s5_command(S5_EVENT_KIND command)
{
    S5_MESSAGE message = {0};
    if (command > S5_SAFETY_STOP)
    {
        return E_PAR;
    }
    message.kind = command;
    return tk_snd_mbf(inbox, &message, sizeof(message), TMO_POL);
}

ER s5_request_scan(S5_MODE mode)
{
    return s5_command((S5_EVENT_KIND)(S5_REQUEST_FORWARD + mode));
}

static unsigned int servo_angle; /* last commanded angle, scan task only */

static void hardware_state(unsigned int angle, unsigned int state)
{
    servo_angle = angle;
    tk_loc_mtx(snapshot_mutex, TMO_FEVR);
    snapshot.servo_angle = angle;
    snapshot.scan_state = state;
    tk_unl_mtx(snapshot_mutex);
}

static unsigned int scan_allowed(S5_MODE mode)
{
    S5_HEALTH health;
    s5_get_health(&health);
    return health.control.pending &&
           (mode == S5_FORWARD ? health.control.state == S5_MONITOR
                               : health.control.state == S5_PROFILING &&
                                     health.control.motion_stopped);
}

static void take_angle(S5_SCAN * scan, unsigned int angle, uint32_t began)
{
    S5_SAMPLE values[S5_FILTER_COUNT] = {0};
    unsigned int faults = 0;
    if (!scan_allowed((S5_MODE)scan->mode))
    {
        return;
    }
    unsigned int moved = servo_angle != angle;
    /* Publish busy before changing PWM so no reader sees forward clear
     * during the start of a pan. */
    hardware_state(angle, 1);
    if (s5_sg90_angle(angle) < E_OK)
    {
        scan->faults |= S5_SERVO_ERROR;
        return;
    }
    if (moved)
    {
        tk_dly_tsk(S5_SERVO_SETTLE_MS);
    }
    hardware_state(angle, 2);
    unsigned int count = scan->mode == S5_FORWARD ? 1 : S5_FILTER_COUNT;
    for (unsigned int i = 0; i < count; i++)
    {
        if (!scan_allowed((S5_MODE)scan->mode))
        {
            return;
        }
        if (s5_now_ms() - began > S5_SCAN_MAX_MS)
        {
            scan->faults |= S5_SCAN_TIMEOUT;
            return;
        }
        values[i] = s5_hcsr04_read(angle, &faults);
    }
    scan->samples[scan->count++] = count == 1 ? values[0] : s5_filter(values);
    scan->faults |= faults;
}

static void scan_task(INT stacd, void * exinf)
{
    S5_SCAN coarse = {0};
    S5_REQUEST request;
    ER ready;
    unsigned int init_fault = S5_SCAN_TIMEOUT;
    (void)stacd;
    (void)exinf;
    ready = s5_hcsr04_init();
    if (ready == E_OK)
    {
        init_fault = S5_SERVO_ERROR;
        ready = s5_sg90_init();
    }
    if (ready < E_OK)
    {
        S5_MESSAGE failure = {0};
        failure.kind = S5_SCAN_DONE;
        failure.scan.faults = init_fault;
        tk_snd_mbf(inbox, &failure, sizeof(failure), TMO_FEVR);
    }
    while (1)
    {
        if (tk_rcv_mbf(requests, &request, TMO_FEVR) < E_OK)
        {
            continue;
        }
        S5_MESSAGE result = {0};
        S5_SCAN * scan = &result.scan;
        uint32_t began = s5_now_ms();
        unsigned int angle = S5_SERVO_CENTRE_DEG;
        scan->mode = request.mode;
        result.kind = S5_SCAN_DONE;
        if (ready < E_OK)
        {
            scan->faults = init_fault;
        }
        else if (request.mode == S5_FORWARD)
        {
            coarse.count = 0; /* moving/forward observations invalidate cache */
            take_angle(scan, angle, began);
        }
        else if (request.mode == S5_COARSE)
        {
            const unsigned int angles[S5_COARSE_COUNT] = S5_COARSE_ANGLES;
            for (unsigned int i = 0; i < S5_COARSE_COUNT; i++)
            {
                angle = angles[i];
                take_angle(scan, angle, began);
                if (!scan_allowed(S5_COARSE) || scan->faults & S5_SCAN_TIMEOUT)
                {
                    break;
                }
            }
            coarse = *scan;
        }
        else
        {
            S5_HEALTH h;
            s5_get_health(&h);
            if (coarse.count == S5_COARSE_COUNT && h.control.profile.valid &&
                h.control.profile.detected &&
                s5_now_ms() - h.control.profile.stamp_ms <=
                    S5_PROFILE_MAX_AGE_MS)
            {
                *scan = coarse;
                scan->mode = S5_FINE;
                int centre = (int)h.control.profile.closest_angle;
                int first = centre - S5_FINE_HALF_SPAN_DEG;
                if (first < S5_SERVO_MIN_DEG)
                {
                    first = S5_SERVO_MIN_DEG;
                }
                if (first + 2 * S5_FINE_HALF_SPAN_DEG > S5_SERVO_MAX_DEG)
                {
                    first = S5_SERVO_MAX_DEG - 2 * S5_FINE_HALF_SPAN_DEG;
                }
                for (unsigned int i = 0; i < S5_FINE_COUNT; i++)
                {
                    angle = first + i * S5_FINE_STEP_DEG;
                    take_angle(scan, angle, began);
                    if (!scan_allowed(S5_FINE) ||
                        scan->faults & S5_SCAN_TIMEOUT)
                    {
                        break;
                    }
                }
            }
        }
        if (s5_now_ms() - began > S5_SCAN_MAX_MS)
        {
            scan->faults |= S5_SCAN_TIMEOUT;
        }
        /* Centre on completion/cancellation. Settling still counts as busy. */
        if (ready == E_OK && request.mode != S5_FORWARD)
        {
            s5_sg90_angle(S5_SERVO_CENTRE_DEG);
            hardware_state(S5_SERVO_CENTRE_DEG, 1);
            tk_dly_tsk(S5_SERVO_SETTLE_MS);
        }
        hardware_state(S5_SERVO_CENTRE_DEG, 0);
        /* Blocking copy to the higher-priority consumer; no lost completion. */
        tk_snd_mbf(inbox, &result, sizeof(result), TMO_FEVR);
    }
}

static void publish(const S5_CONTROL * control)
{
    tk_loc_mtx(snapshot_mutex, TMO_FEVR);
    snapshot.control = *control;
    tk_unl_mtx(snapshot_mutex);
}

static void avoidance_task(INT stacd, void * exinf)
{
    S5_CONTROL control = {0};
    S5_MESSAGE message;
    (void)stacd;
    (void)exinf;
    while (1)
    {
        INT size = tk_rcv_mbf(inbox, &message, 50);
        S5_EVENT_KIND event =
            size < E_OK ? S5_TICK : (S5_EVENT_KIND)message.kind;
        s5_transition(&control, event,
                      event == S5_SCAN_DONE ? &message.scan : 0, s5_now_ms());
        publish(&control);
        if (control.scan_due)
        {
            S5_REQUEST request = {0};
            request.mode = control.mode;
            control.scan_due = 0;
            /* Only one outstanding request, including cancellation drain. */
            ER er = tk_snd_mbf(requests, &request, sizeof(request), TMO_POL);
            if (er < E_OK)
            {
                control.pending = 0;
                control.faults |= S5_SCAN_TIMEOUT;
                s5_transition(&control, S5_SAFETY_STOP, 0, s5_now_ms());
                publish(&control);
            }
        }
    }
}

ER subsystem5_init(void)
{
    T_CMTX mutex = {0};
    T_CMBF buffer = {0};
    mutex.mtxatr = TA_INHERIT;
    snapshot_mutex = tk_cre_mtx(&mutex);
    if (snapshot_mutex < E_OK)
    {
        return snapshot_mutex;
    }
    buffer.mbfatr = TA_TFIFO | TA_USERBUF;
    buffer.maxmsz = sizeof(S5_MESSAGE);
    buffer.bufsz = sizeof(inbox_storage);
    buffer.bufptr = inbox_storage;
    inbox = tk_cre_mbf(&buffer);
    if (inbox < E_OK)
    {
        return inbox;
    }
    buffer.maxmsz = sizeof(S5_REQUEST);
    buffer.bufsz = sizeof(request_storage);
    buffer.bufptr = request_storage;
    requests = tk_cre_mbf(&buffer);
    return requests < E_OK ? requests : E_OK;
}

ER subsystem5_start(void)
{
    ID task;
    if (inbox <= 0 || requests <= 0 || snapshot_mutex <= 0)
    {
        return E_OBJ;
    }
    task = app_start_task(avoidance_task, S5_PRI_AVOID, 0, "s5_avoid");
    if (task < E_OK)
    {
        return task;
    }
    task = app_start_task(scan_task, S5_PRI_SCAN, 0, "s5_scan");
    return task < E_OK ? task : E_OK;
}

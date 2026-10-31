/* Pure profile/planning/state logic: no hardware or kernel calls. */
#include "subsystem5.h"

S5_SAMPLE s5_echo_sample(uint32_t echo_us, unsigned int angle, uint32_t now_ms)
{
    S5_SAMPLE sample = {0};
    uint32_t mm = (echo_us * 10U + 29U) / 58U;
    sample.angle_deg = angle;
    sample.stamp_ms = now_ms;
    sample.valid = mm >= S5_MIN_MM && mm <= S5_MAX_MM;
    sample.distance_mm = sample.valid ? mm : 0;
    return sample;
}

S5_SAMPLE s5_filter(S5_SAMPLE values[S5_FILTER_COUNT])
{
    S5_SAMPLE sorted[S5_FILTER_COUNT];
    unsigned int count = 0;
    for (unsigned int i = 0; i < S5_FILTER_COUNT; i++)
    {
        if (values[i].valid)
        {
            unsigned int j = count++;
            while (j > 0 && sorted[j - 1].distance_mm > values[i].distance_mm)
            {
                sorted[j] = sorted[j - 1];
                j--;
            }
            sorted[j] = values[i];
        }
    }
    /* Two valid values: choose the nearer one, rather than optimistic mean. */
    return count ? sorted[(count - 1) / 2] : values[0];
}

unsigned int s5_forward_clear(const S5_SAMPLE * sample,
                              unsigned int servo_angle, uint32_t now_ms)
{
    int offset = (int)sample->angle_deg - S5_SERVO_CENTRE_DEG;
    int current = (int)servo_angle - S5_SERVO_CENTRE_DEG;
    return sample->valid && sample->distance_mm > S5_OBSTACLE_MM &&
           offset >= -S5_FORWARD_TOLERANCE_DEG &&
           offset <= S5_FORWARD_TOLERANCE_DEG &&
           current >= -S5_FORWARD_TOLERANCE_DEG &&
           current <= S5_FORWARD_TOLERANCE_DEG &&
           (uint32_t)(now_ms - sample->stamp_ms) <= S5_FORWARD_MAX_AGE_MS;
}

void s5_make_profile(const S5_SCAN * scan, uint32_t now_ms, S5_PROFILE * out)
{
    S5_SAMPLE sorted[S5_MAX_SAMPLES];
    unsigned int n = 0;
    int closest = -1;
    uint32_t oldest = 0;
    *out = (S5_PROFILE){0};
    out->faults = scan->faults;
    /* Sort, replacing duplicate angles with their later refined sample. */
    for (unsigned int i = 0; i < scan->count; i++)
    {
        S5_SAMPLE s = scan->samples[i];
        unsigned int j = 0;
        while (j < n && sorted[j].angle_deg < s.angle_deg)
        {
            j++;
        }
        if (j == n || sorted[j].angle_deg != s.angle_deg)
        {
            for (unsigned int k = n; k > j; k--)
            {
                sorted[k] = sorted[k - 1];
            }
            n++;
        }
        sorted[j] = s;
    }
    for (unsigned int i = 0; i < n; i++)
    {
        uint32_t age = now_ms - sorted[i].stamp_ms;
        if (age > oldest)
        {
            oldest = age;
        }
        if (!sorted[i].valid)
        {
            continue;
        }
        out->valid_count++;
        if (closest < 0 || sorted[i].distance_mm < sorted[closest].distance_mm)
        {
            closest = (int)i;
        }
    }
    out->stamp_ms = now_ms - oldest;
    out->valid =
        n > 0 && out->valid_count == n && oldest <= S5_PROFILE_MAX_AGE_MS;
    if (closest < 0)
    {
        return;
    }
    out->closest_mm = sorted[closest].distance_mm;
    out->closest_angle = sorted[closest].angle_deg;
    out->detected = out->closest_mm <= S5_OBSTACLE_MM;
    if (!out->detected)
    {
        return;
    }
    /* Relevant sector is the contiguous group containing the closest point.
     * Invalid rays terminate a group; they never count as free space. */
    unsigned int first = (unsigned int)closest;
    unsigned int last = first;
    while (first > 0 && sorted[first - 1].valid &&
           sorted[first - 1].distance_mm <= S5_OBSTACLE_MM)
    {
        first--;
    }
    while (last + 1 < n && sorted[last + 1].valid &&
           sorted[last + 1].distance_mm <= S5_OBSTACLE_MM)
    {
        last++;
    }
    out->right_edge = sorted[first].angle_deg;
    out->left_edge = sorted[last].angle_deg;
    /* 30 degrees is right; 150 is left. Require two rays on each side,
     * including the outer sector. All side rays must be valid. */
    out->right_valid = first >= 2 && sorted[0].angle_deg == S5_SERVO_MIN_DEG;
    out->left_valid =
        n - last - 1 >= 2 && sorted[n - 1].angle_deg == S5_SERVO_MAX_DEG;
    unsigned int right = S5_MAX_MM, left = S5_MAX_MM;
    for (unsigned int i = 0; i < first; i++)
    {
        if (!sorted[i].valid)
        {
            out->right_valid = 0;
        }
        if (sorted[i].distance_mm < right)
        {
            right = sorted[i].distance_mm;
        }
    }
    for (unsigned int i = last + 1; i < n; i++)
    {
        if (!sorted[i].valid)
        {
            out->left_valid = 0;
        }
        if (sorted[i].distance_mm < left)
        {
            left = sorted[i].distance_mm;
        }
    }
    out->right_mm = out->right_valid ? right : 0;
    out->left_mm = out->left_valid ? left : 0;
}

S5_ACTION s5_choose(const S5_PROFILE * p, uint32_t now_ms,
                    unsigned int calibrated)
{
    if (!p->valid || now_ms - p->stamp_ms > S5_PROFILE_MAX_AGE_MS ||
        p->closest_mm <= S5_EMERGENCY_MM || !p->detected || !calibrated)
    {
        return S5_SAFE_STOP;
    }
    unsigned int left = p->left_valid && p->left_mm >= S5_BYPASS_CLEARANCE_MM;
    unsigned int right =
        p->right_valid && p->right_mm >= S5_BYPASS_CLEARANCE_MM;
    /* Odometry route costs are unavailable. Deterministic left tie-break. */
    return left ? S5_BYPASS_LEFT : right ? S5_BYPASS_RIGHT : S5_SAFE_STOP;
}

static void stop(S5_CONTROL * c, unsigned int fault, uint32_t now)
{
    c->faults |= fault;
    c->state = S5_STOPPED;
    c->action = S5_SAFE_STOP;
    c->active = 0;
    c->scan_due = 0;
    c->profile.valid = 0;
    c->entered_ms = now;
}

static void wait_stop(S5_CONTROL * c, uint32_t now)
{
    c->action = S5_STOP_AND_SCAN;
    c->state = S5_WAIT_STOP;
    c->entered_ms = now;
}

static void scan_request(S5_CONTROL * c, S5_MODE mode, uint32_t now)
{
    c->mode = mode;
    c->scan_due = 1;
    c->pending = 1;
    c->entered_ms = now;
}

void s5_transition(S5_CONTROL * c, S5_EVENT_KIND event, const S5_SCAN * scan,
                   uint32_t now)
{
    if (event == S5_SCAN_DONE)
    {
        c->pending = 0;
    }
    if (event == S5_MOTION_MOVING || event == S5_MOTION_FAILED)
    {
        c->motion_stopped = 0;
        c->profile.valid = 0;
    }
    if (event == S5_SAFETY_STOP || event == S5_CANCEL ||
        event == S5_MOTION_FAILED)
    {
        stop(c, event == S5_MOTION_FAILED ? S5_NO_SAFE_PATH : 0, now);
        return;
    }
    if (event == S5_RESET && !c->pending && c->motion_stopped)
    {
        *c = (S5_CONTROL){0};
        c->motion_stopped = 1;
        return;
    }
    if (event == S5_MOTION_STOPPED)
    {
        c->motion_stopped = 1;
    }
    if (c->state == S5_STOPPED) /* needs a reset and a new activation */
    {
        return;
    }
    if (event == S5_ACTIVATE)
    {
        c->active = 1;
        wait_stop(c, now);
    }
    if ((event == S5_REQUEST_COARSE || event == S5_REQUEST_FINE) && !c->pending)
    {
        c->mode = event == S5_REQUEST_FINE ? S5_FINE : S5_COARSE;
        c->active = 1;
        wait_stop(c, now);
    }
    if (event == S5_REQUEST_FORWARD && !c->pending && c->state == S5_MONITOR)
    {
        scan_request(c, S5_FORWARD, now);
    }
    if (c->state == S5_WAIT_STOP && c->motion_stopped && !c->pending)
    {
        c->state = S5_PROFILING;
        scan_request(c, c->mode == S5_FINE ? S5_FINE : S5_COARSE, now);
    }
    if (event == S5_MOTION_MOVING && c->state == S5_PROFILING)
    {
        stop(c, S5_NO_SAFE_PATH, now);
    }
    if (event == S5_SCAN_DONE)
    {
        c->faults |= scan->faults;
        if (scan->faults & (S5_SERVO_ERROR | S5_SCAN_TIMEOUT))
        {
            stop(c, scan->faults, now);
        }
        else if (scan->mode == S5_FORWARD && c->state == S5_MONITOR)
        {
            c->forward = scan->samples[0];
            if (c->forward.valid && c->forward.distance_mm <= S5_EMERGENCY_MM)
            {
                stop(c, S5_NO_SAFE_PATH, now);
            }
            else if (s5_forward_clear(&c->forward, S5_SERVO_CENTRE_DEG, now))
            {
                c->faults &= ~S5_STALE_RANGE;
                c->action = S5_CONTINUE;
                c->scan_retries = 0;
            }
            else
            {
                wait_stop(c, now);
            }
        }
        else if (c->state == S5_PROFILING)
        {
            s5_make_profile(scan, now, &c->profile);
            if (c->profile.closest_mm &&
                c->profile.closest_mm <= S5_EMERGENCY_MM)
            {
                stop(c, S5_NO_SAFE_PATH, now);
            }
            else if (!c->profile.valid)
            {
                if (c->scan_retries < S5_MAX_SCAN_RETRIES)
                {
                    c->scan_retries++;
                    scan_request(c, S5_COARSE, now);
                }
                else
                {
                    stop(c, S5_NO_SAFE_PATH, now);
                }
            }
            else if (scan->mode == S5_COARSE && c->profile.detected)
            {
                scan_request(c, S5_FINE, now);
            }
            else if (!c->profile.detected)
            {
                c->state = S5_MONITOR;
                c->action = S5_STOP_AND_SCAN;
                scan_request(c, S5_FORWARD, now);
            }
            else
            {
                c->state = S5_PLAN;
                c->action = s5_choose(&c->profile, now, S5_BYPASS_CALIBRATED);
                if (c->action == S5_SAFE_STOP)
                {
                    stop(c, S5_NO_SAFE_PATH, now);
                    /* Stopping the plan does not invalidate the completed
                     * stationary profile (including calibration gating). */
                    c->profile.valid = 1;
                }
                else
                {
                    c->state = S5_WAIT_MOTION;
                    c->motion_stopped = 0;
                    c->entered_ms = now;
                }
            }
        }
    }
    if (event == S5_MOTION_COMPLETE && c->state == S5_WAIT_MOTION)
    {
        c->state = S5_LINE_RECOVERY;
        c->action = S5_RECOVERY;
        c->recovery_retries = 0;
        c->entered_ms = now;
    }
    if (event == S5_LINE_FOUND && c->state == S5_LINE_RECOVERY)
    {
        c->state = S5_MONITOR;
        c->action = S5_STOP_AND_SCAN;
        scan_request(c, S5_FORWARD, now);
    }
    if (c->state == S5_LINE_RECOVERY &&
        (event == S5_RECOVERY_FAILED ||
         now - c->entered_ms > S5_RECOVERY_MAX_MS))
    {
        if (c->recovery_retries < S5_MAX_RECOVERY_RETRIES)
        {
            c->recovery_retries++;
            c->entered_ms = now;
        }
        else
        {
            stop(c, S5_RECOVERY_FAULT, now);
        }
    }
    uint32_t age = now - c->entered_ms;
    if ((c->state == S5_WAIT_STOP && age > S5_WAIT_STOP_MS) ||
        (c->state == S5_PROFILING && age > S5_SCAN_MAX_MS) ||
        (c->state == S5_WAIT_MOTION && age > S5_MOTION_MAX_MS))
    {
        stop(c, S5_SCAN_TIMEOUT, now);
    }
    if (c->state == S5_MONITOR && c->action == S5_CONTINUE &&
        !s5_forward_clear(&c->forward, S5_SERVO_CENTRE_DEG, now))
    {
        c->faults |= S5_STALE_RANGE;
        c->action = S5_STOP_AND_SCAN;
    }
    /* Forward checks only while activated/monitoring, never full sweeping. */
    if (c->active && c->state == S5_MONITOR && !c->pending)
    {
        scan_request(c, S5_FORWARD, now);
    }
}

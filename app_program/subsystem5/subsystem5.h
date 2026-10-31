/* Buddy 5 local contracts. Cross-subsystem wiring needs team agreement. */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#ifdef S5_HOST_TEST
#include "s5_host.h"
#else
#include "../app.h"
#endif
#include <stdint.h>
#include "s5_limits.h"

typedef enum
{
    S5_FORWARD,
    S5_COARSE,
    S5_FINE
} S5_MODE;
typedef enum
{
    S5_NONE,
    S5_CONTINUE,
    S5_STOP_AND_SCAN,
    S5_BYPASS_LEFT,
    S5_BYPASS_RIGHT,
    S5_RECOVERY,
    S5_SAFE_STOP
} S5_ACTION;
typedef enum
{
    S5_MONITOR,
    S5_WAIT_STOP,
    S5_PROFILING,
    S5_PLAN,
    S5_WAIT_MOTION,
    S5_LINE_RECOVERY,
    S5_STOPPED
} S5_STATE;
typedef enum
{
    S5_ACTIVATE,
    S5_CANCEL,
    S5_RESET,
    S5_REQUEST_FORWARD,
    S5_REQUEST_COARSE,
    S5_REQUEST_FINE,
    S5_MOTION_STOPPED,
    S5_MOTION_MOVING,
    S5_MOTION_COMPLETE,
    S5_MOTION_FAILED,
    S5_LINE_FOUND,
    S5_RECOVERY_FAILED,
    S5_SAFETY_STOP,
    S5_SCAN_DONE,
    S5_TICK
} S5_EVENT_KIND;
enum
{
    S5_NO_ECHO = 1U << 0,
    S5_ECHO_TIMEOUT = 1U << 1,
    S5_SCAN_TIMEOUT = 1U << 2,
    S5_STALE_RANGE = 1U << 3,
    S5_NO_SAFE_PATH = 1U << 4,
    S5_RECOVERY_FAULT = 1U << 5,
    S5_SERVO_ERROR = 1U << 6
};

/* 32-bit timestamps are wrapping monotonic milliseconds. Owned copies. */
typedef struct
{
    unsigned int distance_mm : 12;
    unsigned int angle_deg : 8;
    unsigned int valid : 1;
    unsigned int stamp_ms : 32;
} S5_SAMPLE;
typedef struct
{
    unsigned int closest_mm : 12;
    unsigned int closest_angle : 8;
    unsigned int right_edge : 8;
    unsigned int detected : 1;
    unsigned int valid : 1;
    unsigned int width_valid : 1;
    unsigned int left_valid : 1;
    unsigned int left_edge : 8;
    unsigned int width_mm : 12;
    unsigned int right_valid : 1;
    unsigned int valid_count : 4;
    unsigned int faults : 7;
    unsigned int left_mm : 12;
    unsigned int right_mm : 12;
    unsigned int stamp_ms : 32; /* oldest contributing measurement */
} S5_PROFILE;
typedef struct
{
    S5_SAMPLE samples[S5_MAX_SAMPLES];
    unsigned int count : 4;
    unsigned int mode : 2;
    unsigned int faults : 7;
} S5_SCAN;
typedef struct
{
    unsigned int state : 3;
    unsigned int action : 3;
    unsigned int scan_retries : 2;
    unsigned int recovery_retries : 2;
    unsigned int faults : 7;
    unsigned int active : 1;
    unsigned int motion_stopped : 1;
    unsigned int pending : 1;
    unsigned int scan_due : 1;
    unsigned int mode : 2;
    unsigned int entered_ms : 32;
    S5_SAMPLE forward;
    S5_PROFILE profile;
} S5_CONTROL;
typedef struct
{
    S5_CONTROL control;
    unsigned int servo_angle : 8;
    unsigned int scan_state : 2;  /* idle, settling, measuring */
    unsigned int front_clear : 1; /* computed at read time */
} S5_HEALTH;

_Static_assert(sizeof(S5_SAMPLE) == 8, "sample ABI");
_Static_assert(sizeof(S5_PROFILE) == 16, "profile ABI");
_Static_assert(sizeof(S5_SCAN) == 100, "scan ABI");
_Static_assert(sizeof(S5_CONTROL) == 32, "control ABI");
_Static_assert(sizeof(S5_HEALTH) == 36, "health ABI");

/* Only avoidance writes control; scanner owns samples and hardware state.
 * All shared snapshot access is under one inheritance mutex. */
ER s5_command(S5_EVENT_KIND command); /* nonblocking copy; E_TMOUT if full */
ER s5_request_scan(S5_MODE mode);
void s5_get_health(S5_HEALTH * out);
uint32_t s5_now_ms(void);

/* Pure functions also used by deterministic tests. */
S5_SAMPLE s5_echo_sample(uint32_t echo_us, unsigned int angle, uint32_t now_ms);
S5_SAMPLE s5_filter(S5_SAMPLE values[S5_FILTER_COUNT]);
void s5_make_profile(const S5_SCAN * scan, uint32_t now_ms, S5_PROFILE * out);
unsigned int s5_forward_clear(const S5_SAMPLE * sample,
                              unsigned int servo_angle, uint32_t now_ms);
S5_ACTION s5_choose(const S5_PROFILE * profile, uint32_t now_ms,
                    unsigned int calibrated);
void s5_transition(S5_CONTROL * control, S5_EVENT_KIND event,
                   const S5_SCAN * scan, uint32_t now_ms);
#endif

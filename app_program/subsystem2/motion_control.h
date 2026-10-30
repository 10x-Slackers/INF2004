/* Buddy 2 API; units and integration details: docs/MOTION_CONTROL.md. */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define MOTION_CONTROL_PERIOD_US 10000u
#define MOTION_CONTROL_MIN_SAMPLE_US (MOTION_CONTROL_PERIOD_US / 2u)
#define MOTION_CONTROL_PACKET_BYTES 36u
#define MOTION_CONTROL_COUNT_BIAS 8388608u
#define MOTION_CONTROL_SPEED_BIAS 2048u
#define MOTION_CONTROL_STALL 1u
#define MOTION_CONTROL_TIMEOUT 2u
#define MOTION_CONTROL_ENCODER 4u
#define MOTION_CONTROL_TIMING 8u
#define MOTION_CONTROL_RANGE 16u
#define MOTION_CONTROL_SAFETY 32u
#define MOTION_CONTROL_INIT 64u
#define MOTION_CONTROL_STALE 128u

/* Constants avoid full-width enum storage for commands and status. */
#define MOTION_CONTROL_IDLE 0u
#define MOTION_CONTROL_RUNNING 1u
#define MOTION_CONTROL_COMPLETE 2u
#define MOTION_CONTROL_CANCELLED 3u
#define MOTION_CONTROL_FAILED 4u

/* Positive dimensions must be measured; confirmed defaults to zero. */
typedef struct
{
    unsigned int circumference_mm : 10;
    unsigned int track_mm : 10;
    unsigned int counts_per_rev : 16;
    unsigned int kp_q8 : 12;
    unsigned int ki_q8 : 12;
    unsigned int kd_q8 : 12;
    unsigned int speed_mm_s : 10;
    unsigned int duty_limit : 10;
    unsigned int timeout_ms : 16;
    unsigned int stall_ms : 14;
    unsigned int motor_left_reverse : 1;
    unsigned int motor_right_reverse : 1;
    unsigned int encoder_left_reverse : 1;
    unsigned int encoder_right_reverse : 1;
    unsigned int confirmed : 1;
    /* Zero selects legacy shared CPR; nonzero overrides that wheel. */
    unsigned int left_counts_per_rev : 16;
    unsigned int right_counts_per_rev : 16;
} motion_control_config_t;

typedef struct
{
    unsigned int state : 3;
    unsigned int armed : 1;
    unsigned int faults : 8;
    unsigned int command : 3;
    unsigned int left_count : 24;
    unsigned int right_count : 24;
    unsigned int left_mm : 24;
    unsigned int right_mm : 24;
    unsigned int average_mm : 24;
    unsigned int left_mm_s : 12;
    unsigned int right_mm_s : 12;
    uint32_t timestamp_us;
    uint32_t max_execution_us;
    uint32_t max_period_us;
    uint32_t missed_periods;
} motion_control_status_t;

/* Startup only. Command acceptance is nonblocking; check the bool result. */
void motion_control_init(void);
bool motion_control_configure(const motion_control_config_t * p_config);
void motion_control_get_config(motion_control_config_t * p_config);
bool motion_control_set_speed(uint32_t speed_mm_s);
bool motion_control_arm(void);
void motion_control_disarm(void);
void motion_control_stop(void);
void motion_control_safety_stop(uint32_t faults);
bool motion_control_reset_faults(void);
bool motion_control_move_forward(uint32_t distance_mm);
bool motion_control_move_backward(uint32_t distance_mm);
bool motion_control_turn_left(uint32_t angle_ddeg);
bool motion_control_turn_right(uint32_t angle_ddeg);
/* Wheels lifted: one wheel, signed permille, 1..1000 ms; <=200 permille. */
bool motion_control_test_motor(uint32_t wheel, int32_t duty,
                               uint32_t duration_ms);
void motion_control_get_status(motion_control_status_t * p_status);
/* Big-endian bit stream; reserved last bit is zero. No raw structs on wire. */
void motion_control_pack(const motion_control_status_t * p_status,
                         uint8_t * p_packet);
bool motion_control_unpack(motion_control_status_t * p_status,
                           const uint8_t * p_packet, uint32_t length);
int32_t motion_control_decode_count(uint32_t encoded);
int32_t motion_control_decode_speed(uint32_t encoded);
/* Task-only. Entry time is retained for ABI; driver sample time is used. */
void motion_control_tick(uint32_t now_us);
void motion_control_record_execution(uint32_t elapsed_us);

#endif /* MOTION_CONTROL_H */
/* End of file. */

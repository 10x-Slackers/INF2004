/** @file motion_control.h
 *
 * @brief Motion control API. Every function except motion_control_tick()
 * may be called from any task.
 *
 * Moves and turns run in the background. Start one, then poll
 * motion_control_get_status() until state is COMPLETE or FAILED. A new
 * command replaces the one running. Units: docs/MOTION_CONTROL.md.
 */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define MOTION_CONTROL_PERIOD_MS 10u /* motion_control_tick() period. */

/* status.state */
#define MOTION_CONTROL_IDLE     0u /* Stopped, or a drive target expired. */
#define MOTION_CONTROL_RUNNING  1u
#define MOTION_CONTROL_COMPLETE 2u
#define MOTION_CONTROL_FAILED   3u /* See APP_FAULT_MOTION (app_fault.h). */

/* status.command */
#define MOTION_CONTROL_CMD_NONE     0u
#define MOTION_CONTROL_CMD_FORWARD  1u
#define MOTION_CONTROL_CMD_BACKWARD 2u
#define MOTION_CONTROL_CMD_LEFT     3u
#define MOTION_CONTROL_CMD_RIGHT    4u
#define MOTION_CONTROL_CMD_DRIVE    5u

/* Command limits. */
#define MOTION_CONTROL_MAX_DISTANCE_MM 5000u
#define MOTION_CONTROL_MAX_ANGLE_DEG   360u
#define MOTION_CONTROL_MAX_SPEED_MM_S  1000

#define MOTION_CONTROL_PACKET_BYTES 12u

/* Motion status. Forward is positive for every field. */
typedef struct
{
    unsigned int distance_mm : 24; /* Distance rolled, wraps. */
    unsigned int state : 2;
    unsigned int command : 3;
    unsigned int left_count : 16; /* Encoder edges, wraps. */
    unsigned int right_count : 16;
    signed int left_speed : 11; /* mm/s. */
    signed int right_speed : 11;
    unsigned int heading_deg : 9; /* 0..359, anticlockwise from power-up. */
} motion_control_status_t;

void motion_control_init(void);
bool motion_control_move_forward(uint32_t distance_mm);
bool motion_control_move_backward(uint32_t distance_mm);
bool motion_control_turn_left(uint32_t angle_deg);
bool motion_control_turn_right(uint32_t angle_deg);
bool motion_control_drive(int32_t left_mm_s, int32_t right_mm_s);
void motion_control_stop(void);
void motion_control_latch(bool b_latched);
bool motion_control_latched(void);
bool motion_control_set_speed(uint32_t speed_mm_s);
void motion_control_get_status(motion_control_status_t * p_status);
void motion_control_pack(const motion_control_status_t * p_status,
                         uint8_t * p_packet);
void motion_control_tick(uint32_t elapsed_ms);

#endif /* MOTION_CONTROL_H */

/*** end of file ***/

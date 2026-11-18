/** @file app_fault.h
 *
 * @brief The shared fault bitmask. Changed only by team agreement, like
 * app.h. Free of kernel headers, so host tests can stub the three calls.
 */

#ifndef APP_FAULT_H
#define APP_FAULT_H

#include <stdint.h>

/* The shared fault bitmask, one bit per fault, held in a kernel event flag
 * so every set and clear is atomic across both cores and from interrupts.
 * Each owner sets and clears only its own bits. */

/* S2, cleared by the next motion command. */
#define APP_FAULT_STALL_LEFT  (1u << 0) /* Driven 1 s with no edge. */
#define APP_FAULT_STALL_RIGHT (1u << 1) /* Driven 1 s with no edge. */
#define APP_FAULT_ENCODER     (1u << 2) /* Missed phase; still runs. */
/* S4, latched until reset. */
#define APP_FAULT_UPSIDE_DOWN (1u << 3) /* Picked up and turned over. */
#define APP_FAULT_TILT        (1u << 4) /* Over 25 degrees for 0.5 s. */
/* S5: no echo is updated by every scan; the rest are cleared by
 * obstacle_avoidance_start(). */
#define APP_FAULT_NO_ECHO       (1u << 5) /* Last scan had no echo. */
#define APP_FAULT_AVOID_SENSOR  (1u << 6) /* No echo, or no scan. */
#define APP_FAULT_AVOID_BLOCKED (1u << 7) /* Neither side open. */
#define APP_FAULT_AVOID_MOTION  (1u << 8) /* Move failed or hung. */
/* S1. */
#define APP_FAULT_WIFI (1u << 9) /* Wi-Fi or MQTT lost. */
/* S4, cleared once Z settles. */
#define APP_FAULT_LIFTED (1u << 10) /* Lifted quickly. */

#define APP_FAULT_MOTION                                                       \
    (APP_FAULT_STALL_LEFT | APP_FAULT_STALL_RIGHT | APP_FAULT_ENCODER)
#define APP_FAULT_IMU                                                          \
    (APP_FAULT_UPSIDE_DOWN | APP_FAULT_TILT | APP_FAULT_LIFTED)
#define APP_FAULT_AVOID                                                        \
    (APP_FAULT_AVOID_SENSOR | APP_FAULT_AVOID_BLOCKED | APP_FAULT_AVOID_MOTION)

void app_fault_set(uint32_t bits);
void app_fault_clear(uint32_t bits);
uint32_t app_faults(void);

#endif /* APP_FAULT_H */

/*** end of file ***/

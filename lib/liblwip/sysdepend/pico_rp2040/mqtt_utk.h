/*
 *	mqtt_utk.h
 *	MQTT bridge for micro T-Kernel (RP2040, NO_SYS=1)
 *	Plain C only: cyw43_utk.c and the application both include it.
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdbool.h>
#include <stdint.h>

/* Longest message the application queues: a topic number, then the payload. */
#define MQTT_UTK_MSG_MAX	13

/* Topic numbers, the first byte of a queued message (docs/TELEMETRY.md). */
#define MQTT_UTK_TOPIC_TELEMETRY	0u
#define MQTT_UTK_TOPIC_MOTION		1u
#define MQTT_UTK_TOPIC_SCAN		2u
#define MQTT_UTK_TOPIC_HEARTBEAT	3u

/* The payload of a message on csc13/map: 3 bits per letter, A to Z. */
#define MQTT_UTK_MAP_BYTES	10

/* Call from cyw43_utk_task() on every poll while the link is up. The first
 * call connects; every call publishes what the application has queued. */
void mqtt_utk_poll(void);

/* The application's end, in subsystem1.c. Called from cyw43_utk_task()
 * only; none may block. */

/* Copies the next queued message to msg (MQTT_UTK_MSG_MAX bytes) and returns
 * its length; negative when nothing is queued. msg[0] is a topic number. */
int32_t mqtt_utk_app_take(uint8_t *msg);

/* The one-byte payload of a message on csc13/cmd. */
void mqtt_utk_app_command(uint8_t cmd);

/* The MQTT_UTK_MAP_BYTES payload of a message on csc13/map. */
void mqtt_utk_app_map(const uint8_t *map);

/* The broker accepted the connection (true), or it failed or dropped. */
void mqtt_utk_app_online(bool online);

#endif /* MQTT_UTK_H */

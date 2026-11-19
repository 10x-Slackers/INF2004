/*
 *	mqtt_utk.h
 *	MQTT bridge for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Plain C only: cyw43_utk.c and the application both include it, and
 *	neither side's headers suit the other (see mqtt_utk.c).
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdbool.h>
#include <stdint.h>

/* Longest message the application queues: a topic number, then the payload. */
#define MQTT_UTK_MSG_MAX	13

/*
 * Call from cyw43_utk_task() on every poll while the link is CYW43_LINK_UP
 * (see cyw43_utk.c). The first call connects to MQTT_BROKER_IP:
 * MQTT_BROKER_PORT (config/wifi_secrets.h); every call publishes what the
 * application has queued.
 */
void mqtt_utk_poll(void);

/*
 * The application's end of the bridge, defined in
 * app_program/subsystem1/subsystem1.c. All three are called from
 * cyw43_utk_task() only, and none may block.
 */

/* Copies the next queued message to msg (MQTT_UTK_MSG_MAX bytes) and returns
 * its length; negative when nothing is queued. msg[0] is the topic number:
 * 0 telemetry, 1 motion, 2 scan, 3 heartbeat. */
int32_t mqtt_utk_app_take(uint8_t *msg);

/* The one-byte payload of a message on csc13/cmd. */
void mqtt_utk_app_command(uint8_t cmd);

/* The broker accepted the connection (true), or it failed or dropped. */
void mqtt_utk_app_online(bool online);

#endif /* MQTT_UTK_H */

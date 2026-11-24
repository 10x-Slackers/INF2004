/*
 *	mqtt_utk.c
 *	MQTT bridge for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Publishes the messages the application queues and passes it the
 *	commands from csc13/cmd and the barcode actions from csc13/map
 *	(docs/TELEMETRY.md). Runs only in
 *	cyw43_utk_task(), which owns lwIP; its timers ride that task's
 *	sys_check_timeouts(). No kernel headers: kernel calls go through the
 *	mqtt_utk_app_...() functions.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>
#include <string.h>

#define MQTT_UTK_KEEPALIVE_S	60
#define MQTT_UTK_RETRY_MS	3000
#define MQTT_UTK_CMD_TOPIC	"csc13/cmd"
#define MQTT_UTK_MAP_TOPIC	"csc13/map"

/* Indexed by the topic number that starts each queued message. */
static const char *const topics[] = {
	[MQTT_UTK_TOPIC_TELEMETRY] = "csc13/telemetry",
	[MQTT_UTK_TOPIC_MOTION] = "csc13/motion",
	[MQTT_UTK_TOPIC_SCAN] = "csc13/scan",
	[MQTT_UTK_TOPIC_HEARTBEAT] = "csc13/heartbeat"
};

static mqtt_client_t	*client;
static u32_t		rx_len;			/* Payload size of the arriving publish. */
static bool		rx_map;			/* It is on MQTT_UTK_MAP_TOPIC, not the command topic. */
static u8_t		tx[MQTT_UTK_MSG_MAX];	/* A message taken but not yet published. */
static int32_t		tx_len;			/* Its length; 0 or less when there is none. */

static void try_connect(void *arg);

/* Only the command and map topics are subscribed. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	(void)arg;
	rx_len = tot_len;
	rx_map = (strcmp(topic, MQTT_UTK_MAP_TOPIC) == 0);
}

/* Both payloads are short enough to arrive in one call. */
static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	(void)arg;
	(void)flags;
	if(!rx_map && rx_len == 1) {
		mqtt_utk_app_command(data[0]);
	} else if(rx_map && rx_len == MQTT_UTK_MAP_BYTES && len == rx_len) {
		mqtt_utk_app_map(data);
	}
}

/* arg is the topic name. */
static void subscribe_cb(void *arg, err_t err)
{
	tm_printf((UB*)"mqtt: subscribe to %s, err=%d\n", (const char *)arg, (int)err);
}

/* Once per attempt (accepted or failed), and again if it drops later. */
static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)arg;

	mqtt_utk_app_online(status == MQTT_CONNECT_ACCEPTED);
	if(status == MQTT_CONNECT_ACCEPTED) {
		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");
		/* Drop what was queued while offline: it is stale. */
		tx_len = 0;
		while(mqtt_utk_app_take(tx) > 0) {
		}
		/* The broker then sends the retained map, if there is one. */
		if(mqtt_subscribe(c, MQTT_UTK_CMD_TOPIC, 0 /* qos */, subscribe_cb, MQTT_UTK_CMD_TOPIC) != ERR_OK ||
		   mqtt_subscribe(c, MQTT_UTK_MAP_TOPIC, 0 /* qos */, subscribe_cb, MQTT_UTK_MAP_TOPIC) != ERR_OK) {
			tm_printf((UB*)"mqtt: subscribe failed\n");
		}
	} else {
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, try_connect, NULL);
	}
}

/* One connection attempt; also the retry timer, armed once per failure. */
static void try_connect(void *arg)
{
	struct mqtt_connect_client_info_t	info = {0};
	ip_addr_t				broker;
	err_t					err;

	(void)arg;
	info.client_id = MQTT_CLIENT_ID;
	info.keep_alive = MQTT_UTK_KEEPALIVE_S;
	if(!ipaddr_aton(MQTT_BROKER_IP, &broker)) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP " MQTT_BROKER_IP "\n");
		return;
	}

	err = mqtt_client_connect(client, &broker, MQTT_BROKER_PORT,
				  connection_cb, NULL, &info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d, retrying in %ds\n",
			  (int)err, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, try_connect, NULL);
	}
}

void mqtt_utk_poll(void)
{
	if(client == NULL) {
		client = mqtt_client_new();
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);
		try_connect(NULL);
	}

	/* Publish until the queue is empty or mqtt_publish() is full; a refused
	 * message stays in tx for the next poll. */
	while(mqtt_client_is_connected(client)) {
		if(tx_len <= 0) {
			tx_len = mqtt_utk_app_take(tx);
		}
		if(tx_len <= 0 || mqtt_publish(client, topics[tx[0]], &tx[1],
					       (u16_t)(tx_len - 1), 0 /* qos */,
					       0 /* retain */, NULL, NULL) != ERR_OK) {
			break;
		}
		tx_len = 0;
	}
}

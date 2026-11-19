/*
 *	mqtt_utk.c
 *	MQTT bridge for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Publishes the messages the application queues, and passes it the
 *	commands that arrive on csc13/cmd (docs/TELEMETRY.md).
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_poll() on every poll
 *	while the link is up. mqtt.c's keepalive timers and the retry timer
 *	below ride the sys_check_timeouts() polling that task already does for
 *	DHCP and TCP -- nothing runs on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 *	The kernel calls it needs are behind the mqtt_utk_app_...() functions.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_KEEPALIVE_S	60
#define MQTT_UTK_RETRY_MS	3000
#define MQTT_UTK_CMD_TOPIC	"csc13/cmd"

/* Indexed by the topic number that starts each queued message. */
static const char *const topics[] = {
	[MQTT_UTK_TOPIC_TELEMETRY] = "csc13/telemetry",
	[MQTT_UTK_TOPIC_MOTION] = "csc13/motion",
	[MQTT_UTK_TOPIC_SCAN] = "csc13/scan",
	[MQTT_UTK_TOPIC_HEARTBEAT] = "csc13/heartbeat"
};

static mqtt_client_t	*client;
static u32_t		rx_len;			/* Payload size of the arriving publish. */
static u8_t		tx[MQTT_UTK_MSG_MAX];	/* A message taken but not yet published. */
static int32_t		tx_len;			/* Its length; 0 or less when there is none. */

static void try_connect(void *arg);

/* Only MQTT_UTK_CMD_TOPIC is subscribed, so every arriving publish is a
 * command. One is a single byte; any other size is ignored. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	(void)arg;
	(void)topic;
	rx_len = tot_len;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	(void)arg;
	(void)len;
	(void)flags;
	if(rx_len == 1) {
		mqtt_utk_app_command(data[0]);
	}
}

/* mqtt.c calls this once per connection attempt that gets as far as TCP:
 * accepted, or failed. It calls it again when an accepted connection drops. */
static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)arg;

	mqtt_utk_app_online(status == MQTT_CONNECT_ACCEPTED);
	if(status == MQTT_CONNECT_ACCEPTED) {
		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");
		if(mqtt_subscribe(c, MQTT_UTK_CMD_TOPIC, 0 /* qos */, NULL, NULL) != ERR_OK) {
			tm_printf((UB*)"mqtt: subscribe to " MQTT_UTK_CMD_TOPIC " failed\n");
		}
	} else {
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, try_connect, NULL);
	}
}

/* Starts one connection attempt. Also the retry timer's callback: the timer
 * is armed only where an attempt has just failed, here or in connection_cb(),
 * and each attempt fails at most once, so one timer is armed at a time and
 * the client is always disconnected when it fires. */
static void try_connect(void *arg)
{
	struct mqtt_connect_client_info_t	info = {0};
	ip_addr_t				broker;
	err_t					err;

	(void)arg;
	info.client_id = MQTT_CLIENT_ID;
	info.keep_alive = MQTT_UTK_KEEPALIVE_S;
	(void)ipaddr_aton(MQTT_BROKER_IP, &broker);

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

	/* Publish until the queue is empty or mqtt_publish() refuses: it returns
	 * ERR_MEM while its request pool or output buffer is full. That message
	 * stays in tx for the next poll. */
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

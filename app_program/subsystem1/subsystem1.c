/** @file subsystem1.c
 *
 * @brief Subsystem 1 (Wi-Fi, command and telemetry): the Telemetry task and
 * the application's end of the MQTT bridge (docs/TELEMETRY.md).
 *
 * Only the kernel's Wi-Fi service task may call MQTT. The Telemetry task
 * therefore queues each payload in a message buffer, and the service takes
 * it from there to publish. The service calls the mqtt_utk_app_...()
 * functions at the end of this file; mqtt_utk.h declares and names them.
 * Without MQTT=1 only the command flag is built: nothing sets it.
 */

#include "subsystem1.h"
#if TM_MQTT
#include "mqtt_utk.h"
#include "payload.h"
#include "../shared/vehicle_controller.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem3/barcode_decoder.h"
#include "../subsystem3/line_following.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/obstacle_avoidance.h"
#include "../subsystem5/ultrasonic_scan.h"

#define S1_PERIOD_MS        200u /* telemetry and motion. */
#define S1_HEARTBEAT_CYCLES 5u   /* heartbeat every 1 s. */
#define S1_HEARTBEAT_BYTES  2u   /* A 16-bit count. */
#define S1_QUEUE_BYTES      128  /* About three cycles of messages. */

/* Topic number, the first byte of a queued message. */
#define S1_TOPIC_TELEMETRY 0u
#define S1_TOPIC_MOTION    1u
#define S1_TOPIC_SCAN      2u
#define S1_TOPIC_HEARTBEAT 3u

#if (MQTT_UTK_MSG_MAX < (1u + MOTION_CONTROL_PACKET_BYTES)) ||                 \
    (MQTT_UTK_MSG_MAX < (1u + PAYLOAD_TELEMETRY_BYTES))
#error "MQTT_UTK_MSG_MAX is smaller than a queued message"
#endif

static ID gh_s1_queue; /* Message buffer: Telemetry task to Wi-Fi service. */

static void s1_telemetry_task(INT stacd, void * p_exinf);
static void s1_pack_telemetry(const obstacle_avoidance_status_t * p_avoid,
                              const ultrasonic_scan_t * p_scan,
                              uint8_t * p_payload);
static void s1_send(uint8_t * p_message, uint8_t topic, uint32_t bytes);
#endif /* TM_MQTT */

ID gh_s1_command_flag;

/*!
 * @brief Create the command flag and, with MQTT, the message queue.
 */
ER subsystem1_init(void)
{
    T_CFLG flag = {0};
#if TM_MQTT
    T_CMBF queue = {0};

    queue.bufsz = S1_QUEUE_BYTES;
    queue.maxmsz = MQTT_UTK_MSG_MAX;
    gh_s1_queue = tk_cre_mbf(&queue);
    app_check(gh_s1_queue, "s1 queue");
    app_fault_set(APP_FAULT_WIFI); /* Until the broker accepts us. */
#endif
    gh_s1_command_flag = tk_cre_flg(&flag);
    return (gh_s1_command_flag < E_OK) ? gh_s1_command_flag : E_OK;
}

/*!
 * @brief Start the Telemetry task, in MQTT builds only.
 */
ER subsystem1_start(void)
{
#if TM_MQTT
    /* FP is the kernel's generic task pointer type. */
    ID const h_task = app_start_task((FP)s1_telemetry_task, S1_PRI_TELEMETRY,
                                     0u, "s1_telemetry");

    return (h_task < E_OK) ? h_task : E_OK;
#else
    return E_OK;
#endif
}

#if TM_MQTT
/*!
 * @brief Telemetry task: every 200 ms queue telemetry and motion; a scan
 * when avoidance has finished a profile; a heartbeat every fifth cycle.
 */
static void s1_telemetry_task(INT stacd, void * p_exinf)
{
    motion_control_status_t motion = {0};
    obstacle_avoidance_status_t avoid = {0};
    ultrasonic_scan_t scan = {0};
    uint8_t message[MQTT_UTK_MSG_MAX] = {0}; /* Topic number, payload. */
    uint16_t heartbeats = 0u;                /* Heartbeat count, wraps. */
    uint8_t cycles = 0u;                     /* Since a heartbeat, 0..4. */
    bool b_profiling = false;                /* PROFILE last cycle. */

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_dly_tsk(S1_PERIOD_MS);
        motion_control_get_status(&motion);
        obstacle_avoidance_get_status(&avoid);
        ultrasonic_scan_get(&scan);

        s1_pack_telemetry(&avoid, &scan, &message[1]);
        s1_send(message, S1_TOPIC_TELEMETRY, PAYLOAD_TELEMETRY_BYTES);
        motion_control_pack(&motion, &message[1]);
        s1_send(message, S1_TOPIC_MOTION, MOTION_CONTROL_PACKET_BYTES);

        if (b_profiling && (OBSTACLE_AVOIDANCE_PROFILE != avoid.state))
        {
            payload_pack_scan(&scan.profile, &message[1]);
            s1_send(message, S1_TOPIC_SCAN, PAYLOAD_SCAN_BYTES);
        }
        b_profiling = (OBSTACLE_AVOIDANCE_PROFILE == avoid.state);

        if (0u == cycles)
        {
            /* Most significant byte first, like every payload. */
            message[1] = (uint8_t)(heartbeats >> 8);
            message[2] = (uint8_t)(heartbeats & 0xFFu);
            s1_send(message, S1_TOPIC_HEARTBEAT, S1_HEARTBEAT_BYTES);
            heartbeats++;
        }
        /* 0..4 fits uint8_t. */
        cycles = (uint8_t)((cycles + 1u) % S1_HEARTBEAT_CYCLES);
    }
}

/*!
 * @brief Read every telemetry source and pack the telemetry payload.
 *
 * @param[in]  p_avoid   Avoidance status, already read by the task.
 * @param[in]  p_scan    Scan results, already read by the task.
 * @param[out] p_payload PAYLOAD_TELEMETRY_BYTES bytes.
 */
static void s1_pack_telemetry(const obstacle_avoidance_status_t * p_avoid,
                              const ultrasonic_scan_t * p_scan,
                              uint8_t * p_payload)
{
    payload_telemetry_t telemetry = {0};
    line_following_t line = {0};
    barcode_decoder_t barcode = {0};
    s4_motion_t motion_class = {0};
    s4_tilt_t tilt = {0};
    s4_hump_t hump = {0};

    line_following_get(&line);
    barcode_decoder_get(&barcode);
    s4_get_motion(&motion_class);
    s4_get_tilt(&tilt);
    s4_get_hump(&hump);

    telemetry.state = vehicle_controller_state();
    telemetry.latched = motion_control_latched() ? 1u : 0u;
    telemetry.line = line.bits;
    telemetry.junction = line.b_junction;
    telemetry.following = line.b_following;
    telemetry.barcode = barcode.command;
    telemetry.barcodes = barcode.count;
    telemetry.motion_class = motion_class.cls;
    telemetry.pitch = tilt.pitch;
    telemetry.roll = tilt.roll;
    telemetry.hump_max = hump.max;
    telemetry.humps = hump.count;
    telemetry.on_hump = hump.on;
    telemetry.avoid_state = p_avoid->state;
    telemetry.avoid_side = p_avoid->side;
    telemetry.bypasses = p_avoid->bypasses;
    telemetry.forward_mm = p_scan->forward_mm;
    telemetry.faults = app_faults();
    payload_pack_telemetry(&telemetry, p_payload);
}

/*!
 * @brief Queue a message for the Wi-Fi service. It is dropped if the queue
 * is full, so a slow network never blocks the task.
 *
 * @param[in,out] p_message The payload from byte 1; byte 0 is set here.
 * @param[in]     topic     S1_TOPIC_...
 * @param[in]     bytes     Payload length.
 */
static void s1_send(uint8_t * p_message, uint8_t topic, uint32_t bytes)
{
    p_message[0] = topic;
    /* (INT): the kernel's size type; a message is at most 13 bytes. */
    (void)tk_snd_mbf(gh_s1_queue, p_message, (INT)(1u + bytes), TMO_POL);
}

/*!
 * @brief Give the Wi-Fi service the next queued message to publish.
 *
 * @param[out] p_message Topic number, then the payload: MQTT_UTK_MSG_MAX
 * bytes at most.
 * @return The message's length in bytes; negative if the queue is empty.
 */
int32_t mqtt_utk_app_take(uint8_t * p_message)
{
    return tk_rcv_mbf(gh_s1_queue, p_message, TMO_POL);
}

/*!
 * @brief The Wi-Fi service received a command byte: pass its command bits
 * to the Vehicle Controller.
 */
void mqtt_utk_app_command(uint8_t command)
{
    (void)tk_set_flg(gh_s1_command_flag, command & S1_CMD_ALL);
}

/*!
 * @brief The broker connection came up or went down.
 */
void mqtt_utk_app_online(bool b_online)
{
    if (b_online)
    {
        app_fault_clear(APP_FAULT_WIFI);
    }
    else
    {
        app_fault_set(APP_FAULT_WIFI);
    }
}
#endif /* TM_MQTT */

/*** end of file ***/

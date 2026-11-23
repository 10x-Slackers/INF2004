/** @file payload.h
 *
 * @brief The MQTT payloads subsystem 1 packs itself: telemetry and scan
 * (docs/TELEMETRY.md). Each is one bit stream, most significant bit first,
 * in the layout of tools/supervisor/payloads.py. No kernel headers,
 * so the host test builds it on its own.
 */

#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <stdint.h>
#include "../subsystem5/scan_profile.h"

#define PAYLOAD_TELEMETRY_BYTES 11u  /* 87 bits. */
#define PAYLOAD_SCAN_BYTES      9u   /* 68 bits. */
#define PAYLOAD_PITCH_OFFSET    1024 /* Added to pitch and roll when sent. */

/* The telemetry fields in payload order, each in the bits it is sent in.
 * docs/TELEMETRY.md names the source of each. */
typedef struct
{
    unsigned int state : 3;
    unsigned int latched : 1;
    unsigned int line : 2;
    unsigned int junction : 1;
    unsigned int following : 1;
    unsigned int barcode : 3;
    unsigned int barcodes : 4;
    unsigned int motion_class : 3;
    signed int pitch : 11;
    signed int roll : 11;
    unsigned int hump_max : 6;
    unsigned int humps : 4;
    unsigned int on_hump : 1;
    unsigned int avoid_state : 3;
    unsigned int avoid_side : 2;
    unsigned int bypasses : 4;
    unsigned int forward_mm : 12;
    unsigned int faults : 15;
} payload_telemetry_t;

void payload_pack_telemetry(const payload_telemetry_t * p_telemetry,
                            uint8_t * p_payload);
void payload_pack_scan(const scan_profile_t * p_profile, uint8_t * p_payload);

#endif /* PAYLOAD_H */

/*** end of file ***/

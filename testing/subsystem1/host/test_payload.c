/** @file test_payload.c
 *
 * @brief Host test of payload.c: the C packers must give the bytes that
 * payloads.pack() gives for the same values (C_TELEMETRY_BYTES and
 * C_SCAN_BYTES in payloads.py, which S1-U01 checks; it says why these
 * values). Build and run as in S1-U02 of
 * testing/subsystem1/TEST_LOG.md.
 */

#include "../../../app_program/subsystem1/payload.h"
#include <stdio.h>
#include <string.h>

/* Print one result, with the bytes packed. Returns 1 for a failure. */
static int test_check(const char * p_text, const uint8_t * p_packed,
                      const uint8_t * p_expected, size_t bytes)
{
    int const b_pass = (0 == memcmp(p_packed, p_expected, bytes));
    size_t byte = 0u;

    printf("%s  %s matches payloads.pack(): ", b_pass ? "PASS" : "FAIL",
           p_text);
    for (byte = 0u; byte < bytes; byte++)
    {
        printf("%02x", p_packed[byte]);
    }
    printf("\n");
    return b_pass ? 0 : 1;
}

int main(void)
{
    static const uint8_t telemetry_bytes[PAYLOAD_TELEMETRY_BYTES] = {
        0xbbu, 0x93u, 0x3cu, 0x20u, 0x7cu, 0xaau,
        0xb2u, 0x9fu, 0xa0u, 0x90u, 0x02u};
    static const uint8_t scan_bytes[PAYLOAD_SCAN_BYTES] = {
        0x83u, 0x48u, 0x78u, 0x28u, 0x7cu, 0x8fu, 0xa0u, 0x80u, 0x20u};
    const payload_telemetry_t telemetry = {.state = 5u,
                                           .latched = 1u,
                                           .line = 2u,
                                           .junction = 1u,
                                           .following = 1u,
                                           .barcode = 4u,
                                           .barcodes = 9u,
                                           .motion_class = 4u,
                                           .pitch = 900,
                                           .roll = -900,
                                           .hump_max = 42u,
                                           .humps = 10u,
                                           .on_hump = 1u,
                                           .avoid_state = 4u,
                                           .avoid_side = 2u,
                                           .bypasses = 9u,
                                           .forward_mm = 4000u,
                                           .faults = 0x4801u};
    const scan_profile_t profile = {.closest_mm = 2100u,
                                    .closest_deg = 135u,
                                    .right_deg = 130u,
                                    .left_deg = 135u,
                                    .width_mm = 200u,
                                    .left_mm = 4000u,
                                    .right_mm = 2050u};
    uint8_t payload[PAYLOAD_TELEMETRY_BYTES];
    int failures = 0;

    payload_pack_telemetry(&telemetry, payload);
    failures += test_check("telemetry", payload, telemetry_bytes,
                           sizeof(telemetry_bytes));
    payload_pack_scan(&profile, payload);
    failures += test_check("scan", payload, scan_bytes, sizeof(scan_bytes));

    printf("%s\n", (0 == failures) ? "ALL PASS" : "FAILED");
    return failures;
}

/*** end of file ***/

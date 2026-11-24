/** @file payload.c
 *
 * @brief Packs the telemetry and scan MQTT payloads and unpacks the map
 * payload, with explicit shifts.
 */

#include "payload.h"
#include <string.h>

static void payload_put(uint8_t * p_payload, uint32_t * p_bit, uint32_t value,
                        uint32_t bits);

/*!
 * @brief Pack the telemetry payload.
 *
 * @param[out] p_payload PAYLOAD_TELEMETRY_BYTES bytes.
 */
void payload_pack_telemetry(const payload_telemetry_t * p_telemetry,
                            uint8_t * p_payload)
{
    uint32_t bit = 0u;

    memset(p_payload, 0, PAYLOAD_TELEMETRY_BYTES);
    payload_put(p_payload, &bit, p_telemetry->state, 3u);
    payload_put(p_payload, &bit, p_telemetry->latched, 1u);
    payload_put(p_payload, &bit, p_telemetry->line, 2u);
    payload_put(p_payload, &bit, p_telemetry->junction, 1u);
    payload_put(p_payload, &bit, p_telemetry->following, 1u);
    payload_put(p_payload, &bit, p_telemetry->barcode, 3u);
    payload_put(p_payload, &bit, p_telemetry->barcodes, 4u);
    payload_put(p_payload, &bit, p_telemetry->motion_class, 3u);
    /* Pitch and roll are -900..900, so the offset values are positive. */
    payload_put(p_payload, &bit,
                (uint32_t)(p_telemetry->pitch + PAYLOAD_PITCH_OFFSET), 11u);
    payload_put(p_payload, &bit,
                (uint32_t)(p_telemetry->roll + PAYLOAD_PITCH_OFFSET), 11u);
    payload_put(p_payload, &bit, p_telemetry->hump_max, 6u);
    payload_put(p_payload, &bit, p_telemetry->humps, 4u);
    payload_put(p_payload, &bit, p_telemetry->on_hump, 1u);
    payload_put(p_payload, &bit, p_telemetry->avoid_state, 3u);
    payload_put(p_payload, &bit, p_telemetry->avoid_side, 2u);
    payload_put(p_payload, &bit, p_telemetry->bypasses, 4u);
    payload_put(p_payload, &bit, p_telemetry->forward_mm, 12u);
    payload_put(p_payload, &bit, p_telemetry->faults, 15u);
    payload_put(p_payload, &bit, p_telemetry->rejects, 4u);
    payload_put(p_payload, &bit, p_telemetry->letter, 5u);
}

/*!
 * @brief Pack the scan payload from an obstacle profile.
 *
 * @param[out] p_payload PAYLOAD_SCAN_BYTES bytes.
 */
void payload_pack_scan(const scan_profile_t * p_profile, uint8_t * p_payload)
{
    uint32_t bit = 0u;

    memset(p_payload, 0, PAYLOAD_SCAN_BYTES);
    payload_put(p_payload, &bit, p_profile->closest_mm, 12u);
    payload_put(p_payload, &bit, p_profile->closest_deg, 8u);
    payload_put(p_payload, &bit, p_profile->right_deg, 8u);
    payload_put(p_payload, &bit, p_profile->left_deg, 8u);
    payload_put(p_payload, &bit, p_profile->width_mm, 8u);
    payload_put(p_payload, &bit, p_profile->left_mm, 12u);
    payload_put(p_payload, &bit, p_profile->right_mm, 12u);
}

/*!
 * @brief Unpack the map payload: one action per letter, A first.
 *
 * @param[out] p_actions BARCODE_DECODER_LETTERS values, 0..7.
 */
void payload_unpack_map(const uint8_t * p_payload, uint8_t * p_actions)
{
    uint32_t bit = 0u;
    uint32_t letter = 0u;
    uint32_t shift = 0u;

    for (letter = 0u; letter < BARCODE_DECODER_LETTERS; letter++)
    {
        p_actions[letter] = 0u;
        for (shift = 0u; shift < 3u; shift++)
        {
            p_actions[letter] =
                (uint8_t)((p_actions[letter] << 1) |
                          ((p_payload[bit / 8u] >> (7u - (bit % 8u))) & 1u));
            bit++;
        }
    }
}

/*!
 * @brief Append the low bits of a value to a zeroed payload, most
 * significant bit first.
 *
 * @param[in,out] p_bit The next free bit of the payload, moved on.
 */
static void payload_put(uint8_t * p_payload, uint32_t * p_bit, uint32_t value,
                        uint32_t bits)
{
    uint32_t shift = bits;

    while (shift > 0u)
    {
        shift--;
        if (0u != ((value >> shift) & 1u))
        {
            p_payload[*p_bit / 8u] |= (uint8_t)(0x80u >> (*p_bit % 8u));
        }
        (*p_bit)++;
    }
}

/*** end of file ***/

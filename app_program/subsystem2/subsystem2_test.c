/** @file subsystem2_test.c
 *
 * @brief On-target tests for subsystem 2 (make TEST=1). None of them moves
 * a motor; movement is tested by hand (testing/subsystem2/TEST_LOG.md).
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST
#define S2_TEST_WAIT_MS 200 /* 20 control periods. */
#define S2_TEST_TICK_US 1000u

void subsystem2_test(void)
{
    motion_control_status_t status = {0};
    uint8_t packet[MOTION_CONTROL_PACKET_BYTES] = {0};

    TEST_INIT();
    tk_dly_tsk(S2_TEST_WAIT_MS);
    motion_control_get_status(&status);
    TEST_CHECK("S2-U16",
               (MOTION_CONTROL_IDLE == status.state) &&
                   (MOTION_CONTROL_CMD_NONE == status.command),
               "boots idle with the motors braked");
    TEST_CHECK(
        "S2-U17",
        !motion_control_move_forward(0u) &&
            !motion_control_move_forward(MOTION_CONTROL_MAX_DISTANCE_MM + 1u) &&
            !motion_control_turn_left(MOTION_CONTROL_MAX_ANGLE_DEG + 1u) &&
            !motion_control_drive(MOTION_CONTROL_MAX_SPEED_MM_S + 1, 0),
        "out-of-range commands are rejected");
    motion_control_get_status(&status);
    TEST_CHECK("S2-U18", MOTION_CONTROL_IDLE == status.state,
               "a rejected command does not start the motors");
    motion_control_pack(&status, packet);
    TEST_CHECK("S2-U19",
               (MOTION_CONTROL_PACKET_BYTES == sizeof(status)) &&
                   (0u == packet[0]),
               "12-byte packed status and telemetry");
    TEST_CHECK("S2-U20",
               (subsystem2_max_tick_us() > 0u) &&
                   (subsystem2_max_tick_us() < S2_TEST_TICK_US),
               "control step runs and takes under 1 ms");
    TEST_SUMMARY("subsystem2");
}
#endif

/*** end of file ***/

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST
void subsystem2_test(void)
{
    motion_control_status_t status = {0};
    motion_control_status_t decoded = {0};
    uint8_t packet[MOTION_CONTROL_PACKET_BYTES] = {0};
    TEST_INIT();

    tk_dly_tsk(30);
    motion_control_get_status(&status);
    TEST_CHECK("S2-U01", (0u == status.armed), "boot is disarmed");
    TEST_CHECK("S2-U02", !motion_control_move_forward(1u),
               "disarmed motion is rejected");
    motion_control_pack(&status, packet);
    TEST_CHECK("S2-U03", motion_control_unpack(&decoded, packet,
               MOTION_CONTROL_PACKET_BYTES) &&
               (decoded.left_count == status.left_count) &&
               (decoded.faults == status.faults), "binary round trip");
    TEST_SUMMARY("subsystem2");
}
#endif
/* End of file. */

/* Reuse portable hardware stubs to test independent startup profiles. */
#define main motion_fixture_main
#include "test_motion.c"
#undef main
#include "../../../app_program/subsystem2/motion_defaults.h"

int main(void)
{
    motion_control_config_t config = {0};
    g_now = 0u;
    motion_control_init();
    motion_control_get_config(&config);
    CHECK(MOTION_DEFAULT_MOTOR_LEFT_REVERSE == config.motor_left_reverse);
    CHECK(MOTION_DEFAULT_MOTOR_RIGHT_REVERSE == config.motor_right_reverse);
    CHECK(MOTION_DEFAULT_ENCODER_LEFT_REVERSE == config.encoder_left_reverse);
    CHECK(MOTION_DEFAULT_ENCODER_RIGHT_REVERSE == config.encoder_right_reverse);
    CHECK(MOTION_DEFAULT_CIRCUMFERENCE_MM == config.circumference_mm);
    CHECK(MOTION_DEFAULT_TRACK_MM == config.track_mm);
    CHECK(MOTION_DEFAULT_COUNTS_PER_REV == config.counts_per_rev);
    CHECK(MOTION_DEFAULT_LEFT_COUNTS_PER_REV == config.left_counts_per_rev);
    CHECK(MOTION_DEFAULT_RIGHT_COUNTS_PER_REV == config.right_counts_per_rev);
    CHECK(MOTION_DEFAULT_CONFIRMED == config.confirmed);
    CHECK(0u == status().armed);
    CHECK(MOTION_DEFAULT_KP_Q8 == config.kp_q8);
    CHECK(!motion_control_arm()); /* Loaded defaults need a fresh tick. */
    step(0, 0);
    CHECK((0u != MOTION_DEFAULT_CONFIRMED) == motion_control_arm());
    motion_control_disarm();
    config.encoder_left_reverse = 0u;
    config.confirmed = 0u;
    CHECK(motion_control_configure(&config));
    /* Emulate a cold boot in this host fixture, not a runtime init API. */
    motion_control_init();
    motion_control_get_config(&config);
    CHECK(MOTION_DEFAULT_MOTOR_LEFT_REVERSE == config.motor_left_reverse);
    CHECK(MOTION_DEFAULT_MOTOR_RIGHT_REVERSE == config.motor_right_reverse);
    CHECK(MOTION_DEFAULT_ENCODER_LEFT_REVERSE == config.encoder_left_reverse);
    CHECK(MOTION_DEFAULT_CONFIRMED == config.confirmed);
    CHECK(0u == status().armed);
    puts("PASS defaults/reboot: no automatic arm, runtime edits not retained");
    return 0;
}

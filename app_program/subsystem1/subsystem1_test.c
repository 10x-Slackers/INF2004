/** @file subsystem1_test.c
 *
 * @brief On-target tests for subsystem 1 (make TEST=1). Record results in
 * testing/subsystem1/TEST_LOG.md. Subsystem 1 is tested on the host
 * (testing/subsystem1/host/), so there are no on-target checks.
 */

#include "../test_report.h"

#if APP_TEST
/*!
 * @brief Run subsystem 1's on-target tests.
 */
void subsystem1_test(void)
{
    TEST_INIT();
    TEST_SUMMARY("subsystem1");
}
#endif /* APP_TEST */

/*** end of file ***/

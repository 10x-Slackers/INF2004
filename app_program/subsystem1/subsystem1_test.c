/** @file subsystem1_test.c
 *
 * @brief On-target tests for subsystem 1 (make TEST=1). Record results in
 * testing/subsystem1/TEST_LOG.md. IDs: S1-U01, ...
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST
/*!
 * @brief Run subsystem 1's on-target tests.
 */
void subsystem1_test(void)
{
    TEST_INIT();

    /* TODO: add TEST_CHECK("S1-U01", condition, "description") calls here. */

    TEST_SUMMARY("subsystem1");
}
#endif /* APP_TEST */

/*** end of file ***/

/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5 (make TEST=1). Record results in
 * testing/subsystem5/TEST_LOG.md. IDs: S5-U01, ...
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST
void subsystem5_test(void)
{
    TEST_INIT();

    /* Add TEST_CHECK("S5-U01", condition, "description") calls here. */

    TEST_SUMMARY("subsystem5");
}
#endif /* APP_TEST */

/*** end of file ***/

/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3 (make TEST=1). Record results in
 * testing/subsystem3/TEST_LOG.md. IDs: S3-U01, ...
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST
void subsystem3_test(void)
{
    TEST_INIT();

    /* Add TEST_CHECK("S3-U01", condition, "description") calls here. */

    TEST_SUMMARY("subsystem3");
}
#endif /* APP_TEST */

/*** end of file ***/

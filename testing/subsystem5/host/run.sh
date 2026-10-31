#!/usr/bin/env bash
# Run from the repository root.
set -euo pipefail
gcc -std=c11 -Wall -Wextra -Werror -DS5_HOST_TEST -DAPP_TEST=1 \
    -Itesting/subsystem5/host app_program/subsystem5/s5_logic.c \
    app_program/subsystem5/subsystem5_test.c \
    testing/subsystem5/host/main.c -o /tmp/s5_host_test
/tmp/s5_host_test | tee /tmp/s5_host_test.txt
! grep -q ' FAIL ' /tmp/s5_host_test.txt

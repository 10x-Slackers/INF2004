#!/usr/bin/env bash
# Run from repository root; failures propagate to the caller.
set -euo pipefail
gcc -std=c11 -Wall -Wextra -Werror -DS5_HOST_TEST -DAPP_TEST=1 \
    -Itesting/subsystem5/host app_program/subsystem5/s5_logic.c \
    app_program/subsystem5/subsystem5_test.c \
    testing/subsystem5/host/main.c -o build_make/s5_host_test
build_make/s5_host_test | tee testing/subsystem5/logs/host-unit.txt
! grep -q ' FAIL ' testing/subsystem5/logs/host-unit.txt

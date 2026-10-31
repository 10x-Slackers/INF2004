#!/usr/bin/env bash
set -euo pipefail
gcc -std=c11 -Wall -Wextra -Werror -DS5_HOST_TEST -DTM_CONSOLE_UART=0 \
    -Itesting/subsystem5/host/hcsr04 -Itesting/subsystem5/host \
    -Iapp_program/subsystem5 app_program/subsystem5/drv_hcsr04.c \
    app_program/subsystem5/s5_logic.c \
    testing/subsystem5/host/hcsr04/driver_test.c -o build_make/s5_driver_test
build_make/s5_driver_test

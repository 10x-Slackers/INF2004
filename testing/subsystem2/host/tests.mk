# Included by the root Makefile; all test programs run on the host only.
HOST_FLAGS := -std=c99 -Wall -Wextra -Werror -pedantic -pthread $(HOST_CFLAGS)
HOST_BASE := app_program/subsystem2/ctrl_math.c app_program/subsystem2/console_motion.c
HOST_CONTROL := app_program/subsystem2/motion_control.c
HOST_DIR := build/host
HOST_GEOMETRY := -DMOTION_DEFAULT_KP_Q8=256u -DMOTION_DEFAULT_CIRCUMFERENCE_MM=200u -DMOTION_DEFAULT_TRACK_MM=100u -DMOTION_DEFAULT_COUNTS_PER_REV=1000u -DMOTION_DEFAULT_LEFT_COUNTS_PER_REV=0u -DMOTION_DEFAULT_RIGHT_COUNTS_PER_REV=0u

test:
	@mkdir -p $(HOST_DIR)
	$(HOST_CC) $(HOST_FLAGS) -O2 $(HOST_CONTROL) $(HOST_BASE) testing/subsystem2/host/test_motion.c -o $(HOST_DIR)/motion
	./$(HOST_DIR)/motion
	$(HOST_CC) $(HOST_FLAGS) -DS2_DIAGNOSTIC=1 -O2 $(HOST_CONTROL) $(HOST_BASE) testing/subsystem2/host/test_motion.c -o $(HOST_DIR)/diagnostic
	./$(HOST_DIR)/diagnostic
	@set -e; for smp in 0 1; do \
	 for test in 0 1; do for standalone in 0 1; do \
	 $(HOST_CC) $(HOST_FLAGS) -DTK_SUPPORT_SMP=$$smp -DAPP_TEST=$$test -DAPP_S2_STANDALONE=$$standalone testing/subsystem2/host/test_startup.c -o $(HOST_DIR)/startup; \
	 ./$(HOST_DIR)/startup; done; done; \
	 $(HOST_CC) $(HOST_FLAGS) -DTK_SUPPORT_SMP=$$smp testing/subsystem2/host/test_schedule.c -o $(HOST_DIR)/schedule; \
	 ./$(HOST_DIR)/schedule; done
	@set -e; for profile in 0 1 2; do \
	 flags=''; if [ $$profile -gt 0 ]; then flags='$(HOST_GEOMETRY)'; fi; \
	 if [ $$profile -eq 2 ]; then flags="$$flags -DMOTION_DEFAULT_CONFIRMED=1u"; fi; \
	 $(HOST_CC) $(HOST_FLAGS) $$flags $(HOST_CONTROL) $(HOST_BASE) testing/subsystem2/host/test_defaults.c -o $(HOST_DIR)/defaults; \
	 ./$(HOST_DIR)/defaults; done
	@set -e; for value in LEFT_COUNTS_PER_REV=65536u COUNTS_PER_REV=65536u ENCODER_LEFT_REVERSE=2u SPEED_MM_S=0u; do \
	 if printf '\043include "app_program/subsystem2/motion_defaults.h"\n' | $(HOST_CC) $(HOST_FLAGS) -DMOTION_DEFAULT_$$value -x c -fsyntax-only - >$(HOST_DIR)/invalid.txt 2>&1; then \
	 cat $(HOST_DIR)/invalid.txt; exit 1; fi; done
	@set -e; for flags in '-DMOTION_DEFAULT_CONFIRMED=1u -DMOTION_DEFAULT_LEFT_COUNTS_PER_REV=1u' '-DMOTION_DEFAULT_CONFIRMED=1u -DMOTION_DEFAULT_LEFT_COUNTS_PER_REV=0u -DMOTION_DEFAULT_RIGHT_COUNTS_PER_REV=0u'; do \
	 if printf '\043include "app_program/subsystem2/motion_defaults.h"\n' | $(HOST_CC) $(HOST_FLAGS) $$flags -x c -fsyntax-only - >$(HOST_DIR)/invalid.txt 2>&1; then exit 1; fi; done
	$(PYTHON) testing/subsystem2/host/test_irq.py
	$(PYTHON) testing/subsystem2/host/compare_representation.py
	$(PYTHON) testing/subsystem2/host/check_source.py

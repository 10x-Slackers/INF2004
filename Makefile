# Native Pico W build; inherited kernel rules live in build_make/.
.DEFAULT_GOAL := all
ROOT := $(abspath .)
-include config/local.mk
PICO_SDK_PATH ?= $(ROOT)/.pico-sdk
export PICO_SDK_PATH
SMP ?= 1
TEST ?= 0
S2_ONLY ?= 0
CONSOLE ?= usb_cdc
WIFI ?= cyw43
NET ?= none
MQTT ?= 0
CRASH_REPORT ?= 0
PROFILE := smp$(SMP)-$(CONSOLE)-$(WIFI)-$(NET)-mqtt$(MQTT)-crash$(CRASH_REPORT)-test$(TEST)$(if $(filter 1,$(S2_ONLY)),-s2only)
BUILD_DIR := $(ROOT)/build/$(PROFILE)
PYTHON ?= python3
HOST_CC ?= cc
export HOST_CC HOST_CFLAGS
HOST_FLAGS := -std=c99 -Wall -Wextra -Werror -pedantic -pthread
HOST_SOURCES := app_program/subsystem2/motion_control.c \
 testing/subsystem2/host/test_motion.c

.PHONY: all test clean help
all:
	@mkdir -p "$(BUILD_DIR)"
	$(MAKE) -C "$(BUILD_DIR)" -f "$(ROOT)/build_make/makefile" \
	 ROOT="$(ROOT)" SMP=$(SMP) TEST=$(TEST) S2_ONLY=$(S2_ONLY) CONSOLE=$(CONSOLE) WIFI=$(WIFI) \
	 NET=$(NET) MQTT=$(MQTT) CRASH_REPORT=$(CRASH_REPORT)

test:
	@mkdir -p build/host
	$(HOST_CC) $(HOST_FLAGS) $(HOST_CFLAGS) -O2 $(HOST_SOURCES) -lm \
	 -o build/host/test_motion
	./build/host/test_motion

clean:
	$(PYTHON) -c 'import shutil; shutil.rmtree("build", ignore_errors=True)'

help:
	@echo 'make       Build Pico W SMP firmware with USB CDC and Wi-Fi'
	@echo 'make test  Run the Buddy 2 motion host test (simulated robot)'
	@echo 'make clean Remove only generated files under build/'
	@echo 'PICO_SDK_PATH=/path/to/pico-sdk (default: .pico-sdk)'
	@echo 'Optional config/local.mk: PICO_SDK_PATH := /path/to/pico-sdk'
	@echo 'Profiles: TEST=1 SMP=0 CONSOLE=uart WIFI=none NET=lwip MQTT=1'
	@echo 'S2_ONLY=1  Bench image that starts only subsystem 2 (motion)'
	@echo 'Outputs: build/<profile>/mtk3pico_<options>.{elf,map,uf2}'

# Native Pico W build; inherited kernel rules live in build_make/.
.DEFAULT_GOAL := all
ROOT := $(abspath .)
-include config/local.mk
PICO_SDK_PATH ?= $(ROOT)/.pico-sdk
export PICO_SDK_PATH
SMP ?= 1
TEST ?= 0
CONSOLE ?= usb_cdc
WIFI ?= cyw43
NET ?= none
MQTT ?= 0
CRASH_REPORT ?= 0
PROFILE := smp$(SMP)-$(CONSOLE)-$(WIFI)-$(NET)-mqtt$(MQTT)-crash$(CRASH_REPORT)-test$(TEST)
BUILD_DIR := $(ROOT)/build/$(PROFILE)
PYTHON ?= python3
HOST_CC ?= cc
export HOST_CC HOST_CFLAGS
HOST_FLAGS := -std=c99 -Wall -Wextra -Werror -pedantic -pthread
HOST_SOURCES := app_program/subsystem2/motion_control.c \
 app_program/subsystem2/ctrl_math.c \
 app_program/subsystem2/console_motion.c testing/subsystem2/host/test_motion.c

.PHONY: all test clean help
all:
	@mkdir -p "$(BUILD_DIR)"
	$(MAKE) -C "$(BUILD_DIR)" -f "$(ROOT)/build_make/makefile" \
	 ROOT="$(ROOT)" SMP=$(SMP) TEST=$(TEST) CONSOLE=$(CONSOLE) WIFI=$(WIFI) \
	 NET=$(NET) MQTT=$(MQTT) CRASH_REPORT=$(CRASH_REPORT)

test:
	@mkdir -p build/host
	$(HOST_CC) $(HOST_FLAGS) $(HOST_CFLAGS) -O2 $(HOST_SOURCES) \
	 -o build/host/test_motion
	./build/host/test_motion
	$(PYTHON) testing/subsystem2/host/compare_representation.py
	$(PYTHON) testing/subsystem2/host/check_source.py

clean:
	$(PYTHON) -c 'import shutil; shutil.rmtree("build", ignore_errors=True)'

help:
	@echo 'make       Build Pico W SMP firmware with USB CDC and Wi-Fi'
	@echo 'make test  Run C99 host tests and Buddy 2 source checks'
	@echo 'make clean Remove only generated files under build/'
	@echo 'PICO_SDK_PATH=/path/to/pico-sdk (default: .pico-sdk)'
	@echo 'Optional config/local.mk: PICO_SDK_PATH := /path/to/pico-sdk'
	@echo 'Profiles: TEST=1 SMP=0 CONSOLE=uart WIFI=none NET=lwip MQTT=1'
	@echo 'Outputs: build/<profile>/mtk3pico_<options>.{elf,map,uf2}'

# Native Pico W build; inherited kernel rules live in build_make/.
.DEFAULT_GOAL := all
ROOT := $(abspath .)
-include config/local.mk
PICO_SDK_PATH ?= $(ROOT)/.pico-sdk
export PICO_SDK_PATH
SMP ?= 1
TEST ?= 0
S2_STANDALONE ?= 0
S2_DIAGNOSTIC ?= 0
CONSOLE ?= usb_cdc
WIFI ?= $(if $(filter 1,$(S2_STANDALONE)),none,cyw43)
NET ?= none
MQTT ?= 0
CRASH_REPORT ?= 0
PROFILE := smp$(SMP)-$(CONSOLE)-$(WIFI)-$(NET)-mqtt$(MQTT)-crash$(CRASH_REPORT)-test$(TEST)$(if $(filter 1,$(S2_STANDALONE)),-s2only,)
PROFILE := $(PROFILE)$(if $(filter 1,$(S2_DIAGNOSTIC)),-s2diag,)
BUILD_DIR := $(ROOT)/build/$(PROFILE)
PYTHON ?= python3
HOST_CC ?= cc
export HOST_CC HOST_CFLAGS
.PHONY: all test diagnostic verify clean help
all:
	@mkdir -p "$(BUILD_DIR)"
	$(MAKE) -C "$(BUILD_DIR)" -f "$(ROOT)/build_make/makefile" \
	 ROOT="$(ROOT)" SMP=$(SMP) TEST=$(TEST) S2_STANDALONE=$(S2_STANDALONE) \
	 S2_DIAGNOSTIC=$(S2_DIAGNOSTIC) \
	 CONSOLE=$(CONSOLE) WIFI=$(WIFI) \
	 NET=$(NET) MQTT=$(MQTT) CRASH_REPORT=$(CRASH_REPORT)

diagnostic:
	$(MAKE) S2_STANDALONE=1 S2_DIAGNOSTIC=1 TEST=1 all

verify:
	+@bash -o pipefail -c 'set -e; { \
	 shasum -a 256 app_program/subsystem2/*.[ch] testing/subsystem2/host/*.[ch] testing/subsystem2/host/*.py testing/subsystem2/host/tests.mk Makefile build_make/pico_rp2040.mk build_make/mtkernel_3/app_program/subdir.mk; \
	 echo "Host C and port tests"; $(MAKE) -s test && \
	 echo "AddressSanitizer and UndefinedBehaviorSanitizer (-fsanitize=address,undefined)" && $(MAKE) -s test HOST_CFLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all" && \
	 echo "Integrated normal and TEST builds" && $(MAKE) -s -j8 && $(MAKE) -s TEST=1 -j8 && \
	 echo "Standalone normal and TEST builds" && $(MAKE) -s S2_STANDALONE=1 -j8 && \
	 $(MAKE) -s S2_STANDALONE=1 TEST=1 -j8 && \
	 echo "Single diagnostic build" && $(MAKE) -s diagnostic -j8 && \
	 echo "Direct build_make normal and TEST builds" && $(MAKE) -s -C build_make -j8 && \
	 $(MAKE) -s -C build_make TEST=1 -j8; \
	 } 2>&1 | tee testing/subsystem2/current.txt'

clean:
	rm -rf build

help:
	@echo 'make       Build Pico W SMP firmware with USB CDC and Wi-Fi'
	@echo 'make test  Run C99 host tests and Buddy 2 source checks'
	@echo 'make clean Remove only generated files under build/'
	@echo 'PICO_SDK_PATH=/path/to/pico-sdk (default: .pico-sdk)'
	@echo 'Optional config/local.mk: PICO_SDK_PATH := /path/to/pico-sdk'
	@echo 'Profiles: TEST=1 SMP=0 CONSOLE=uart WIFI=none NET=lwip MQTT=1'
	@echo 'S2_STANDALONE=1: Buddy 2 only, USB CDC, Wi-Fi off; TEST=1 adds only S2 tests'
	@echo 'Standalone profiles/images add -s2only / _s2only'
	@echo 'make diagnostic: one standalone diagnostic image, explicit arm required'
	@echo 'make verify: tests/sanitizers/builds; overwrite testing/subsystem2/current.txt'
	@echo 'Outputs: build/<profile>/mtk3pico_<options>.{elf,map,uf2}'

export PYTHONDONTWRITEBYTECODE := 1
include testing/subsystem2/host/tests.mk
